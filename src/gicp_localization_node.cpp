// SPDX-FileCopyrightText: Copyright 2025 FAST-LIVO2
// SPDX-License-Identifier: MIT

// Project Headers
#include "gicp_localization_node.hpp"

// C++ Standard Library
#include <chrono>
#include <sstream>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

// Eigen
#include <Eigen/Dense>

// OpenMP
#include <omp.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// small_gicp
#include <small_gicp/registration/registration_helper.hpp>

namespace fast_livo {

GICPLocalizationNode::GICPLocalizationNode(const rclcpp::NodeOptions& options)
    : Node("gicp_localization_node", options)
{
  declareParameters();
  loadParameters();

  // Initialize prior point cloud
  prior_pcd_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  if (!config_.prior_pcd_path.empty()) {
    if (loadPriorPCD(config_.prior_pcd_path)) {
      RCLCPP_INFO(this->get_logger(), "Prior PCD loaded successfully from: %s",
                  config_.prior_pcd_path.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load prior PCD from: %s",
                   config_.prior_pcd_path.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No prior PCD path specified. Set prior_pcd_path parameter.");
  }

  // Initialize pose to identity
  init_rot_ = Eigen::Matrix3d::Identity();
  init_pos_ = Eigen::Vector3d::Zero();
  
  // Initialize accumulated point cloud
  accumulated_scan_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  accumulated_frame_count_ = 0;

  // Create subscribers
  sub_scan_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
    "/livox/lidar",
    rclcpp::SensorDataQoS(),
    std::bind(&GICPLocalizationNode::scanCallback, this, std::placeholders::_1)
  );

  sub_init_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_guess",
    rclcpp::QoS(10),
    std::bind(&GICPLocalizationNode::initPoseCallback, this, std::placeholders::_1)
  );

  // Create publishers
  pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "/initial_pose_refined",
    rclcpp::QoS(10)
  );

  // Prior PCD publisher with transient_local (latched) for late subscribers
  rclcpp::QoS prior_pcd_qos(rclcpp::KeepLast(1));
  prior_pcd_qos.reliable();
  prior_pcd_qos.transient_local();
  pub_prior_pcd_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/prior_map", prior_pcd_qos);

  pub_local_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/local_map",
    rclcpp::QoS(1)
  );
  pub_aligned_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/aligned_scan",
    rclcpp::QoS(1)
  );
  pub_source_scan_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/gicp_source_scan",
    rclcpp::QoS(1)
  );
  pub_target_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/gicp_target_map",
    rclcpp::QoS(1)
  );
  pub_result_ = this->create_publisher<std_msgs::msg::String>(
    "/gicp_localization_result",
    rclcpp::QoS(10)
  );

  // Create service
  srv_relocalize_ = this->create_service<std_srvs::srv::Trigger>(
    "/trigger_relocalization",
    std::bind(
      &GICPLocalizationNode::relocalizeService,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    )
  );

  RCLCPP_INFO(this->get_logger(), "GICP Localization Node initialized");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: /livox/lidar");
  RCLCPP_INFO(this->get_logger(), "  Subscribing to: /initial_pose_guess");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /prior_map (latched)");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /initial_pose_refined");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /local_map");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /aligned_scan");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /gicp_source_scan");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /gicp_target_map");
  RCLCPP_INFO(this->get_logger(), "  Publishing to: /gicp_localization_result");
  RCLCPP_INFO(this->get_logger(), "  Service: /trigger_relocalization");

  // Publish prior PCD after all publishers are created
  if (!prior_pcd_->points.empty()) {
    publishPriorPCD();
  }
}

void GICPLocalizationNode::declareParameters() {
  this->declare_parameter<std::string>("prior_pcd_path", "");
  this->declare_parameter<double>("search_radius", 3.0);
  this->declare_parameter<double>("downsampling_resolution", 0.1);
  this->declare_parameter<double>("max_correspondence_distance", 1.0);
  this->declare_parameter<int>("max_iterations", 20);
  this->declare_parameter<int>("num_threads", 4);
  this->declare_parameter<bool>("publish_local_map", true);
  this->declare_parameter<bool>("verbose", true);
}

void GICPLocalizationNode::loadParameters() {
  this->get_parameter("prior_pcd_path", config_.prior_pcd_path);
  this->get_parameter("search_radius", config_.search_radius);
  this->get_parameter("downsampling_resolution", config_.downsampling_resolution);
  this->get_parameter("max_correspondence_distance", config_.max_correspondence_distance);
  this->get_parameter("max_iterations", config_.max_iterations);
  this->get_parameter("num_threads", config_.num_threads);
  this->get_parameter("publish_local_map", config_.publish_local_map);
  this->get_parameter("verbose", config_.verbose);

  if (config_.verbose) {
    RCLCPP_INFO(this->get_logger(), "GICP Localization Parameters:");
    RCLCPP_INFO(this->get_logger(), "  prior_pcd_path: %s", config_.prior_pcd_path.c_str());
    RCLCPP_INFO(this->get_logger(), "  search_radius: %.2f m", config_.search_radius);
    RCLCPP_INFO(this->get_logger(), "  downsampling_resolution: %.2f m", config_.downsampling_resolution);
    RCLCPP_INFO(this->get_logger(), "  max_correspondence_distance: %.2f m", config_.max_correspondence_distance);
    RCLCPP_INFO(this->get_logger(), "  max_iterations: %d", config_.max_iterations);
    RCLCPP_INFO(this->get_logger(), "  num_threads: %d", config_.num_threads);
  }
}

bool GICPLocalizationNode::loadPriorPCD(const std::string& pcd_path) {
  if (pcd_path.empty()) {
    RCLCPP_WARN(this->get_logger(), "PCD path is empty");
    return false;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_path, *prior_pcd_) == -1) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", pcd_path.c_str());
    return false;
  }

  RCLCPP_INFO(this->get_logger(), "Loaded prior PCD with %zu points", prior_pcd_->points.size());

  // Convert to small_gicp format for faster processing
  prior_points_ = pclToSmallGICP(prior_pcd_);

  return true;
}

std::vector<Eigen::Vector4f> GICPLocalizationNode::pclToSmallGICP(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  std::vector<Eigen::Vector4f> points;
  points.reserve(cloud->points.size());

  for (const auto& pt : cloud->points) {
    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      points.emplace_back(pt.x, pt.y, pt.z, 1.0f);
    }
  }

  return points;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GICPLocalizationNode::extractLocalMap(
  const Eigen::Vector3d& center, double radius)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  const double radius_sq = radius * radius;

  for (const auto& pt : prior_pcd_->points) {
    if (std::isnan(pt.x) || std::isnan(pt.y) || std::isnan(pt.z)) {
      continue;
    }
    double dx = pt.x - center.x();
    double dy = pt.y - center.y();
    double dz = pt.z - center.z();
    if ((dx * dx + dy * dy + dz * dz) <= radius_sq) {
      local_cloud->points.push_back(pt);
    }
  }

  if (!local_cloud->points.empty()) {
    local_cloud->width = local_cloud->points.size();
    local_cloud->height = 1;
    local_cloud->is_dense = false;
  }

  return local_cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GICPLocalizationNode::convertLivoxToPCL(
  const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  int plsize = msg->point_num;
  cloud->reserve(plsize);

  // Parameters matching MID-360
  const int N_SCANS = 4;  // Default scan lines
  const int point_filter_num = 1;  // Default: no filtering
  const double blind = 0.01;  // Default blind zone
  const double blind_sqr = blind * blind;

  uint valid_num = 0;
  uint points_added = 0;

  for (uint i = 0; i < static_cast<uint>(plsize); i++)
  {
    // Check if point line is within valid range (matching avia_handler line 168)
    // Note: tag check is commented out in avia_handler when feature_enabled=false
    if (msg->points[i].line < N_SCANS)
    {
      valid_num++;

      // Apply point filter (matching avia_handler line 188)
      if (valid_num % point_filter_num == 0)
      {
        // Check blind zone (matching avia_handler line 190)
        double dist_sq = msg->points[i].x * msg->points[i].x + 
                         msg->points[i].y * msg->points[i].y + 
                         msg->points[i].z * msg->points[i].z;
        
        if (dist_sq >= blind_sqr)
        {
          pcl::PointXYZI point;
          point.x = msg->points[i].x;
          point.y = msg->points[i].y;
          point.z = msg->points[i].z;
          point.intensity = msg->points[i].reflectivity;
          cloud->points.push_back(point);
          points_added++;
        }
      }
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;
  cloud->is_dense = false;
  
  RCLCPP_DEBUG(this->get_logger(), 
    "Point cloud conversion: input=%d, valid_lines=%u, output=%zu", 
    plsize, valid_num, cloud->points.size());
  
  return cloud;
}

void GICPLocalizationNode::scanCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  scan_count_++;

  // Wait for initial pose guess
  if (!pose_received_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      5000,
      "Waiting for initial pose guess on /initial_pose_guess topic"
    );
    return;
  }

  // Only perform localization once (can be retriggered via service)
  if (localization_done_) {
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Scan #%d received", scan_count_);
  RCLCPP_INFO(this->get_logger(), "Input point cloud size: %d", msg->point_num);

  // Convert Livox CustomMsg to PCL
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan = convertLivoxToPCL(msg);

  if (scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Empty scan received, skipping");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Converted point cloud size: %zu", scan->points.size());

  // Accumulate point clouds
  *accumulated_scan_ += *scan;
  accumulated_frame_count_++;
  
  RCLCPP_INFO(this->get_logger(), "Accumulated frames: %d/%d, Total points: %zu", 
              accumulated_frame_count_, ACCUMULATION_FRAMES, accumulated_scan_->points.size());

  // Wait until we have accumulated enough frames
  if (accumulated_frame_count_ < ACCUMULATION_FRAMES) {
    RCLCPP_INFO(this->get_logger(), "Waiting for more frames... (%d/%d)", 
                accumulated_frame_count_, ACCUMULATION_FRAMES);
    return;
  }

  // Perform downsampling on accumulated point cloud
  RCLCPP_INFO(this->get_logger(), "Downsampling accumulated point cloud (resolution: 0.1 m)...");
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_scan = downsamplePointCloud(accumulated_scan_, 0.1);
  
  RCLCPP_INFO(this->get_logger(), "Downsampled point cloud size: %zu (from %zu)", 
              downsampled_scan->points.size(), accumulated_scan_->points.size());

  if (downsampled_scan->points.empty()) {
    RCLCPP_WARN(this->get_logger(), "Downsampled point cloud is empty, resetting accumulation");
    accumulated_scan_->clear();
    accumulated_frame_count_ = 0;
    return;
  }

  // Perform GICP registration with downsampled accumulated scan
  RCLCPP_INFO(this->get_logger(), "Performing GICP localization with accumulated and downsampled scan...");
  GICPLocResult result = performGICP(downsampled_scan);

  // Publish results
  publishResult(result);

  // Reset accumulation for next attempt
  accumulated_scan_->clear();
  accumulated_frame_count_ = 0;

  if (result.success) {
    localization_done_ = true;
    RCLCPP_INFO(this->get_logger(),
      "Localization SUCCESS! Use /trigger_relocalization service to redo.");
  } else {
    RCLCPP_WARN(this->get_logger(), "Localization FAILED: %s", result.message.c_str());
    RCLCPP_WARN(this->get_logger(),
      "You can retry with a different initial pose via /initial_pose_guess topic");
  }
}

GICPLocResult GICPLocalizationNode::performGICP(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_scan)
{
  GICPLocResult result;
  auto start_time = omp_get_wtime();

  // Check if prior map is loaded
  if (prior_pcd_->points.empty()) {
    result.message = "Prior PCD not loaded. Check prior_pcd_path parameter.";
    RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
    return result;
  }

  // Extract local map around the initial pose
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_map = extractLocalMap(init_pos_, config_.search_radius);

  // Publish source scan (always available, before registration)
  publishSourceScan(source_scan);

  if (local_map->points.size() < 100) {
    result.message = "Insufficient points in local map: " +
                    std::to_string(local_map->points.size()) +
                    " (minimum: 100). Check search_radius or initial pose.";
    RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
    // Still publish target map even if insufficient (for debugging)
    publishTargetMap(local_map);
    return result;
  }

  RCLCPP_INFO(this->get_logger(), "Local map extracted: %zu points (search_radius=%.2f m)", 
              local_map->points.size(), config_.search_radius);

  // Publish local map for visualization
  if (config_.publish_local_map) {
    publishLocalMap(local_map);
  }

  // Convert to small_gicp format
  std::vector<Eigen::Vector4f> target_points = pclToSmallGICP(local_map);
  std::vector<Eigen::Vector4f> source_points = pclToSmallGICP(source_scan);

  RCLCPP_INFO(this->get_logger(), "Point clouds after conversion:");
  RCLCPP_INFO(this->get_logger(), "  Target (local map): %zu points", target_points.size());
  RCLCPP_INFO(this->get_logger(), "  Source (scan): %zu points", source_points.size());

  // Publish target map (before registration)
  publishTargetMap(local_map);

  if (source_points.size() < 50) {
    result.message = "Insufficient points in source scan: " +
                    std::to_string(source_points.size()) +
                    " (minimum: 50)";
    RCLCPP_ERROR(this->get_logger(), "%s", result.message.c_str());
    // Note: No aligned scan published here since registration was not performed
    return result;
  }

  // Construct initial guess
  Eigen::Isometry3d init_T = Eigen::Isometry3d::Identity();
  init_T.linear() = init_rot_;
  init_T.translation() = init_pos_;

  RCLCPP_INFO(this->get_logger(), "Initial pose guess:");
  RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]", 
              init_pos_.x(), init_pos_.y(), init_pos_.z());
  RCLCPP_INFO(this->get_logger(), "  Search radius: %.2f m", config_.search_radius);

  // Configure GICP registration
  small_gicp::RegistrationSetting setting;
  setting.type = small_gicp::RegistrationSetting::GICP;
  setting.downsampling_resolution = config_.downsampling_resolution;
  setting.max_correspondence_distance = config_.max_correspondence_distance;
  setting.max_iterations = config_.max_iterations;
  setting.num_threads = config_.num_threads;
  setting.verbose = config_.verbose;

  RCLCPP_INFO(this->get_logger(), "GICP settings:");
  RCLCPP_INFO(this->get_logger(), "  Downsampling resolution: %.3f m", setting.downsampling_resolution);
  RCLCPP_INFO(this->get_logger(), "  Max correspondence distance: %.3f m", setting.max_correspondence_distance);
  RCLCPP_INFO(this->get_logger(), "  Max iterations: %d", setting.max_iterations);

  // Perform GICP registration
  RCLCPP_INFO(this->get_logger(), "Running GICP registration...");
  small_gicp::RegistrationResult gicp_result =
    small_gicp::align(target_points, source_points, init_T, setting);

  // Process results
  result.converged = gicp_result.converged;
  result.num_inliers = gicp_result.num_inliers;
  result.iterations = gicp_result.iterations;
  result.alignment_error = gicp_result.error;
  result.rot_refined = gicp_result.T_target_source.linear();
  result.pos_refined = gicp_result.T_target_source.translation();
  result.computation_time = omp_get_wtime() - start_time;

  RCLCPP_INFO(this->get_logger(), "GICP Registration Results:");
  RCLCPP_INFO(this->get_logger(), "  Converged: %s", result.converged ? "YES" : "NO");
  RCLCPP_INFO(this->get_logger(), "  Error: %.4f m", result.alignment_error);
  RCLCPP_INFO(this->get_logger(), "  Inliers: %zu", result.num_inliers);
  RCLCPP_INFO(this->get_logger(), "  Iterations: %zu", result.iterations);
  RCLCPP_INFO(this->get_logger(), "  Time: %.2f ms", result.computation_time * 1000.0);
  RCLCPP_INFO(this->get_logger(), "  Position: [%.3f, %.3f, %.3f]",
              result.pos_refined.x(), result.pos_refined.y(), result.pos_refined.z());

  // Validate results
  bool converged = result.converged;
  bool enough_inliers = result.num_inliers >= 50;
  bool low_error = result.alignment_error < config_.max_correspondence_distance * 2.0;

  // Publish aligned scan for visualization (regardless of success)
  // This shows the source scan transformed by the registration result
  publishAlignedScan(source_scan, result.rot_refined, result.pos_refined);

  if (converged && enough_inliers && low_error) {
    result.success = true;
    result.message = "Localization successful";
  } else {
    result.success = false;
    std::stringstream ss;
    ss << "Localization validation failed - ";
    ss << "Converged: " << (converged ? "yes" : "no") << ", ";
    ss << "Inliers: " << result.num_inliers << " (>= 50: " << (enough_inliers ? "yes" : "no") << "), ";
    ss << "Error: " << result.alignment_error << " m (< " << (config_.max_correspondence_distance * 2.0) << ": " << (low_error ? "yes" : "no") << ")";
    result.message = ss.str();
  }

  return result;
}

void GICPLocalizationNode::initPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Extract position
  init_pos_(0) = msg->pose.pose.position.x;
  init_pos_(1) = msg->pose.pose.position.y;
  init_pos_(2) = msg->pose.pose.position.z;

  // Extract orientation
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w
  );
  tf2::Matrix3x3 m(q);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      init_rot_(i, j) = m[i][j];
    }
  }

  pose_received_ = true;

  RCLCPP_INFO(this->get_logger(), "Initial pose received:");
  RCLCPP_INFO(this->get_logger(), "  Position: [%.2f, %.2f, %.2f]",
              init_pos_.x(), init_pos_.y(), init_pos_.z());

  // Convert to Euler angles for display
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
  RCLCPP_INFO(this->get_logger(), "  Orientation (RPY): [%.2f, %.2f, %.2f] rad", roll, pitch, yaw);
}

void GICPLocalizationNode::publishResult(const GICPLocResult& result) {
  // Publish refined pose
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = this->now();
  pose_msg.header.frame_id = "map";

  pose_msg.pose.pose.position.x = result.pos_refined.x();
  pose_msg.pose.pose.position.y = result.pos_refined.y();
  pose_msg.pose.pose.position.z = result.pos_refined.z();

  Eigen::Quaterniond q(result.rot_refined);
  pose_msg.pose.pose.orientation.w = q.w();
  pose_msg.pose.pose.orientation.x = q.x();
  pose_msg.pose.pose.orientation.y = q.y();
  pose_msg.pose.pose.orientation.z = q.z();

  pub_pose_->publish(pose_msg);

  // Publish result as JSON string
  std_msgs::msg::String result_msg;
  std::stringstream ss;
  ss << "{";
  ss << "\"success\": " << (result.success ? "true" : "false") << ",";
  ss << "\"converged\": " << (result.converged ? "true" : "false") << ",";
  ss << "\"error\": " << result.alignment_error << ",";
  ss << "\"inliers\": " << result.num_inliers << ",";
  ss << "\"iterations\": " << result.iterations << ",";
  ss << "\"time_ms\": " << (result.computation_time * 1000.0) << ",";
  ss << "\"position\": [" << result.pos_refined.x() << ", "
                       << result.pos_refined.y() << ", "
                       << result.pos_refined.z() << "],";
  ss << "\"message\": \"" << result.message << "\"";
  ss << "}";
  result_msg.data = ss.str();
  pub_result_->publish(result_msg);
}

void GICPLocalizationNode::publishPriorPCD() {
  if (prior_pcd_->points.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*prior_pcd_, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  pub_prior_pcd_->publish(msg);

  RCLCPP_INFO(this->get_logger(), "Published prior map with %zu points to /prior_map (latched)",
              prior_pcd_->points.size());
}

void GICPLocalizationNode::publishLocalMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& local_map)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*local_map, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  pub_local_map_->publish(msg);
}

void GICPLocalizationNode::publishAlignedScan(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
  const Eigen::Matrix3d& rot,
  const Eigen::Vector3d& pos)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZI>());

  // Construct 4x4 transformation matrix
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = rot;
  transform.block<3, 1>(0, 3) = pos;

  pcl::transformPointCloud(*scan, *aligned, transform);

  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*aligned, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";
  pub_aligned_scan_->publish(msg);
}

void GICPLocalizationNode::publishSourceScan(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*scan, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = "livox";  // Source scan is in livox frame
  pub_source_scan_->publish(msg);
}

void GICPLocalizationNode::publishTargetMap(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_map)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*target_map, msg);
  msg.header.stamp = this->now();
  msg.header.frame_id = "map";  // Target map is in map frame
  pub_target_map_->publish(msg);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr GICPLocalizationNode::downsamplePointCloud(
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double resolution)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
  
  if (cloud->points.empty()) {
    return downsampled;
  }

  pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
  voxel_filter.setInputCloud(cloud);
  voxel_filter.setLeafSize(resolution, resolution, resolution);
  voxel_filter.filter(*downsampled);

  return downsampled;
}

void GICPLocalizationNode::relocalizeService(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  (void)req;
  std::lock_guard<std::mutex> lock(mutex_);
  localization_done_ = false;
  // Reset accumulation when relocalization is triggered
  accumulated_scan_->clear();
  accumulated_frame_count_ = 0;
  res->success = true;
  res->message = "Relocalization triggered. Waiting for next scan...";
  RCLCPP_INFO(this->get_logger(), "Relocalization triggered via service");
}

}  // namespace fast_livo

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fast_livo::GICPLocalizationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
