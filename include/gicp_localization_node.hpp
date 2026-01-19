// SPDX-FileCopyrightText: Copyright 2025 FAST-LIVO2
// SPDX-License-Identifier: MIT
#ifndef GICP_LOCALIZATION_NODE_HPP
#define GICP_LOCALIZATION_NODE_HPP

// C++ Standard Library
#include <memory>
#include <mutex>
#include <vector>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// ROS2 Core
#include <rclcpp/rclcpp.hpp>

// ROS2 Message Types
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

namespace fast_livo {

// Configuration parameters for GICP localization
struct GICPLocConfig {
  std::string prior_pcd_path = "";
  double search_radius = 50.0;
  double downsampling_resolution = 0.5;
  double max_correspondence_distance = 1.0;
  int max_iterations = 20;
  int num_threads = 4;
  bool publish_local_map = true;
  bool verbose = true;
};

// GICP registration result
struct GICPLocResult {
  bool success = false;
  bool converged = false;
  Eigen::Matrix3d rot_refined;
  Eigen::Vector3d pos_refined;
  double alignment_error = 0.0;
  size_t num_inliers = 0;
  size_t iterations = 0;
  double computation_time = 0.0;
  std::string message;
};

class GICPLocalizationNode : public rclcpp::Node {
public:
  explicit GICPLocalizationNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~GICPLocalizationNode() = default;

private:
  // Configuration
  GICPLocConfig config_;

  // Prior point cloud (PCL format for loading)
  pcl::PointCloud<pcl::PointXYZI>::Ptr prior_pcd_;
  std::vector<Eigen::Vector4f> prior_points_;

  // Initial pose guess
  Eigen::Matrix3d init_rot_;
  Eigen::Vector3d init_pos_;
  bool pose_received_ = false;

  // Subscribers
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_scan_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_prior_pcd_;     // Latched prior map
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_map_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_aligned_scan_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_source_scan_;  // GICP input: source scan
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_target_map_;  // GICP input: target map
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_result_;

  // Service
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_relocalize_;

  // State
  bool localization_done_ = false;
  int scan_count_ = 0;
  std::mutex mutex_;
  
  // Accumulated point clouds for multi-frame registration
  pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_scan_;
  int accumulated_frame_count_ = 0;
  static constexpr int ACCUMULATION_FRAMES = 10;

  // Callback functions
  void scanCallback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
  void initPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void relocalizeService(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  // Core functionality
  bool loadPriorPCD(const std::string& pcd_path);
  pcl::PointCloud<pcl::PointXYZI>::Ptr extractLocalMap(
    const Eigen::Vector3d& center, double radius);
  std::vector<Eigen::Vector4f> pclToSmallGICP(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  pcl::PointCloud<pcl::PointXYZI>::Ptr convertLivoxToPCL(
    const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
  GICPLocResult performGICP(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source_scan);
  void publishResult(const GICPLocResult& result);
  void publishPriorPCD();
  void publishLocalMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& local_map);
  void publishAlignedScan(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const Eigen::Matrix3d& rot,
    const Eigen::Vector3d& pos);
  void publishSourceScan(const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan);
  void publishTargetMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr& target_map);
  pcl::PointCloud<pcl::PointXYZI>::Ptr downsamplePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, double resolution);

  // Parameter handling
  void declareParameters();
  void loadParameters();
};

}  // namespace fast_livo

#endif  // GICP_LOCALIZATION_NODE_HPP

