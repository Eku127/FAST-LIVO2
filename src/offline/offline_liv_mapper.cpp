/*
 * Offline LIV Mapper for FAST-LIVO2
 * Implementation - reuses core LIVO2 components
 */

#include "offline/offline_liv_mapper.h"

#include <filesystem>
#include <iostream>
#include <iomanip>
#include <fstream>

#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>

namespace livo2_offline {

OfflineLIVMapper::OfflineLIVMapper(const std::string& config_path) {
    initFromYaml(config_path);
    initComponents();
}

OfflineLIVMapper::OfflineLIVMapper(const OfflineConfig& config, const std::string& yaml_config_path)
    : offline_config_(config) {
    initFromYaml(yaml_config_path);
    initComponents();
}

OfflineLIVMapper::~OfflineLIVMapper() {
    // Clean up voxel map
    for (auto& pair : voxel_map_) {
        if (pair.second) {
            delete pair.second;
        }
    }
    voxel_map_.clear();
}

void OfflineLIVMapper::initFromYaml(const std::string& yaml_path) {
    YAML::Node config;
    try {
        config = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        std::cerr << "[OfflineLIVMapper] Failed to load config: " << yaml_path 
                  << "\nError: " << e.what() << std::endl;
        throw;
    }
    
    // Navigate to ros__parameters if present (ROS2 param format)
    YAML::Node params = config;
    if (config["/**"] && config["/**"]["ros__parameters"]) {
        params = config["/**"]["ros__parameters"];
    }
    
    // Common settings
    if (params["common"]) {
        auto common = params["common"];
        if (common["lid_topic"]) offline_config_.lidar_topic = common["lid_topic"].as<std::string>();
        if (common["imu_topic"]) offline_config_.imu_topic = common["imu_topic"].as<std::string>();
    }
    
    // Preprocess settings
    if (params["preprocess"]) {
        auto preprocess = params["preprocess"];
        if (preprocess["filter_size_surf"]) {
            filter_size_surf_min_ = preprocess["filter_size_surf"].as<double>();
            offline_config_.filter_size_surf = filter_size_surf_min_;
        }
    }
    
    // IMU settings
    if (params["imu"]) {
        auto imu = params["imu"];
        if (imu["imu_en"]) imu_en_ = imu["imu_en"].as<bool>();
        if (imu["imu_int_frame"]) imu_int_frame_ = imu["imu_int_frame"].as<int>();
        if (imu["gravity_est_en"]) gravity_est_en_ = imu["gravity_est_en"].as<bool>();
        if (imu["ba_bg_est_en"]) ba_bg_est_en_ = imu["ba_bg_est_en"].as<bool>();
    }
    
    // Publish settings
    if (params["publish"]) {
        auto publish = params["publish"];
        if (publish["dense_map_en"]) offline_config_.dense_map = publish["dense_map_en"].as<bool>();
    }
    
    // Extrinsics
    ext_t_ = Eigen::Vector3d::Zero();
    ext_r_ = Eigen::Matrix3d::Identity();
    
    if (params["extrin_calib"]) {
        auto extrin = params["extrin_calib"];
        if (extrin["extrinsic_T"]) {
            auto T = extrin["extrinsic_T"].as<std::vector<double>>();
            if (T.size() >= 3) {
                ext_t_ << T[0], T[1], T[2];
            }
        }
        if (extrin["extrinsic_R"]) {
            auto R = extrin["extrinsic_R"].as<std::vector<double>>();
            if (R.size() >= 9) {
                ext_r_ << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
            }
        }
    }
    
    std::cout << "[OfflineLIVMapper] Config loaded from: " << yaml_path << std::endl;
    std::cout << "  LiDAR topic: " << offline_config_.lidar_topic << std::endl;
    std::cout << "  IMU topic: " << offline_config_.imu_topic << std::endl;
    std::cout << "  IMU enabled: " << (imu_en_ ? "yes" : "no") << std::endl;
}

void OfflineLIVMapper::initComponents() {
    // Initialize point clouds
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    
    // Initialize preprocessor
    p_pre_.reset(new Preprocess());
    
    // Initialize IMU processor
    p_imu_.reset(new ImuProcess());
    p_imu_->set_extrinsic(ext_t_, ext_r_);
    p_imu_->set_imu_init_frame_num(imu_int_frame_);
    
    if (!imu_en_) p_imu_->disable_imu();
    if (!gravity_est_en_) p_imu_->disable_gravity_est();
    if (!ba_bg_est_en_) p_imu_->disable_bias_est();
    
    // Initialize voxel map manager with default config
    VoxelMapConfig voxel_config;
    voxel_config.max_iterations_ = 5;
    voxel_config.max_voxel_size_ = 0.15;
    voxel_config.max_layer_ = 2;
    voxel_config.max_points_num_ = 50;
    voxel_config.planner_threshold_ = 0.01;
    voxel_config.beam_err_ = 0.05;
    voxel_config.dept_err_ = 0.02;
    voxel_config.sigma_num_ = 3.0;
    voxel_config.is_pub_plane_map_ = false;
    voxel_config.map_sliding_en = false;
    voxel_config.sliding_thresh = 8.0;
    voxel_config.half_map_size = 100;
    voxel_config.layer_init_num_ = {5, 5, 5, 5, 5};
    voxelmap_manager_.reset(new VoxelMapManager(voxel_config, voxel_map_));
    voxelmap_manager_->extT_ = ext_t_;
    voxelmap_manager_->extR_ = ext_r_;
    
    // Initialize downsampling filter
    downsample_filter_.setLeafSize(filter_size_surf_min_, filter_size_surf_min_, filter_size_surf_min_);
    
    // Set SLAM mode
    slam_mode_ = imu_en_ ? ONLY_LIO : ONLY_LO;
    
    // Initialize lidar measures
    lidar_measures_.lidar.reset(new PointCloudXYZI());
    lidar_measures_.pcl_proc_cur.reset(new PointCloudXYZI());
    lidar_measures_.pcl_proc_next.reset(new PointCloudXYZI());
    
    // Initialize keyframe tracking
    last_kf_pos_ = Eigen::Vector3d::Zero();
    last_kf_rot_ = Eigen::Matrix3d::Identity();
    
    std::cout << "[OfflineLIVMapper] Components initialized" << std::endl;
}

void OfflineLIVMapper::addImu(const sensor_msgs::msg::Imu::SharedPtr& msg) {
    if (!imu_en_) return;
    if (last_timestamp_lidar_ < 0.0) return;  // Wait for first lidar
    
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    double timestamp = rclcpp::Time(msg->header.stamp).seconds();
    
    if (last_timestamp_imu_ > 0.0 && timestamp < last_timestamp_imu_) {
        std::cerr << "[OfflineLIVMapper] IMU loop back detected, skipping" << std::endl;
        return;
    }
    
    last_timestamp_imu_ = timestamp;
    imu_buffer_.push_back(msg);
}

void OfflineLIVMapper::addLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    double cur_time = rclcpp::Time(msg->header.stamp).seconds();
    
    if (cur_time < last_timestamp_lidar_) {
        std::cerr << "[OfflineLIVMapper] LiDAR loop back detected, clearing buffer" << std::endl;
        lidar_buffer_.clear();
        lidar_time_buffer_.clear();
    }
    
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre_->process(msg, cloud);
    
    if (!cloud || cloud->empty()) {
        return;
    }
    
    lidar_buffer_.push_back(cloud);
    lidar_time_buffer_.push_back(cur_time);
    last_timestamp_lidar_ = cur_time;
}

void OfflineLIVMapper::addPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& msg) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    double cur_time = rclcpp::Time(msg->header.stamp).seconds();
    
    if (cur_time < last_timestamp_lidar_) {
        std::cerr << "[OfflineLIVMapper] LiDAR loop back detected, clearing buffer" << std::endl;
        lidar_buffer_.clear();
        lidar_time_buffer_.clear();
    }
    
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre_->process(msg, cloud);
    
    if (!cloud || cloud->empty()) {
        return;
    }
    
    lidar_buffer_.push_back(cloud);
    lidar_time_buffer_.push_back(cur_time);
    last_timestamp_lidar_ = cur_time;
}

bool OfflineLIVMapper::processOnce() {
    if (!syncPackages()) {
        return false;
    }
    
    // Handle first frame
    if (!is_first_frame_) {
        first_lidar_time_ = lidar_measures_.last_lio_update_time;
        p_imu_->first_lidar_time = first_lidar_time_;
        is_first_frame_ = true;
        std::cout << "[OfflineLIVMapper] First LiDAR frame at t=" 
                  << std::fixed << std::setprecision(3) << first_lidar_time_ << std::endl;
    }
    
    // Process IMU
    processImuData();
    
    // State estimation
    if (lidar_measures_.lio_vio_flg == LIO || lidar_measures_.lio_vio_flg == LO) {
        handleLIO();
    }
    
    return true;
}

bool OfflineLIVMapper::syncPackages() {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    if (lidar_buffer_.empty()) return false;
    if (imu_buffer_.empty() && imu_en_) return false;
    
    switch (slam_mode_) {
    case ONLY_LIO: {
        if (lidar_measures_.last_lio_update_time < 0.0) {
            lidar_measures_.last_lio_update_time = lidar_time_buffer_.front();
        }
        
        if (!lidar_pushed_) {
            lidar_measures_.lidar = lidar_buffer_.front();
            if (lidar_measures_.lidar->points.size() <= 1) return false;
            
            lidar_measures_.lidar_frame_beg_time = lidar_time_buffer_.front();
            lidar_measures_.lidar_frame_end_time = lidar_measures_.lidar_frame_beg_time + 
                lidar_measures_.lidar->points.back().curvature / 1000.0;
            lidar_measures_.pcl_proc_cur = lidar_measures_.lidar;
            lidar_pushed_ = true;
        }
        
        if (imu_en_ && last_timestamp_imu_ < lidar_measures_.lidar_frame_end_time) {
            return false;
        }
        
        MeasureGroup m;
        m.imu.clear();
        m.lio_time = lidar_measures_.lidar_frame_end_time;
        
        while (!imu_buffer_.empty()) {
            if (rclcpp::Time(imu_buffer_.front()->header.stamp).seconds() > lidar_measures_.lidar_frame_end_time) {
                break;
            }
            m.imu.push_back(imu_buffer_.front());
            imu_buffer_.pop_front();
        }
        
        lidar_buffer_.pop_front();
        lidar_time_buffer_.pop_front();
        
        lidar_measures_.lio_vio_flg = LIO;
        lidar_measures_.measures.push_back(m);
        lidar_pushed_ = false;
        return true;
    }
    
    case ONLY_LO: {
        if (!lidar_pushed_) {
            if (lidar_buffer_.empty()) return false;
            lidar_measures_.lidar = lidar_buffer_.front();
            lidar_measures_.lidar_frame_beg_time = lidar_time_buffer_.front();
            lidar_measures_.lidar_frame_end_time = lidar_measures_.lidar_frame_beg_time +
                lidar_measures_.lidar->points.back().curvature / 1000.0;
            lidar_pushed_ = true;
        }
        
        MeasureGroup m;
        m.lio_time = lidar_measures_.lidar_frame_end_time;
        
        lidar_buffer_.pop_front();
        lidar_time_buffer_.pop_front();
        
        lidar_pushed_ = false;
        lidar_measures_.lio_vio_flg = LO;
        lidar_measures_.measures.push_back(m);
        return true;
    }
    
    default:
        return false;
    }
}

void OfflineLIVMapper::processImuData() {
    p_imu_->Process2(lidar_measures_, state_, feats_undistort_);
    
    state_propagat_ = state_;
    voxelmap_manager_->state_ = state_;
    voxelmap_manager_->feats_undistort_ = feats_undistort_;
}

void OfflineLIVMapper::handleLIO() {
    if (feats_undistort_->empty() || feats_undistort_ == nullptr) {
        std::cerr << "[OfflineLIVMapper] No points in undistorted cloud" << std::endl;
        return;
    }
    
    // Downsample
    downsample_filter_.setInputCloud(feats_undistort_);
    downsample_filter_.filter(*feats_down_body_);
    
    int feats_down_size = feats_down_body_->points.size();
    voxelmap_manager_->feats_down_body_ = feats_down_body_;
    
    // Transform to world frame
    transformLidar(state_.rot_end, state_.pos_end, feats_down_body_, feats_down_world_);
    voxelmap_manager_->feats_down_world_ = feats_down_world_;
    voxelmap_manager_->feats_down_size_ = feats_down_size;
    
    // Initialize map if needed
    if (!lidar_map_inited_) {
        lidar_map_inited_ = true;
        voxelmap_manager_->BuildVoxelMap();
    }
    
    // State estimation
    voxelmap_manager_->StateEstimation(state_propagat_);
    state_ = voxelmap_manager_->state_;
    
    // Update voxel map
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
    transformLidar(state_.rot_end, state_.pos_end, feats_down_body_, world_lidar);
    
    for (size_t i = 0; i < world_lidar->points.size(); i++) {
        voxelmap_manager_->pv_list_[i].point_w << world_lidar->points[i].x, 
                                                   world_lidar->points[i].y, 
                                                   world_lidar->points[i].z;
        Eigen::Matrix3d point_crossmat = voxelmap_manager_->cross_mat_list_[i];
        Eigen::Matrix3d var = voxelmap_manager_->body_cov_list_[i];
        var = (state_.rot_end * ext_r_) * var * (state_.rot_end * ext_r_).transpose() +
              (-point_crossmat) * state_.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + 
              state_.cov.block<3, 3>(3, 3);
        voxelmap_manager_->pv_list_[i].var = var;
    }
    voxelmap_manager_->UpdateVoxelMap(voxelmap_manager_->pv_list_);
    
    // Map sliding if enabled
    if (voxelmap_manager_->config_setting_.map_sliding_en) {
        voxelmap_manager_->mapSliding();
    }
    
    // Store trajectory
    storePose();
    
    // Check and extract keyframe
    if (isKeyFrame()) {
        extractKeyFrame();
    }
    
    frame_count_++;
    
    // Progress output
    if (frame_count_ % 100 == 0) {
        std::cout << "[OfflineLIVMapper] Frame " << frame_count_ 
                  << " | Keyframes: " << keyframes_.size()
                  << " | Pos: " << state_.pos_end.transpose() << std::endl;
    }
}

void OfflineLIVMapper::transformLidar(const Eigen::Matrix3d& rot, const Eigen::Vector3d& t,
                                       const PointCloudXYZI::Ptr& input, PointCloudXYZI::Ptr& output) {
    output->clear();
    output->reserve(input->size());
    
    for (size_t i = 0; i < input->size(); i++) {
        const auto& p_in = input->points[i];
        Eigen::Vector3d p(p_in.x, p_in.y, p_in.z);
        p = rot * (ext_r_ * p + ext_t_) + t;
        
        PointType p_out;
        p_out.x = p(0);
        p_out.y = p(1);
        p_out.z = p(2);
        p_out.intensity = p_in.intensity;
        output->points.push_back(p_out);
    }
}

bool OfflineLIVMapper::isKeyFrame() const {
    if (keyframes_.empty()) return true;
    
    double delta_trans = (state_.pos_end - last_kf_pos_).norm();
    
    Eigen::Quaterniond q_cur(state_.rot_end);
    Eigen::Quaterniond q_last(last_kf_rot_);
    double delta_deg = q_cur.angularDistance(q_last) * 180.0 / M_PI;
    
    return (delta_trans > offline_config_.keyframe_delta_trans) || 
           (delta_deg > offline_config_.keyframe_delta_deg);
}

void OfflineLIVMapper::extractKeyFrame() {
    KeyFrame kf;
    kf.id = keyframes_.size();
    kf.timestamp = lidar_measures_.measures.back().lio_time;
    kf.rotation = state_.rot_end;
    kf.position = state_.pos_end;
    
    // Copy body frame point cloud
    kf.body_cloud.reset(new PointCloudXYZI(*feats_down_body_));
    
    keyframes_.push_back(kf);
    last_kf_pos_ = state_.pos_end;
    last_kf_rot_ = state_.rot_end;
    
    std::cout << "[OfflineLIVMapper] KeyFrame #" << kf.id 
              << " t=" << std::fixed << std::setprecision(3) << kf.timestamp
              << " pos=" << kf.position.transpose() << std::endl;
}

void OfflineLIVMapper::storePose() {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.linear() = state_.rot_end;
    pose.translation() = state_.pos_end;
    
    double timestamp = lidar_measures_.measures.back().lio_time;
    trajectory_.emplace_back(timestamp, pose);
}

void OfflineLIVMapper::setKeyframeThresholds(double trans_m, double rot_deg) {
    offline_config_.keyframe_delta_trans = trans_m;
    offline_config_.keyframe_delta_deg = rot_deg;
}

void OfflineLIVMapper::savePosesTUM(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OfflineLIVMapper] Failed to open: " << path << std::endl;
        return;
    }
    
    file << "# timestamp tx ty tz qx qy qz qw" << std::endl;
    file << std::fixed << std::setprecision(9);
    
    for (const auto& tp : trajectory_) {
        Eigen::Quaterniond q(tp.pose.rotation());
        Eigen::Vector3d t = tp.pose.translation();
        
        file << tp.timestamp << " "
             << t.x() << " " << t.y() << " " << t.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }
    
    file.close();
    std::cout << "[OfflineLIVMapper] Saved " << trajectory_.size() 
              << " poses to: " << path << std::endl;
}

void OfflineLIVMapper::saveKeyframePoses(const std::string& path) const {
    std::ofstream file(path);
    if (!file.is_open()) {
        std::cerr << "[OfflineLIVMapper] Failed to open: " << path << std::endl;
        return;
    }
    
    file << "# id timestamp tx ty tz qx qy qz qw" << std::endl;
    file << std::fixed << std::setprecision(9);
    
    for (const auto& kf : keyframes_) {
        Eigen::Quaterniond q(kf.rotation);
        
        file << kf.id << " " << kf.timestamp << " "
             << kf.position.x() << " " << kf.position.y() << " " << kf.position.z() << " "
             << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
             << std::endl;
    }
    
    file.close();
    std::cout << "[OfflineLIVMapper] Saved " << keyframes_.size() 
              << " keyframe poses to: " << path << std::endl;
}

void OfflineLIVMapper::saveKeyframeClouds(const std::string& dir) const {
    namespace fs = std::filesystem;
    
    if (!fs::exists(dir)) {
        fs::create_directories(dir);
    }
    
    for (const auto& kf : keyframes_) {
        std::string filename = dir + "/" + std::to_string(kf.id) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *kf.body_cloud);
    }
    
    std::cout << "[OfflineLIVMapper] Saved " << keyframes_.size() 
              << " keyframe clouds to: " << dir << std::endl;
}

void OfflineLIVMapper::saveGlobalMap(const std::string& path, double resolution) const {
    PointCloudXYZI::Ptr global_map(new PointCloudXYZI());
    
    for (const auto& kf : keyframes_) {
        PointCloudXYZI::Ptr world_cloud(new PointCloudXYZI());
        
        // Transform body cloud to world frame
        for (const auto& p_body : kf.body_cloud->points) {
            Eigen::Vector3d p(p_body.x, p_body.y, p_body.z);
            p = kf.rotation * (ext_r_ * p + ext_t_) + kf.position;
            
            PointType p_world;
            p_world.x = p.x();
            p_world.y = p.y();
            p_world.z = p.z();
            p_world.intensity = p_body.intensity;
            world_cloud->points.push_back(p_world);
        }
        
        *global_map += *world_cloud;
    }
    
    // Downsample if resolution > 0
    if (resolution > 0 && !global_map->empty()) {
        PointCloudXYZI::Ptr filtered(new PointCloudXYZI());
        pcl::VoxelGrid<PointType> filter;
        filter.setInputCloud(global_map);
        filter.setLeafSize(resolution, resolution, resolution);
        filter.filter(*filtered);
        global_map = filtered;
    }
    
    if (!global_map->empty()) {
        pcl::io::savePCDFileBinary(path, *global_map);
        std::cout << "[OfflineLIVMapper] Saved global map with " << global_map->size() 
                  << " points to: " << path << std::endl;
    } else {
        std::cerr << "[OfflineLIVMapper] Global map is empty, not saved" << std::endl;
    }
}

} // namespace livo2_offline
