/*
 * Offline LIV Mapper for FAST-LIVO2
 * Implementation - reuses core LIVO2 components
 */

#include "offline/offline_liv_mapper.h"

#include <filesystem>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/memory.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling.hpp>

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
    voxel_map_.clear();
    if (voxelmap_manager_) {
        voxelmap_manager_->voxel_map_.clear();
    }
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
        // CRITICAL: Preprocess parameters for point cloud preprocessing
        if (preprocess["blind"]) preprocess_blind_ = preprocess["blind"].as<double>();
        if (preprocess["lidar_type"]) preprocess_lidar_type_ = preprocess["lidar_type"].as<int>();
        if (preprocess["scan_line"]) preprocess_scan_line_ = preprocess["scan_line"].as<int>();
        if (preprocess["point_filter_num"]) preprocess_point_filter_num_ = preprocess["point_filter_num"].as<int>();
    }
    
    // LIO / VoxelMap settings - read from yaml instead of hardcoding
    if (params["lio"]) {
        auto lio = params["lio"];
        if (lio["max_iterations"]) voxel_max_iterations_ = lio["max_iterations"].as<int>();
        if (lio["voxel_size"]) voxel_size_ = lio["voxel_size"].as<double>();
        if (lio["max_layer"]) voxel_max_layer_ = lio["max_layer"].as<int>();
        if (lio["max_points_num"]) voxel_max_points_num_ = lio["max_points_num"].as<int>();
        if (lio["min_eigen_value"]) voxel_planner_threshold_ = lio["min_eigen_value"].as<double>();
        if (lio["beam_err"]) voxel_beam_err_ = lio["beam_err"].as<double>();
        if (lio["dept_err"]) voxel_dept_err_ = lio["dept_err"].as<double>();
        if (lio["layer_init_num"]) {
            voxel_layer_init_num_ = lio["layer_init_num"].as<std::vector<int>>();
        }
    }
    
    // Local map settings
    if (params["local_map"]) {
        auto local_map = params["local_map"];
        if (local_map["map_sliding_en"]) map_sliding_en_ = local_map["map_sliding_en"].as<bool>();
        if (local_map["half_map_size"]) half_map_size_ = local_map["half_map_size"].as<int>();
        if (local_map["sliding_thresh"]) sliding_thresh_ = local_map["sliding_thresh"].as<double>();
    }
    
    // IMU settings
    if (params["imu"]) {
        auto imu = params["imu"];
        if (imu["imu_en"]) imu_en_ = imu["imu_en"].as<bool>();
        if (imu["imu_int_frame"]) imu_int_frame_ = imu["imu_int_frame"].as<int>();
        if (imu["gravity_est_en"]) gravity_est_en_ = imu["gravity_est_en"].as<bool>();
        if (imu["ba_bg_est_en"]) ba_bg_est_en_ = imu["ba_bg_est_en"].as<bool>();
        // IMU noise covariances - CRITICAL for IMU integration accuracy
        if (imu["acc_cov"]) acc_cov_ = imu["acc_cov"].as<double>();
        if (imu["gyr_cov"]) gyr_cov_ = imu["gyr_cov"].as<double>();
        if (imu["b_acc_cov"]) b_acc_cov_ = imu["b_acc_cov"].as<double>();
        if (imu["b_gyr_cov"]) b_gyr_cov_ = imu["b_gyr_cov"].as<double>();
    }
    
    // Publish settings
    if (params["publish"]) {
        auto publish = params["publish"];
        if (publish["dense_map_en"]) offline_config_.dense_map = publish["dense_map_en"].as<bool>();
    }
    
    // Offline settings
    if (params["offline"]) {
        auto offline = params["offline"];
        if (offline["keyframe_delta_trans"])
            offline_config_.keyframe_delta_trans = offline["keyframe_delta_trans"].as<double>();
        if (offline["keyframe_delta_deg"])
            offline_config_.keyframe_delta_deg = offline["keyframe_delta_deg"].as<double>();
        if (offline["output_dir"])
            offline_config_.output_dir = offline["output_dir"].as<std::string>();
        if (offline["save_keyframes"])
            offline_config_.save_keyframes = offline["save_keyframes"].as<bool>();
        if (offline["save_global_map"])
            offline_config_.save_global_map = offline["save_global_map"].as<bool>();
        if (offline["global_map_resolution"])
            offline_config_.global_map_resolution = offline["global_map_resolution"].as<double>();
    }

#ifdef USE_BACKEND
    // Backend PGO and loop closure settings
    if (params["backend"]) {
        auto backend = params["backend"];
        if (backend["enabled"]) backend_enabled_ = backend["enabled"].as<bool>();
        if (backend["save_g2o"]) backend_save_g2o_ = backend["save_g2o"].as<bool>();
        if (backend["g2o_filename"]) backend_g2o_filename_ = backend["g2o_filename"].as<std::string>();
        if (backend["save_loop_constraints"]) backend_save_loop_constraints_ = backend["save_loop_constraints"].as<bool>();
        if (backend["pgo"]) {
            auto pgo = backend["pgo"];
            if (pgo["use_isam2"]) pgo_config_.use_isam2 = pgo["use_isam2"].as<bool>();
            if (pgo["relinearize_threshold"]) pgo_config_.relinearize_threshold = pgo["relinearize_threshold"].as<double>();
            if (pgo["relinearize_skip"]) pgo_config_.relinearize_skip = pgo["relinearize_skip"].as<int>();
            if (pgo["prior_noise"]) pgo_config_.prior_noise = pgo["prior_noise"].as<double>();
            if (pgo["odom_rot_noise"]) pgo_config_.odom_rot_noise = pgo["odom_rot_noise"].as<double>();
            if (pgo["odom_trans_noise"]) pgo_config_.odom_trans_noise = pgo["odom_trans_noise"].as<double>();
        }
        if (backend["loop_closure"]) {
            auto lc = backend["loop_closure"];
            if (lc["search_radius"]) loop_config_.search_radius = lc["search_radius"].as<double>();
            if (lc["time_threshold"]) loop_config_.time_threshold = lc["time_threshold"].as<double>();
            if (lc["fitness_threshold"]) loop_config_.fitness_threshold = lc["fitness_threshold"].as<double>();
            if (lc["submap_half_range"]) loop_config_.submap_half_range = lc["submap_half_range"].as<int>();
            if (lc["submap_resolution"]) loop_config_.submap_resolution = lc["submap_resolution"].as<double>();
            if (lc["min_detect_interval"]) loop_config_.min_detect_interval = lc["min_detect_interval"].as<double>();
            // small_gicp GICP parameters (replaces PCL ICP)
            if (lc["gicp_max_iterations"]) loop_config_.gicp_max_iterations = lc["gicp_max_iterations"].as<int>();
            if (lc["gicp_max_correspondence_dist"]) loop_config_.gicp_max_correspondence_dist = lc["gicp_max_correspondence_dist"].as<double>();
            if (lc["gicp_num_threads"]) loop_config_.gicp_num_threads = lc["gicp_num_threads"].as<int>();
            if (lc["gicp_correspondence_randomness"]) loop_config_.gicp_correspondence_randomness = lc["gicp_correspondence_randomness"].as<int>();
        }
    }
#endif

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
    std::cout << "  IMU covariances - acc: " << acc_cov_ << ", gyr: " << gyr_cov_ 
              << ", b_acc: " << b_acc_cov_ << ", b_gyr: " << b_gyr_cov_ << std::endl;
    std::cout << "  Preprocess - blind: " << preprocess_blind_ << ", lidar_type: " << preprocess_lidar_type_
              << ", scan_line: " << preprocess_scan_line_ << ", point_filter: " << preprocess_point_filter_num_ << std::endl;
    std::cout << "  VoxelMap - voxel_size: " << voxel_size_ << ", max_layer: " << voxel_max_layer_ 
              << ", max_iter: " << voxel_max_iterations_ << std::endl;
    std::cout << "  LocalMap - sliding_en: " << (map_sliding_en_ ? "yes" : "no") 
              << ", half_size: " << half_map_size_ << std::endl;
    std::cout << "  Keyframe delta trans: " << offline_config_.keyframe_delta_trans << " m" << std::endl;
    std::cout << "  Keyframe delta deg: " << offline_config_.keyframe_delta_deg << " deg" << std::endl;
    std::cout << "  Output dir: " << offline_config_.output_dir << std::endl;
    std::cout << "  Save keyframes: " << (offline_config_.save_keyframes ? "yes" : "no") << std::endl;
    std::cout << "  Save global map: " << (offline_config_.save_global_map ? "yes" : "no") << std::endl;
}

void OfflineLIVMapper::initComponents() {
    // Initialize point clouds
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
    
    // Initialize preprocessor with parameters from yaml (same as online mode)
    p_pre_.reset(new Preprocess());
    p_pre_->blind = preprocess_blind_;
    p_pre_->blind_sqr = preprocess_blind_ * preprocess_blind_;
    p_pre_->lidar_type = preprocess_lidar_type_;
    p_pre_->N_SCANS = preprocess_scan_line_;
    p_pre_->point_filter_num = preprocess_point_filter_num_;
    
    // Initialize IMU processor with parameters from yaml (CRITICAL - same as online mode)
    p_imu_.reset(new ImuProcess());
    p_imu_->set_extrinsic(ext_t_, ext_r_);
    p_imu_->set_imu_init_frame_num(imu_int_frame_);
    // CRITICAL: Set IMU noise covariances - this was missing before!
    p_imu_->set_gyr_cov_scale(Eigen::Vector3d(gyr_cov_, gyr_cov_, gyr_cov_));
    p_imu_->set_acc_cov_scale(Eigen::Vector3d(acc_cov_, acc_cov_, acc_cov_));
    p_imu_->set_gyr_bias_cov(Eigen::Vector3d(b_gyr_cov_, b_gyr_cov_, b_gyr_cov_));
    p_imu_->set_acc_bias_cov(Eigen::Vector3d(b_acc_cov_, b_acc_cov_, b_acc_cov_));
    
    if (!imu_en_) p_imu_->disable_imu();
    if (!gravity_est_en_) p_imu_->disable_gravity_est();
    if (!ba_bg_est_en_) p_imu_->disable_bias_est();
    
    // Initialize voxel map manager with parameters from yaml (same as online mode)
    VoxelMapConfig voxel_config;
    voxel_config.max_iterations_ = voxel_max_iterations_;
    voxel_config.max_voxel_size_ = voxel_size_;
    voxel_config.max_layer_ = voxel_max_layer_;
    voxel_config.max_points_num_ = voxel_max_points_num_;
    voxel_config.planner_threshold_ = voxel_planner_threshold_;
    voxel_config.beam_err_ = voxel_beam_err_;
    voxel_config.dept_err_ = voxel_dept_err_;
    voxel_config.sigma_num_ = 3.0;
    voxel_config.is_pub_plane_map_ = false;
    voxel_config.map_sliding_en = map_sliding_en_;
    voxel_config.sliding_thresh = sliding_thresh_;
    voxel_config.half_map_size = half_map_size_;
    voxel_config.layer_init_num_ = voxel_layer_init_num_;
    voxelmap_manager_.reset(new VoxelMapManager(voxel_config, voxel_map_));
    voxelmap_manager_->extT_ = ext_t_;
    voxelmap_manager_->extR_ = ext_r_;
    
    // Downsampling uses small_gicp::voxelgrid_sampling in handleLIO() (no PCL VoxelGrid member)
    
    // Set SLAM mode
    slam_mode_ = imu_en_ ? ONLY_LIO : ONLY_LO;
    
    // Initialize lidar measures
    lidar_measures_.lidar.reset(new PointCloudXYZI());
    lidar_measures_.pcl_proc_cur.reset(new PointCloudXYZI());
    lidar_measures_.pcl_proc_next.reset(new PointCloudXYZI());
    
    // Initialize keyframe tracking
    last_kf_pos_ = Eigen::Vector3d::Zero();
    last_kf_rot_ = Eigen::Matrix3d::Identity();

#ifdef USE_BACKEND
    if (backend_enabled_) {
        pose_graph_ = std::make_shared<PoseGraph>(pgo_config_);
        loop_detector_ = std::make_shared<LoopDetector>(loop_config_);
        std::cout << "[OfflineLIVMapper] Backend PGO and loop closure enabled" << std::endl;
    }
#endif

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
    
    // Downsample with small_gicp (avoids PCL VoxelGrid destructor free() bug)
    {
        auto xyz_in = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        xyz_in->reserve(feats_undistort_->size());
        for (const auto& pt : feats_undistort_->points) {
            pcl::PointXYZ p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            xyz_in->push_back(p);
        }
        auto xyz_down = small_gicp::voxelgrid_sampling(*xyz_in, filter_size_surf_min_);
        feats_down_body_ = pcl::make_shared<PointCloudXYZI>();
        feats_down_body_->reserve(xyz_down->size());
        for (const auto& pt : xyz_down->points) {
            PointType p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = 0;
            feats_down_body_->points.push_back(p);
        }
        feats_down_body_->width = feats_down_body_->points.size();
        feats_down_body_->height = 1;
        feats_down_body_->is_dense = true;
    }
    
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
    
    const size_t n_update = std::min(static_cast<size_t>(world_lidar->points.size()),
                                    voxelmap_manager_->pv_list_.size());
    if (n_update != world_lidar->points.size()) {
        std::cerr << "[OfflineLIVMapper] pv_list size mismatch: world_lidar=" << world_lidar->points.size()
                  << " pv_list=" << voxelmap_manager_->pv_list_.size() << std::endl;
    }
    for (size_t i = 0; i < n_update; i++) {
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
    kf.r_local = state_.rot_end;
    kf.t_local = state_.pos_end;
#ifdef USE_BACKEND
    if (backend_enabled_ && pose_graph_) {
        kf.r_global = pose_graph_->offsetR() * state_.rot_end;
        kf.t_global = pose_graph_->offsetR() * state_.pos_end + pose_graph_->offsetT();
    } else
#endif
    {
        kf.r_global = state_.rot_end;
        kf.t_global = state_.pos_end;
    }
    kf.pose_covariance = state_.cov.block<6, 6>(0, 0);

    // Copy body frame point cloud
    kf.body_cloud.reset(new PointCloudXYZI(*feats_down_body_));

#ifdef USE_BACKEND
    if (backend_enabled_ && pose_graph_) {
        const KeyFrame* prev_kf = keyframes_.empty() ? nullptr : &keyframes_.back();
        pose_graph_->addKeyframe(kf, prev_kf);
    }
#endif

    // Immediately save keyframe cloud if incremental saving is enabled
    if (incremental_save_enabled_) {
        std::string filename = incremental_output_dir_ + "/keyframes/" + std::to_string(kf.id) + ".pcd";
        pcl::io::savePCDFileBinary(filename, *kf.body_cloud);
    }

    keyframes_.push_back(kf);
    last_kf_pos_ = state_.pos_end;
    last_kf_rot_ = state_.rot_end;

#ifdef USE_BACKEND
    if (backend_enabled_ && pose_graph_ && loop_detector_) {
        auto loop = loop_detector_->detect(keyframes_, kf.id);
        if (loop) {
            pose_graph_->addLoopConstraint(*loop);
            std::cout << "[OfflineLIVMapper] Loop closure: " << loop->target_id << " <-> " << loop->source_id
                      << " score=" << loop->fitness_score << std::endl;
        }
        pose_graph_->optimize();
        updateKeyframePoses();
    }
#endif

    std::cout << "[OfflineLIVMapper] KeyFrame #" << kf.id
              << " t=" << std::fixed << std::setprecision(3) << kf.timestamp
              << " pos=" << kf.t_global.transpose() << std::endl;
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

void OfflineLIVMapper::setOutputDirectory(const std::string& dir) {
    incremental_output_dir_ = dir;
    incremental_save_enabled_ = !dir.empty();
    
    if (incremental_save_enabled_) {
        // Ensure keyframes directory exists
        namespace fs = std::filesystem;
        fs::create_directories(dir + "/keyframes");
        std::cout << "[OfflineLIVMapper] Incremental saving enabled to: " << dir << std::endl;
    }
}

void OfflineLIVMapper::shutdown() {
    keyframes_.clear();
    trajectory_.clear();
    if (voxelmap_manager_) {
        voxelmap_manager_->voxel_map_.clear();
        voxelmap_manager_.reset();
    }
    if (feats_undistort_) feats_undistort_->clear();
    if (feats_down_body_) feats_down_body_->clear();
    if (feats_down_world_) feats_down_world_->clear();
    p_imu_.reset();
    p_pre_.reset();
    std::cout << "[OfflineLIVMapper] Shutdown complete" << std::endl;
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
        Eigen::Quaterniond q(kf.r_global);
        file << kf.id << " " << kf.timestamp << " "
             << kf.t_global.x() << " " << kf.t_global.y() << " " << kf.t_global.z() << " "
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
    // Use pcl::make_shared for Eigen-aligned allocator (avoids SIGSEGV in free on exit)
    PointCloudXYZI::Ptr global_map = pcl::make_shared<PointCloudXYZI>();
    
    for (const auto& kf : keyframes_) {
        PointCloudXYZI::Ptr world_cloud = pcl::make_shared<PointCloudXYZI>();
        
        // Transform body cloud to world frame
        for (const auto& p_body : kf.body_cloud->points) {
            Eigen::Vector3d p(p_body.x, p_body.y, p_body.z);
            p = kf.r_global * (ext_r_ * p + ext_t_) + kf.t_global;
            
            PointType p_world;
            p_world.x = static_cast<float>(p.x());
            p_world.y = static_cast<float>(p.y());
            p_world.z = static_cast<float>(p.z());
            p_world.intensity = p_body.intensity;
            world_cloud->points.push_back(p_world);
        }
        
        *global_map += *world_cloud;
    }
    
    // Downsample using small_gicp instead of PCL VoxelGrid (avoids PCL free() bug on destruct)
    if (resolution > 0 && !global_map->empty()) {
        auto xyz_cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        xyz_cloud->reserve(global_map->size());
        for (const auto& pt : global_map->points) {
            pcl::PointXYZ p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            xyz_cloud->push_back(p);
        }
        auto downsampled_xyz = small_gicp::voxelgrid_sampling(*xyz_cloud, resolution);
        global_map = pcl::make_shared<PointCloudXYZI>();
        global_map->reserve(downsampled_xyz->size());
        for (const auto& pt : downsampled_xyz->points) {
            PointType p_out;
            p_out.x = pt.x;
            p_out.y = pt.y;
            p_out.z = pt.z;
            p_out.intensity = 0;
            global_map->points.push_back(p_out);
        }
        global_map->width = global_map->points.size();
        global_map->height = 1;
        global_map->is_dense = true;
    }
    
    if (!global_map->empty()) {
        pcl::io::savePCDFileBinary(path, *global_map);
        std::cout << "[OfflineLIVMapper] Saved global map with " << global_map->size() 
                  << " points to: " << path << std::endl;
    } else {
        std::cerr << "[OfflineLIVMapper] Global map is empty, not saved" << std::endl;
    }
}

#ifdef USE_BACKEND
void OfflineLIVMapper::updateKeyframePoses() {
    if (!pose_graph_ || keyframes_.empty()) return;
    std::vector<Eigen::Isometry3d> poses = pose_graph_->getOptimizedPoses();
    if (poses.size() != keyframes_.size()) return;
    for (size_t i = 0; i < keyframes_.size(); ++i) {
        keyframes_[i].r_global = poses[i].linear();
        keyframes_[i].t_global = poses[i].translation();
    }
}

void OfflineLIVMapper::saveBackendOutput() const {
    if (!backend_enabled_ || !pose_graph_) return;
    namespace fs = std::filesystem;
    std::string out_dir = incremental_save_enabled_ ? incremental_output_dir_ : offline_config_.output_dir;
    if (out_dir.empty()) out_dir = "./output";
    fs::create_directories(out_dir);
    if (backend_save_g2o_) {
        std::string g2o_path = out_dir + "/" + backend_g2o_filename_;
        pose_graph_->saveG2o(g2o_path);
    }
    if (backend_save_loop_constraints_ && loop_detector_) {
        const auto& pairs = loop_detector_->historyPairs();
        if (!pairs.empty()) {
            std::string loop_path = out_dir + "/loop_constraints.txt";
            std::ofstream f(loop_path);
            if (f.is_open()) {
                f << "# target_id source_id\n";
                for (const auto& p : pairs) f << p.first << " " << p.second << "\n";
                f.close();
                std::cout << "[OfflineLIVMapper] Saved " << pairs.size() << " loop constraints to: " << loop_path << std::endl;
            }
        }
    }
}
#endif

} // namespace livo2_offline
