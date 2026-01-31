/*
 * Offline LIV Mapper for FAST-LIVO2
 * Processes LiDAR and IMU data from bag files without ROS2 pub/sub
 */

#ifndef OFFLINE_LIV_MAPPER_H
#define OFFLINE_LIV_MAPPER_H

#include <string>
#include <vector>
#include <deque>
#include <memory>
#include <mutex>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

// Include LIVO2 core components
#include "common_lib.h"
#include "IMU_Processing.h"
#include "preprocess.h"
#include "voxel_map.h"

#include "offline/keyframe_types.h"

#ifdef USE_BACKEND
#include "backend/pose_graph.h"
#include "backend/loop_detector.h"
#endif

namespace livo2_offline {

/**
 * @brief Configuration for offline mapping
 */
struct OfflineConfig {
    // LiDAR topics
    std::string lidar_topic = "/livox/lidar";
    std::string imu_topic = "/livox/imu";
    
    // Keyframe thresholds (aligned with lightning-lm: kf_dis_th_=2.0, kf_angle_th_=15°)
    double keyframe_delta_trans = 2.0;  // meters
    double keyframe_delta_deg = 15.0;   // degrees
    
    // Processing settings
    bool dense_map = false;
    double filter_size_surf = 0.1;
    
    // Output settings
    std::string output_dir = "./output";
    bool save_keyframes = true;
    bool save_global_map = true;
    double global_map_resolution = 0.1;  // voxel resolution for global map
};

/**
 * @brief Trajectory pose with timestamp
 */
struct TimedPose {
    double timestamp;
    Eigen::Isometry3d pose;
    
    TimedPose() : timestamp(0.0), pose(Eigen::Isometry3d::Identity()) {}
    TimedPose(double t, const Eigen::Isometry3d& p) : timestamp(t), pose(p) {}
};

/**
 * @brief Offline LIV Mapper
 * 
 * Reuses LIVO2 core components (ImuProcess, VoxelMapManager, Preprocess)
 * but operates without ROS2 pub/sub for offline bag processing.
 */
class OfflineLIVMapper {
public:
    /**
     * @brief Construct offline mapper with config file
     * @param config_path Path to YAML config file (same format as online)
     */
    explicit OfflineLIVMapper(const std::string& config_path);
    
    /**
     * @brief Construct offline mapper with explicit config
     * @param config Configuration struct
     * @param yaml_config_path Path to YAML for LIVO2 parameters
     */
    OfflineLIVMapper(const OfflineConfig& config, const std::string& yaml_config_path);
    
    ~OfflineLIVMapper();

    // === Data injection interface (replaces ROS subscribers) ===
    
    /**
     * @brief Add IMU measurement
     * @param msg IMU message
     */
    void addImu(const sensor_msgs::msg::Imu::SharedPtr& msg);
    
    /**
     * @brief Add Livox point cloud
     * @param msg Livox CustomMsg
     */
    void addLivox(const livox_ros_driver2::msg::CustomMsg::SharedPtr& msg);
    
    /**
     * @brief Add standard PointCloud2
     * @param msg PointCloud2 message
     */
    void addPointCloud2(const sensor_msgs::msg::PointCloud2::SharedPtr& msg);
    
    /**
     * @brief Process one frame if data is available
     * @return true if a frame was processed
     */
    bool processOnce();
    
    // === Results access ===
    
    /**
     * @brief Get all keyframes
     * @return Vector of keyframes
     */
    const KeyFrameVector& keyframes() const { return keyframes_; }
    
    /**
     * @brief Get full trajectory
     * @return Vector of timed poses
     */
    const std::vector<TimedPose>& trajectory() const { return trajectory_; }
    
    /**
     * @brief Get current state
     * @return Current state group
     */
    const StatesGroup& state() const { return state_; }
    
    /**
     * @brief Get frame count
     * @return Number of processed frames
     */
    size_t frameCount() const { return frame_count_; }
    
    // === Output functions ===
    
    /**
     * @brief Save trajectory to TUM format file
     * @param path Output file path
     */
    void savePosesTUM(const std::string& path) const;
    
    /**
     * @brief Save keyframe poses (before PGO when backend enabled, else current)
     * @param path Output file path (e.g. keyframes.txt)
     */
    void saveKeyframePoses(const std::string& path) const;

#ifdef USE_BACKEND
    /**
     * @brief Number of loop closures detected (0 => PGO before/after poses will be almost identical)
     */
    size_t loopClosureCount() const;

    /**
     * @brief Save optimized keyframe poses (after PGO)
     * @param path Output file path (e.g. keyframe_opt.txt)
     */
    void saveKeyframePosesOpt(const std::string& path) const;
#endif
    
    /**
     * @brief Save all keyframe point clouds
     * @param dir Output directory
     */
    void saveKeyframeClouds(const std::string& dir) const;
    
    /**
     * @brief Save global map (merged from all keyframes)
     * @param path Output PCD file path
     * @param resolution Voxel resolution for downsampling (0 = no downsampling)
     */
    void saveGlobalMap(const std::string& path, double resolution = 0.1) const;

#ifdef USE_BACKEND
    /**
     * @brief Save backend output (g2o, loop constraints) to output dir
     */
    void saveBackendOutput() const;
#endif

    // === Configuration ===
    
    /**
     * @brief Get offline config
     */
    const OfflineConfig& config() const { return offline_config_; }
    
    /**
     * @brief Set keyframe thresholds
     */
    void setKeyframeThresholds(double trans_m, double rot_deg);
    
    /**
     * @brief Set output directory for incremental saving
     * @param dir Output directory path
     * 
     * When set, keyframe point clouds will be saved immediately
     * as they are extracted, rather than at program end.
     */
    void setOutputDirectory(const std::string& dir);
    
    /**
     * @brief Explicitly release resources before destruction
     * 
     * Call this before the object goes out of scope to ensure
     * proper cleanup order of internal resources.
     */
    void shutdown();

private:
    // Configuration
    OfflineConfig offline_config_;
    
    // LIVO2 core components (reused)
    PreprocessPtr p_pre_;
    ImuProcessPtr p_imu_;
    VoxelMapManagerPtr voxelmap_manager_;
    std::unordered_map<VOXEL_LOCATION, std::shared_ptr<VoxelOctoTree>> voxel_map_;
    
    // State
    StatesGroup state_;
    StatesGroup state_propagat_;
    SLAM_MODE slam_mode_;
    
    // Point clouds
    PointCloudXYZI::Ptr feats_undistort_;
    PointCloudXYZI::Ptr feats_down_body_;
    PointCloudXYZI::Ptr feats_down_world_;
    
    // Data buffers
    std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer_;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer_;
    std::deque<double> lidar_time_buffer_;
    LidarMeasureGroup lidar_measures_;
    std::mutex buffer_mutex_;
    
    // Processing state
    bool lidar_pushed_ = false;
    bool is_first_frame_ = false;
    bool lidar_map_inited_ = false;
    double first_lidar_time_ = 0.0;
    double last_timestamp_lidar_ = -1.0;
    double last_timestamp_imu_ = -1.0;
    size_t frame_count_ = 0;
    
    // Extrinsics
    Eigen::Vector3d ext_t_;
    Eigen::Matrix3d ext_r_;
    
    // Downsampling: use small_gicp at call site (no PCL VoxelGrid member to avoid free() bug on destruct)
    double filter_size_surf_min_ = 0.1;
    
    // IMU settings
    bool imu_en_ = true;
    bool gravity_est_en_ = true;
    bool ba_bg_est_en_ = true;
    int imu_int_frame_ = 3;
    // IMU noise covariances (CRITICAL for matching online mode)
    double acc_cov_ = 0.1;
    double gyr_cov_ = 0.1;
    double b_acc_cov_ = 0.0001;
    double b_gyr_cov_ = 0.0001;
    
    // Preprocess settings (CRITICAL for matching online mode)
    double preprocess_blind_ = 0.5;
    int preprocess_lidar_type_ = 1;  // Default: Livox
    int preprocess_scan_line_ = 4;
    int preprocess_point_filter_num_ = 1;
    
    // VoxelMap settings (CRITICAL for matching online mode)
    int voxel_max_iterations_ = 5;
    double voxel_size_ = 0.15;
    int voxel_max_layer_ = 2;
    int voxel_max_points_num_ = 50;
    double voxel_planner_threshold_ = 0.01;
    double voxel_beam_err_ = 0.05;
    double voxel_dept_err_ = 0.02;
    std::vector<int> voxel_layer_init_num_ = {5, 5, 5, 5, 5};
    
    // Local map settings
    bool map_sliding_en_ = false;
    int half_map_size_ = 100;
    double sliding_thresh_ = 8.0;
    
    // Keyframe management (aligned allocator for Eigen members in KeyFrame)
    KeyFrameVector keyframes_;
    Eigen::Vector3d last_kf_pos_;
    Eigen::Matrix3d last_kf_rot_;
    
    // Trajectory storage
    std::vector<TimedPose> trajectory_;
    
    // Incremental saving
    std::string incremental_output_dir_;
    bool incremental_save_enabled_ = false;

#ifdef USE_BACKEND
    // Backend PGO and loop closure
    bool backend_enabled_ = false;
    PoseGraph::Options pgo_options_;
    LoopDetector::Options loop_options_;
    std::shared_ptr<PoseGraph> pose_graph_;
    std::shared_ptr<LoopDetector> loop_detector_;
    bool backend_save_g2o_ = true;
    std::string backend_g2o_filename_ = "pose_graph.g2o";
    bool backend_save_loop_constraints_ = true;
#endif

    // === Internal methods ===
    
    /**
     * @brief Initialize from YAML config
     */
    void initFromYaml(const std::string& yaml_path);
    
    /**
     * @brief Initialize components
     */
    void initComponents();
    
    /**
     * @brief Sync packages (adapted from LIVMapper::sync_packages)
     */
    bool syncPackages();
    
    /**
     * @brief Process IMU data (adapted from LIVMapper::processImu)
     */
    void processImuData();
    
    /**
     * @brief Handle LIO estimation (adapted from LIVMapper::handleLIO)
     */
    void handleLIO();
    
    /**
     * @brief Transform point cloud
     */
    void transformLidar(const Eigen::Matrix3d& rot, const Eigen::Vector3d& t,
                        const PointCloudXYZI::Ptr& input, PointCloudXYZI::Ptr& output);
    
    /**
     * @brief Check if current frame should be a keyframe
     */
    bool isKeyFrame() const;
    
    /**
     * @brief Extract and store keyframe
     */
    void extractKeyFrame();
    
    /**
     * @brief Store current pose to trajectory
     */
    void storePose();

#ifdef USE_BACKEND
    /**
     * @brief Update keyframe global poses from pose graph optimization
     */
    void updateKeyframePoses();
#endif
};

} // namespace livo2_offline

#endif // OFFLINE_LIV_MAPPER_H
