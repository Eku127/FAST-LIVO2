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
#include <pcl/filters/voxel_grid.h>

#include "livox_ros_driver2/msg/custom_msg.hpp"

// Include LIVO2 core components
#include "common_lib.h"
#include "IMU_Processing.h"
#include "preprocess.h"
#include "voxel_map.h"

namespace livo2_offline {

/**
 * @brief Configuration for offline mapping
 */
struct OfflineConfig {
    // LiDAR topics
    std::string lidar_topic = "/livox/lidar";
    std::string imu_topic = "/livox/imu";
    
    // Keyframe thresholds
    double keyframe_delta_trans = 0.5;  // meters
    double keyframe_delta_deg = 10.0;   // degrees
    
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
 * @brief Keyframe structure for PGO preparation
 */
struct KeyFrame {
    size_t id;
    double timestamp;
    Eigen::Matrix3d rotation;
    Eigen::Vector3d position;
    PointCloudXYZI::Ptr body_cloud;  // Point cloud in body frame
    
    KeyFrame() : id(0), timestamp(0.0), 
                 rotation(Eigen::Matrix3d::Identity()),
                 position(Eigen::Vector3d::Zero()),
                 body_cloud(new PointCloudXYZI()) {}
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
    const std::vector<KeyFrame>& keyframes() const { return keyframes_; }
    
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
     * @brief Save keyframe poses
     * @param path Output file path
     */
    void saveKeyframePoses(const std::string& path) const;
    
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
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*> voxel_map_;
    
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
    
    // Downsampling filter
    pcl::VoxelGrid<PointType> downsample_filter_;
    double filter_size_surf_min_ = 0.1;
    
    // IMU settings
    bool imu_en_ = true;
    bool gravity_est_en_ = true;
    bool ba_bg_est_en_ = true;
    int imu_int_frame_ = 3;
    
    // Keyframe management
    std::vector<KeyFrame> keyframes_;
    Eigen::Vector3d last_kf_pos_;
    Eigen::Matrix3d last_kf_rot_;
    
    // Trajectory storage
    std::vector<TimedPose> trajectory_;
    
    // Incremental saving
    std::string incremental_output_dir_;
    bool incremental_save_enabled_ = false;
    
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
};

} // namespace livo2_offline

#endif // OFFLINE_LIV_MAPPER_H
