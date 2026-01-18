/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#include "LIVMapper.h"
#include <pcl/common/io.h>
#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

LIVMapper::LIVMapper()
    : Node("laserMapping"),
      extT(0, 0, 0),
      extR(M3D::Identity())
{
  extrinT.assign(3, 0.0);
  extrinR.assign(9, 0.0);

  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());

  readParameters();
  VoxelMapConfig voxel_config;
  loadVoxelConfig(this, voxel_config);

  feats_undistort.reset(new PointCloudXYZI());
  feats_down_body.reset(new PointCloudXYZI());
  feats_down_world.reset(new PointCloudXYZI());
  pcl_w_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_pub.reset(new PointCloudXYZI());
  pcl_wait_save.reset(new PointCloudXYZI());
  pcl_wait_save_intensity.reset(new PointCloudXYZI());
  voxelmap_manager.reset(new VoxelMapManager(voxel_config, voxel_map));
  root_dir = ROOT_DIR;
  initializeFiles();
  initializeComponents();
  path.header.stamp = this->now();
  path.header.frame_id = "camera_init";
}

LIVMapper::~LIVMapper() {}

void LIVMapper::readParameters()
{
  this->declare_parameter<std::string>("common.lid_topic", "/livox/lidar");
  this->declare_parameter<std::string>("common.imu_topic", "/livox/imu");
  this->declare_parameter<bool>("common.ros_driver_bug_fix", false);
  this->declare_parameter<int>("common.lidar_en", 1);
  this->declare_parameter<double>("time_offset.imu_time_offset", 0.0);
  this->declare_parameter<double>("time_offset.lidar_time_offset", 0.0);
  this->declare_parameter<bool>("uav.imu_rate_odom", false);
  this->declare_parameter<bool>("uav.gravity_align_en", false);

  this->declare_parameter<std::string>("evo.seq_name", "01");
  this->declare_parameter<bool>("evo.pose_output_en", false);
  this->declare_parameter<double>("imu.gyr_cov", 1.0);
  this->declare_parameter<double>("imu.acc_cov", 1.0);
  this->declare_parameter<int>("imu.imu_int_frame", 3);
  this->declare_parameter<bool>("imu.imu_en", false);
  this->declare_parameter<bool>("imu.gravity_est_en", true);
  this->declare_parameter<bool>("imu.ba_bg_est_en", true);

  this->declare_parameter<double>("preprocess.blind", 0.01);
  this->declare_parameter<double>("preprocess.filter_size_surf", 0.5);
  this->declare_parameter<bool>("preprocess.hilti_en", false);
  this->declare_parameter<int>("preprocess.lidar_type", AVIA);
  this->declare_parameter<int>("preprocess.scan_line", 6);
  this->declare_parameter<int>("preprocess.point_filter_num", 3);
  this->declare_parameter<bool>("preprocess.feature_extract_enabled", false);

  this->declare_parameter<int>("pcd_save.interval", -1);
  this->declare_parameter<bool>("pcd_save.pcd_save_en", false);
  this->declare_parameter<int>("pcd_save.type", 0);

  this->declare_parameter<bool>("pcd_save.colmap_output_en", false);
  this->declare_parameter<double>("pcd_save.filter_size_pcd", 0.5);
  this->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_T", std::vector<double>());
  this->declare_parameter<std::vector<double>>("extrin_calib.extrinsic_R", std::vector<double>());
  this->declare_parameter<int>("publish.pub_scan_num", 1);
  this->declare_parameter<bool>("publish.pub_effect_point_en", false);
  this->declare_parameter<bool>("publish.dense_map_en", false);

  // Relocalization parameters
  this->declare_parameter<bool>("relocalization.enabled", false);
  this->declare_parameter<std::string>("relocalization.prior_map_path", "");
  this->declare_parameter<std::string>("relocalization.map_update_mode", "full");
  this->declare_parameter<double>("relocalization.initial_pose.x", 0.0);
  this->declare_parameter<double>("relocalization.initial_pose.y", 0.0);
  this->declare_parameter<double>("relocalization.initial_pose.z", 0.0);
  this->declare_parameter<double>("relocalization.initial_pose.roll", 0.0);
  this->declare_parameter<double>("relocalization.initial_pose.pitch", 0.0);
  this->declare_parameter<double>("relocalization.initial_pose.yaw", 0.0);

  // TF parameters
  this->declare_parameter<std::string>("tf.map_frame", "map");
  this->declare_parameter<std::string>("tf.odom_frame", "odom");
  this->declare_parameter<std::string>("tf.body_frame", "body");
  this->declare_parameter<bool>("tf.publish_tf", true);

  // Map save parameters
  this->declare_parameter<bool>("map_save.enabled", false);
  this->declare_parameter<std::string>("map_save.path", "");

  this->get_parameter("common.lid_topic", lid_topic);
  this->get_parameter("common.imu_topic", imu_topic);
  this->get_parameter("common.ros_driver_bug_fix", ros_driver_fix_en);
  this->get_parameter("common.lidar_en", lidar_en);
  this->get_parameter("time_offset.imu_time_offset", imu_time_offset);
  this->get_parameter("time_offset.lidar_time_offset", lidar_time_offset);
  this->get_parameter("uav.imu_rate_odom", imu_prop_enable);
  this->get_parameter("uav.gravity_align_en", gravity_align_en);

  this->get_parameter("evo.seq_name", seq_name);
  this->get_parameter("evo.pose_output_en", pose_output_en);
  this->get_parameter("imu.gyr_cov", gyr_cov);
  this->get_parameter("imu.acc_cov", acc_cov);
  this->get_parameter("imu.imu_int_frame", imu_int_frame);
  this->get_parameter("imu.imu_en", imu_en);
  this->get_parameter("imu.gravity_est_en", gravity_est_en);
  this->get_parameter("imu.ba_bg_est_en", ba_bg_est_en);

  this->get_parameter("preprocess.blind", p_pre->blind);
  this->get_parameter("preprocess.filter_size_surf", filter_size_surf_min);
  this->get_parameter("preprocess.hilti_en", hilti_en);
  this->get_parameter("preprocess.lidar_type", p_pre->lidar_type);
  this->get_parameter("preprocess.scan_line", p_pre->N_SCANS);
  this->get_parameter("preprocess.point_filter_num", p_pre->point_filter_num);
  this->get_parameter("preprocess.feature_extract_enabled", p_pre->feature_enabled);

  this->get_parameter("pcd_save.interval", pcd_save_interval);
  this->get_parameter("pcd_save.pcd_save_en", pcd_save_en);
  this->get_parameter("pcd_save.type", pcd_save_type);

  this->get_parameter("pcd_save.colmap_output_en", colmap_output_en);
  this->get_parameter("pcd_save.filter_size_pcd", filter_size_pcd);
  this->get_parameter("extrin_calib.extrinsic_T", extrinT);
  this->get_parameter("extrin_calib.extrinsic_R", extrinR);
  this->get_parameter("publish.pub_scan_num", pub_scan_num);
  this->get_parameter("publish.pub_effect_point_en", pub_effect_point_en);
  this->get_parameter("publish.dense_map_en", dense_map_en);

  // Get relocalization parameters
  this->get_parameter("relocalization.enabled", localization_mode_en_);
  this->get_parameter("relocalization.prior_map_path", prior_map_path_);
  std::string map_update_mode_str;
  this->get_parameter("relocalization.map_update_mode", map_update_mode_str);
  if (map_update_mode_str == "none") {
    map_update_mode_ = MAP_UPDATE_NONE;
  } else if (map_update_mode_str == "incremental") {
    map_update_mode_ = MAP_UPDATE_INCREMENTAL;
  } else {
    map_update_mode_ = MAP_UPDATE_FULL;
  }
  this->get_parameter("relocalization.initial_pose.x", init_pose_x_);
  this->get_parameter("relocalization.initial_pose.y", init_pose_y_);
  this->get_parameter("relocalization.initial_pose.z", init_pose_z_);
  this->get_parameter("relocalization.initial_pose.roll", init_pose_roll_);
  this->get_parameter("relocalization.initial_pose.pitch", init_pose_pitch_);
  this->get_parameter("relocalization.initial_pose.yaw", init_pose_yaw_);

  // Get TF parameters
  this->get_parameter("tf.map_frame", map_frame_);
  this->get_parameter("tf.odom_frame", odom_frame_);
  this->get_parameter("tf.body_frame", body_frame_);
  this->get_parameter("tf.publish_tf", publish_tf_);

  // Get map save parameters
  this->get_parameter("map_save.enabled", map_save_en_);
  this->get_parameter("map_save.path", map_save_path_);

  p_pre->blind_sqr = p_pre->blind * p_pre->blind;
}

void LIVMapper::initializeComponents()
{
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  extT << VEC_FROM_ARRAY(extrinT);
  extR << MAT_FROM_ARRAY(extrinR);

  voxelmap_manager->extT_ << VEC_FROM_ARRAY(extrinT);
  voxelmap_manager->extR_ << MAT_FROM_ARRAY(extrinR);

  p_imu->set_extrinsic(extT, extR);
  p_imu->set_gyr_cov_scale(V3D(gyr_cov, gyr_cov, gyr_cov));
  p_imu->set_acc_cov_scale(V3D(acc_cov, acc_cov, acc_cov));
  p_imu->set_gyr_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_acc_bias_cov(V3D(0.0001, 0.0001, 0.0001));
  p_imu->set_imu_init_frame_num(imu_int_frame);

  if (!imu_en) p_imu->disable_imu();
  if (!gravity_est_en) p_imu->disable_gravity_est();
  if (!ba_bg_est_en) p_imu->disable_bias_est();

  slam_mode_ = imu_en ? ONLY_LIO : ONLY_LO;

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Relocalization mode initialization
  if (localization_mode_en_) {
    std::cout << GREEN << "[LIVMapper] Localization mode enabled" << RESET << std::endl;

    // Load prior map
    if (!prior_map_path_.empty()) {
      std::cout << "[LIVMapper] Loading prior map from: " << prior_map_path_ << std::endl;
      if (voxelmap_manager->loadMap(prior_map_path_)) {
        prior_map_loaded_ = true;
        lidar_map_inited = true;  // Map is already initialized
        std::cout << GREEN << "[LIVMapper] Prior map loaded successfully" << RESET << std::endl;
      } else {
        std::cerr << RED << "[LIVMapper] Failed to load prior map!" << RESET << std::endl;
      }
    }

    // Set initial pose
    _state.pos_end = V3D(init_pose_x_, init_pose_y_, init_pose_z_);

    // Convert RPY to rotation matrix
    Eigen::AngleAxisd rollAngle(init_pose_roll_, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(init_pose_pitch_, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(init_pose_yaw_, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    _state.rot_end = q.toRotationMatrix();

    std::cout << "[LIVMapper] Initial pose set to: "
              << "pos=[" << init_pose_x_ << ", " << init_pose_y_ << ", " << init_pose_z_ << "], "
              << "rpy=[" << init_pose_roll_ << ", " << init_pose_pitch_ << ", " << init_pose_yaw_ << "]" << std::endl;

    // Print map update mode
    std::string mode_str = (map_update_mode_ == MAP_UPDATE_NONE) ? "none" :
                           (map_update_mode_ == MAP_UPDATE_INCREMENTAL) ? "incremental" : "full";
    std::cout << "[LIVMapper] Map update mode: " << mode_str << std::endl;
  } else {
    std::cout << "[LIVMapper] SLAM mode enabled (mapping)" << std::endl;

    // Set default map save path if not specified
    if (map_save_en_ && map_save_path_.empty()) {
      map_save_path_ = std::string(ROOT_DIR) + "Log/map/voxelmap.bin";
    }
  }
}

void LIVMapper::initializeFiles() 
{
  if (pcd_save_en && colmap_output_en)
  {
      const std::string folderPath = std::string(ROOT_DIR) + "/scripts/colmap_output.sh";
      
      std::string chmodCommand = "chmod +x " + folderPath;
      
      int chmodRet = system(chmodCommand.c_str());  
      if (chmodRet != 0) {
          std::cerr << "Failed to set execute permissions for the script." << std::endl;
          return;
      }

      int executionRet = system(folderPath.c_str());
      if (executionRet != 0) {
          std::cerr << "Failed to execute the script." << std::endl;
          return;
      }
  }
  if(colmap_output_en) fout_points.open(std::string(ROOT_DIR) + "Log/Colmap/sparse/0/points3D.txt", std::ios::out);
  if(pcd_save_en) fout_lidar_pos.open(std::string(ROOT_DIR) + "Log/pcd/lidar_poses.txt", std::ios::out);
  fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"), std::ios::out);
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"), std::ios::out);
}

void LIVMapper::initializeSubscribersAndPublishers() 
{
  using std::placeholders::_1;
  
  if (p_pre->lidar_type == AVIA) {
    sub_pcl_livox = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
      lid_topic, 200000, std::bind(&LIVMapper::livox_pcl_cbk, this, _1));
  } else {
    sub_pcl = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lid_topic, 200000, std::bind(&LIVMapper::standard_pcl_cbk, this, _1));
  }
  sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, 200000, std::bind(&LIVMapper::imu_cbk, this, _1));
  
  pubLaserCloudFullRes = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_registered", 100);
  pubNormal = this->create_publisher<visualization_msgs::msg::MarkerArray>("visualization_marker", 100);
  pubLaserCloudEffect = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_effected", 100);
  pubLaserCloudMap = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Laser_map", 100);
  pubOdomAftMapped = this->create_publisher<nav_msgs::msg::Odometry>("/aft_mapped_to_init", 10);
  pubPath = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  plane_pub = this->create_publisher<visualization_msgs::msg::Marker>("/planner_normal", 1);
  voxel_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/voxels", 1);
  pubLaserCloudDyn = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj", 100);
  pubLaserCloudDynRmed = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_removed", 100);
  pubLaserCloudDynDbg = this->create_publisher<sensor_msgs::msg::PointCloud2>("/dyn_obj_dbg_hist", 100);
  mavros_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
  pubImuPropOdom = this->create_publisher<nav_msgs::msg::Odometry>("/LIVO2/imu_propagate", 10000);
  imu_prop_timer = this->create_wall_timer(
    std::chrono::milliseconds(4), std::bind(&LIVMapper::imu_prop_callback, this));
  voxelmap_manager->voxel_map_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/planes", 10000);
}

void LIVMapper::handleFirstFrame() 
{
  if (!is_first_frame)
  {
    _first_lidar_time = LidarMeasures.last_lio_update_time;
    p_imu->first_lidar_time = _first_lidar_time; // Only for IMU data log
    is_first_frame = true;
    cout << "FIRST LIDAR FRAME!" << endl;
  }
}

void LIVMapper::gravityAlignment() 
{
  if (!p_imu->imu_need_init && !gravity_align_finished) 
  {
    std::cout << "Gravity Alignment Starts" << std::endl;
    V3D ez(0, 0, -1), gz(_state.gravity);
    Quaterniond G_q_I0 = Quaterniond::FromTwoVectors(gz, ez);
    M3D G_R_I0 = G_q_I0.toRotationMatrix();

    _state.pos_end = G_R_I0 * _state.pos_end;
    _state.rot_end = G_R_I0 * _state.rot_end;
    _state.vel_end = G_R_I0 * _state.vel_end;
    _state.gravity = G_R_I0 * _state.gravity;
    gravity_align_finished = true;
    std::cout << "Gravity Alignment Finished" << std::endl;
  }
}

void LIVMapper::processImu() 
{
  // double t0 = omp_get_wtime();

  p_imu->Process2(LidarMeasures, _state, feats_undistort);

  if (gravity_align_en) gravityAlignment();

  state_propagat = _state;
  voxelmap_manager->state_ = _state;
  voxelmap_manager->feats_undistort_ = feats_undistort;

  // double t_prop = omp_get_wtime();

  // std::cout << "[ Mapping ] feats_undistort: " << feats_undistort->size() << std::endl;
  // std::cout << "[ Mapping ] predict cov: " << _state.cov.diagonal().transpose() << std::endl;
  // std::cout << "[ Mapping ] predict sta: " << state_propagat.pos_end.transpose() << state_propagat.vel_end.transpose() << std::endl;
}

void LIVMapper::stateEstimationAndMapping() 
{
  switch (LidarMeasures.lio_vio_flg) 
  {
    case LIO:
    case LO:
      handleLIO();
      break;
    default:
      break;
  }
}


void LIVMapper::handleLIO() 
{    
  euler_cur = RotMtoEuler(_state.rot_end);
  fout_pre << setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
           << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
           << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << endl;
           
  if (feats_undistort->empty() || (feats_undistort == nullptr)) 
  {
    std::cout << "[ LIO ]: No point!!!" << std::endl;
    return;
  }

  double t0 = omp_get_wtime();

  downSizeFilterSurf.setInputCloud(feats_undistort);
  downSizeFilterSurf.filter(*feats_down_body);
  
  double t_down = omp_get_wtime();

  feats_down_size = feats_down_body->points.size();
  voxelmap_manager->feats_down_body_ = feats_down_body;
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, feats_down_world);
  voxelmap_manager->feats_down_world_ = feats_down_world;
  voxelmap_manager->feats_down_size_ = feats_down_size;
  
  if (!lidar_map_inited) 
  {
    lidar_map_inited = true;
    voxelmap_manager->BuildVoxelMap();
  }

  double t1 = omp_get_wtime();

  voxelmap_manager->StateEstimation(state_propagat);
  _state = voxelmap_manager->state_;
  _pv_list = voxelmap_manager->pv_list_;

  double t2 = omp_get_wtime();

  if (imu_prop_enable) 
  {
    ekf_finish_once = true;
    latest_ekf_state = _state;
    latest_ekf_time = LidarMeasures.last_lio_update_time;
    state_update_flg = true;
  }

  if (pose_output_en) 
  {
    static bool pos_opend = false;
    // static int ocount = 0;
    std::ofstream outFile, evoFile;
    if (!pos_opend) 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::out);
      pos_opend = true;
      if (!evoFile.is_open()) RCLCPP_ERROR(this->get_logger(), "open fail\n");
    } 
    else 
    {
      evoFile.open(std::string(ROOT_DIR) + "Log/result/" + seq_name + ".txt", std::ios::app);
      if (!evoFile.is_open()) RCLCPP_ERROR(this->get_logger(), "open fail\n");
    }
    Eigen::Matrix4d outT;
    Eigen::Quaterniond q(_state.rot_end);
    evoFile << std::fixed;
    evoFile << LidarMeasures.last_lio_update_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " "
            << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
  }
  
  euler_cur = RotMtoEuler(_state.rot_end);
  tf2::Quaternion q;
  q.setRPY(euler_cur(0), euler_cur(1), euler_cur(2));
  geoQuat.x = q.x();
  geoQuat.y = q.y();
  geoQuat.z = q.z();
  geoQuat.w = q.w();
  publish_odometry();

  double t3 = omp_get_wtime();

  PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI());
  transformLidar(_state.rot_end, _state.pos_end, feats_down_body, world_lidar);
  for (size_t i = 0; i < world_lidar->points.size(); i++)
  {
    voxelmap_manager->pv_list_[i].point_w << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
    M3D point_crossmat = voxelmap_manager->cross_mat_list_[i];
    M3D var = voxelmap_manager->body_cov_list_[i];
    var = (_state.rot_end * extR) * var * (_state.rot_end * extR).transpose() +
          (-point_crossmat) * _state.cov.block<3, 3>(0, 0) * (-point_crossmat).transpose() + _state.cov.block<3, 3>(3, 3);
    voxelmap_manager->pv_list_[i].var = var;
  }

  // Map update based on mode
  switch (map_update_mode_) {
    case MAP_UPDATE_NONE:
      // Pure localization - no map update
      std::cout << "[ LIO ] Localization mode - no map update" << std::endl;
      break;
    case MAP_UPDATE_INCREMENTAL:
      // Only add new voxels
      voxelmap_manager->UpdateVoxelMapIncremental(voxelmap_manager->pv_list_);
      std::cout << "[ LIO ] Incremental map update" << std::endl;
      break;
    case MAP_UPDATE_FULL:
    default:
      // Full update (normal SLAM behavior)
      voxelmap_manager->UpdateVoxelMap(voxelmap_manager->pv_list_);
      std::cout << "[ LIO ] Full map update" << std::endl;
      break;
  }
  _pv_list = voxelmap_manager->pv_list_;
  
  double t4 = omp_get_wtime();

  if(voxelmap_manager->config_setting_.map_sliding_en)
  {
    voxelmap_manager->mapSliding();
  }
  
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++) 
  {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
  }
  *pcl_w_wait_pub = *laserCloudWorld;

  publish_frame_world();
  if (pub_effect_point_en) publish_effect_world(voxelmap_manager->ptpl_list_);
  if (voxelmap_manager->config_setting_.is_pub_plane_map_) voxelmap_manager->pubVoxelMap();
  publish_path();
  publish_mavros();

  frame_num++;
  aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t4 - t0) / frame_num;

  // aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + (t2 - t1) / frame_num;
  // aver_time_map_inre = aver_time_map_inre * (frame_num - 1) / frame_num + (t4 - t3) / frame_num;
  // aver_time_solve = aver_time_solve * (frame_num - 1) / frame_num + (solve_time) / frame_num;
  // aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1) / frame_num + solve_const_H_time / frame_num;
  // printf("[ mapping time ]: per scan: propagation %0.6f downsample: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f \n"
  //         "[ mapping time ]: average: icp: %0.6f construct H: %0.6f, total: %0.6f \n",
  //         t_prop - t0, t1 - t_prop, match_time, solve_time, t3 - t1, t5 - t3, t5 - t0, aver_time_icp, aver_time_const_H_time, aver_time_consu);

  // printf("\033[1;36m[ LIO mapping time ]: current scan: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n"
  //         "\033[1;36m[ LIO mapping time ]: average: icp: %0.6f secs, map incre: %0.6f secs, total: %0.6f secs.\033[0m\n",
  //         t2 - t1, t4 - t3, t4 - t0, aver_time_icp, aver_time_map_inre, aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m|                         LIO Mapping Time                    |\033[0m\n");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;34m| %-29s | %-27s |\033[0m\n", "Algorithm Stage", "Time (secs)");
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "DownSample", t_down - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "ICP", t2 - t1);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "updateVoxelMap", t4 - t3);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Current Total Time", t4 - t0);
  printf("\033[1;36m| %-29s | %-27f |\033[0m\n", "Average Total Time", aver_time_consu);
  printf("\033[1;34m+-------------------------------------------------------------+\033[0m\n");

  euler_cur = RotMtoEuler(_state.rot_end);
  fout_out << std::setw(20) << LidarMeasures.last_lio_update_time - _first_lidar_time << " " << euler_cur.transpose() * 57.3 << " "
            << _state.pos_end.transpose() << " " << _state.vel_end.transpose() << " " << _state.bias_g.transpose() << " "
            << _state.bias_a.transpose() << " " << V3D(_state.inv_expo_time, 0, 0).transpose() << " " << feats_undistort->points.size() << std::endl;
}

void LIVMapper::savePCD() 
{
  if (pcd_save_en && (pcl_wait_save->points.size() > 0 || pcl_wait_save_intensity->points.size() > 0) && pcd_save_interval < 0) 
  {
    std::string raw_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_raw_points.pcd";
    std::string downsampled_points_dir = std::string(ROOT_DIR) + "Log/pcd/all_downsampled_points.pcd";
    pcl::PCDWriter pcd_writer;

    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
      // Convert PointXYZINormal to PointXYZI for voxel filtering
      pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::copyPointCloud(*pcl_wait_save, *temp_cloud);
      voxel_filter.setInputCloud(temp_cloud);
      voxel_filter.setLeafSize(filter_size_pcd, filter_size_pcd, filter_size_pcd);
      voxel_filter.filter(*downsampled_cloud);
  
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save); // Save the raw point cloud data
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir 
                << " with point count: " << pcl_wait_save->points.size() << RESET << std::endl;
      
      pcd_writer.writeBinary(downsampled_points_dir, *downsampled_cloud); // Save the downsampled point cloud data
      std::cout << GREEN << "Downsampled point cloud data saved to: " << downsampled_points_dir 
                << " with point count after filtering: " << downsampled_cloud->points.size() << RESET << std::endl;
    }
    
    if (pcl_wait_save_intensity->points.size() > 0)
    {
      pcd_writer.writeBinary(raw_points_dir, *pcl_wait_save_intensity);
      std::cout << GREEN << "Raw point cloud data saved to: " << raw_points_dir
                << " with point count: " << pcl_wait_save_intensity->points.size() << RESET << std::endl;
    }
  }

  // Save voxel map if enabled (only in SLAM mode, not localization mode)
  if (map_save_en_ && !localization_mode_en_ && lidar_map_inited) {
    // Create directory if it doesn't exist
    std::string map_dir = std::string(ROOT_DIR) + "Log/map/";
    std::string mkdir_cmd = "mkdir -p " + map_dir;
    int ret = system(mkdir_cmd.c_str());
    (void)ret;  // Suppress unused return value warning

    if (map_save_path_.empty()) {
      map_save_path_ = map_dir + "voxelmap.bin";
    }

    std::cout << "[LIVMapper] Saving voxel map to: " << map_save_path_ << std::endl;
    if (voxelmap_manager->saveMap(map_save_path_)) {
      std::cout << GREEN << "[LIVMapper] Voxel map saved successfully" << RESET << std::endl;
    } else {
      std::cerr << RED << "[LIVMapper] Failed to save voxel map" << RESET << std::endl;
    }
  }
}

void LIVMapper::run() 
{
  rclcpp::Rate rate(5000);
  while (rclcpp::ok()) 
  {
    rclcpp::spin_some(shared_from_this());
    if (!sync_packages(LidarMeasures)) 
    {
      rate.sleep();
      continue;
    }
    handleFirstFrame();

    processImu();

    // if (!p_imu->imu_time_init) continue;

    stateEstimationAndMapping();
  }
  savePCD();
}

void LIVMapper::prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr)
{
  double mean_acc_norm = p_imu->IMU_mean_acc_norm;
  acc_avr = acc_avr * G_m_s2 / mean_acc_norm - imu_prop_state.bias_a;
  angvel_avr -= imu_prop_state.bias_g;

  M3D Exp_f = Exp(angvel_avr, dt);
  /* propogation of IMU attitude */
  imu_prop_state.rot_end = imu_prop_state.rot_end * Exp_f;

  /* Specific acceleration (global frame) of IMU */
  V3D acc_imu = imu_prop_state.rot_end * acc_avr + V3D(imu_prop_state.gravity[0], imu_prop_state.gravity[1], imu_prop_state.gravity[2]);

  /* propogation of IMU */
  imu_prop_state.pos_end = imu_prop_state.pos_end + imu_prop_state.vel_end * dt + 0.5 * acc_imu * dt * dt;

  /* velocity of IMU */
  imu_prop_state.vel_end = imu_prop_state.vel_end + acc_imu * dt;
}

void LIVMapper::imu_prop_callback()
{
  if (p_imu->imu_need_init || !new_imu || !ekf_finish_once) { return; }
  mtx_buffer_imu_prop.lock();
  new_imu = false; // 控制propagate频率和IMU频率一致
  if (imu_prop_enable && !prop_imu_buffer.empty())
  {
    static double last_t_from_lidar_end_time = 0;
    if (state_update_flg)
    {
      imu_propagate = latest_ekf_state;
      // drop all useless imu pkg
      while ((!prop_imu_buffer.empty() && rclcpp::Time(prop_imu_buffer.front().header.stamp).seconds() < latest_ekf_time))
      {
        prop_imu_buffer.pop_front();
      }
      last_t_from_lidar_end_time = 0;
      for (size_t i = 0; i < prop_imu_buffer.size(); i++)
      {
        double t_from_lidar_end_time = rclcpp::Time(prop_imu_buffer[i].header.stamp).seconds() - latest_ekf_time;
        double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
        // cout << "prop dt" << dt << ", " << t_from_lidar_end_time << ", " << last_t_from_lidar_end_time << endl;
        V3D acc_imu(prop_imu_buffer[i].linear_acceleration.x, prop_imu_buffer[i].linear_acceleration.y, prop_imu_buffer[i].linear_acceleration.z);
        V3D omg_imu(prop_imu_buffer[i].angular_velocity.x, prop_imu_buffer[i].angular_velocity.y, prop_imu_buffer[i].angular_velocity.z);
        prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
        last_t_from_lidar_end_time = t_from_lidar_end_time;
      }
      state_update_flg = false;
    }
    else
    {
      V3D acc_imu(newest_imu.linear_acceleration.x, newest_imu.linear_acceleration.y, newest_imu.linear_acceleration.z);
      V3D omg_imu(newest_imu.angular_velocity.x, newest_imu.angular_velocity.y, newest_imu.angular_velocity.z);
      double t_from_lidar_end_time = rclcpp::Time(newest_imu.header.stamp).seconds() - latest_ekf_time;
      double dt = t_from_lidar_end_time - last_t_from_lidar_end_time;
      prop_imu_once(imu_propagate, dt, acc_imu, omg_imu);
      last_t_from_lidar_end_time = t_from_lidar_end_time;
    }

    V3D posi, vel_i;
    Eigen::Quaterniond q;
    posi = imu_propagate.pos_end;
    vel_i = imu_propagate.vel_end;
    q = Eigen::Quaterniond(imu_propagate.rot_end);
    imu_prop_odom.header.frame_id = "world";
    imu_prop_odom.header.stamp = newest_imu.header.stamp;
    imu_prop_odom.pose.pose.position.x = posi.x();
    imu_prop_odom.pose.pose.position.y = posi.y();
    imu_prop_odom.pose.pose.position.z = posi.z();
    imu_prop_odom.pose.pose.orientation.w = q.w();
    imu_prop_odom.pose.pose.orientation.x = q.x();
    imu_prop_odom.pose.pose.orientation.y = q.y();
    imu_prop_odom.pose.pose.orientation.z = q.z();
    imu_prop_odom.twist.twist.linear.x = vel_i.x();
    imu_prop_odom.twist.twist.linear.y = vel_i.y();
    imu_prop_odom.twist.twist.linear.z = vel_i.z();
    pubImuPropOdom->publish(imu_prop_odom);
  }
  mtx_buffer_imu_prop.unlock();
}

void LIVMapper::transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
  PointCloudXYZI().swap(*trans_cloud);
  trans_cloud->reserve(input_cloud->size());
  for (size_t i = 0; i < input_cloud->size(); i++)
  {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    p = (rot * (extR * p + extT) + t);
    PointType pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void LIVMapper::pointBodyToWorld(const PointType &pi, PointType &po)
{
  V3D p_body(pi.x, pi.y, pi.z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po.x = p_global(0);
  po.y = p_global(1);
  po.z = p_global(2);
  po.intensity = pi.intensity;
}

template <typename T> void LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
  V3D p_body(pi[0], pi[1], pi[2]);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

template <typename T> Matrix<T, 3, 1> LIVMapper::pointBodyToWorld(const Matrix<T, 3, 1> &pi)
{
  V3D p(pi[0], pi[1], pi[2]);
  p = (_state.rot_end * (extR * p + extT) + _state.pos_end);
  Matrix<T, 3, 1> po(p[0], p[1], p[2]);
  return po;
}

void LIVMapper::RGBpointBodyToWorld(PointType const *const pi, PointType *const po)
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(_state.rot_end * (extR * p_body + extT) + _state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LIVMapper::RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu(extR * p_body_lidar + extT);

  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void LIVMapper::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (!lidar_en) return;
  mtx_buffer.lock();

  double cur_head_time = rclcpp::Time(msg->header.stamp).seconds() + lidar_time_offset;
  // cout<<"got feature"<<endl;
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // RCLCPP_INFO(this->get_logger(), "get point cloud at time: %.6f", rclcpp::Time(msg->header.stamp).seconds());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg_in)
{
  if (!lidar_en) return;
  mtx_buffer.lock();
  livox_ros_driver2::msg::CustomMsg::SharedPtr msg = std::make_shared<livox_ros_driver2::msg::CustomMsg>(*msg_in);
  // if ((abs(rclcpp::Time(msg->header.stamp).seconds() - last_timestamp_lidar) > 0.2 && last_timestamp_lidar > 0) || sync_jump_flag)
  // {
  //   RCLCPP_WARN(this->get_logger(), "lidar jumps %.3f\n", rclcpp::Time(msg->header.stamp).seconds() - last_timestamp_lidar);
  //   sync_jump_flag = true;
  //   msg->header.stamp = rclcpp::Time(static_cast<int64_t>((last_timestamp_lidar + 0.1) * 1e9));
  // }
  double msg_time = rclcpp::Time(msg->header.stamp).seconds();
  if (abs(last_timestamp_imu - msg_time) > 1.0 && !imu_buffer.empty())
  {
    double timediff_imu_wrt_lidar = last_timestamp_imu - msg_time;
    printf("\033[95mSelf sync IMU and LiDAR, HARD time lag is %.10lf \n\033[0m", timediff_imu_wrt_lidar - 0.100);
    // imu_time_offset = timediff_imu_wrt_lidar;
  }

  double cur_head_time = msg_time;
  RCLCPP_INFO(this->get_logger(), "Get LiDAR, its header time: %.6f", cur_head_time);
  if (cur_head_time < last_timestamp_lidar)
  {
    RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
    lid_raw_data_buffer.clear();
  }
  // RCLCPP_INFO(this->get_logger(), "get point cloud at time: %.6f", rclcpp::Time(msg->header.stamp).seconds());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);

  if (!ptr || ptr->empty()) {
    RCLCPP_ERROR(this->get_logger(), "Received an empty point cloud");
    mtx_buffer.unlock();
    return;
  }

  lid_raw_data_buffer.push_back(ptr);
  lid_header_time_buffer.push_back(cur_head_time);
  last_timestamp_lidar = cur_head_time;

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

void LIVMapper::imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in)
{
  if (!imu_en) return;

  if (last_timestamp_lidar < 0.0) return;
  // RCLCPP_INFO(this->get_logger(), "get imu at time: %.6f", rclcpp::Time(msg_in->header.stamp).seconds());
  sensor_msgs::msg::Imu::SharedPtr msg = std::make_shared<sensor_msgs::msg::Imu>(*msg_in);
  double original_time = rclcpp::Time(msg->header.stamp).seconds();
  double adjusted_time = original_time - imu_time_offset;
  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(adjusted_time * 1e9));
  double timestamp = adjusted_time;

  if (fabs(last_timestamp_lidar - timestamp) > 0.5 && (!ros_driver_fix_en))
  {
    RCLCPP_WARN(this->get_logger(), "IMU and LiDAR not synced! delta time: %lf .\n", last_timestamp_lidar - timestamp);
  }

  if (ros_driver_fix_en) timestamp += std::round(last_timestamp_lidar - timestamp);
  msg->header.stamp = rclcpp::Time(static_cast<int64_t>(timestamp * 1e9));

  mtx_buffer.lock();

  if (last_timestamp_imu > 0.0 && timestamp < last_timestamp_imu)
  {
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    RCLCPP_ERROR(this->get_logger(), "imu loop back, offset: %lf \n", last_timestamp_imu - timestamp);
    return;
  }

  // if (last_timestamp_imu > 0.0 && timestamp > last_timestamp_imu + 0.2)
  // {

  //   RCLCPP_WARN(this->get_logger(), "imu time stamp Jumps %0.4lf seconds \n", timestamp - last_timestamp_imu);
  //   mtx_buffer.unlock();
  //   sig_buffer.notify_all();
  //   return;
  // }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
  mtx_buffer.unlock();
  if (imu_prop_enable)
  {
    mtx_buffer_imu_prop.lock();
    if (imu_prop_enable && !p_imu->imu_need_init) { prop_imu_buffer.push_back(*msg); }
    newest_imu = *msg;
    new_imu = true;
    mtx_buffer_imu_prop.unlock();
  }
  sig_buffer.notify_all();
}


bool LIVMapper::sync_packages(LidarMeasureGroup &meas)
{
  if (lid_raw_data_buffer.empty() && lidar_en) return false;
  if (imu_buffer.empty() && imu_en) return false;

  switch (slam_mode_)
  {
  case ONLY_LIO:
  {
    if (meas.last_lio_update_time < 0.0) meas.last_lio_update_time = lid_header_time_buffer.front();
    if (!lidar_pushed)
    {
      // If not push the lidar into measurement data buffer
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      if (meas.lidar->points.size() <= 1) return false;

      meas.lidar_frame_beg_time = lid_header_time_buffer.front();                                                // generate lidar_frame_beg_time
      meas.lidar_frame_end_time = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      meas.pcl_proc_cur = meas.lidar;
      lidar_pushed = true;                                                                                       // flag
    }

    if (imu_en && last_timestamp_imu < meas.lidar_frame_end_time)
    { // waiting imu message needs to be
      // larger than _lidar_frame_end_time,
      // make sure complete propagate.
      // ROS_ERROR("out sync");
      return false;
    }

    struct MeasureGroup m; // standard method to keep imu message.

    m.imu.clear();
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    while (!imu_buffer.empty())
    {
      if (rclcpp::Time(imu_buffer.front()->header.stamp).seconds() > meas.lidar_frame_end_time) break;
      m.imu.push_back(imu_buffer.front());
      imu_buffer.pop_front();
    }
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();

    meas.lio_vio_flg = LIO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    // ROS_INFO("ONlY HAS LiDAR and IMU, NO IMAGE!");
    lidar_pushed = false; // sync one whole lidar scan.
    return true;

    break;
  }


  case ONLY_LO:
  {
    if (!lidar_pushed) 
    { 
      // If not in lidar scan, need to generate new meas
      if (lid_raw_data_buffer.empty())  return false;
      meas.lidar = lid_raw_data_buffer.front(); // push the first lidar topic
      meas.lidar_frame_beg_time = lid_header_time_buffer.front(); // generate lidar_beg_time
      meas.lidar_frame_end_time  = meas.lidar_frame_beg_time + meas.lidar->points.back().curvature / double(1000); // calc lidar scan end time
      lidar_pushed = true;             
    }
    struct MeasureGroup m; // standard method to keep imu message.
    m.lio_time = meas.lidar_frame_end_time;
    mtx_buffer.lock();
    lid_raw_data_buffer.pop_front();
    lid_header_time_buffer.pop_front();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
    lidar_pushed = false; // sync one whole lidar scan.
    meas.lio_vio_flg = LO; // process lidar topic, so timestamp should be lidar scan end.
    meas.measures.push_back(m);
    return true;
    break;
  }

  default:
  {
    printf("!! WRONG SLAM TYPE !!");
    return false;
  }
  }
  RCLCPP_ERROR(this->get_logger(), "out sync");
  return false;
}

void LIVMapper::publish_frame_world()
{
  if (pcl_w_wait_pub->empty()) return;

  /*** Publish Frame ***/
  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*pcl_w_wait_pub, laserCloudmsg); 
  laserCloudmsg.header.stamp = this->now();
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes->publish(laserCloudmsg);

  /**************** save map ****************/
  // 1. make sure you have enough memories
  // 2. noted that pcd save will influence the real-time performences **/
  double update_time = LidarMeasures.measures.back().lio_time;
  std::stringstream ss_time;
  ss_time << std::fixed << std::setprecision(6) << update_time;

  if (pcd_save_en)
  {
    static int scan_wait_num = 0;

    switch (pcd_save_type)
    {
      case 0: /** world frame **/
        *pcl_wait_save += *pcl_w_wait_pub;
        if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO) scan_wait_num++;
        break;

      case 1: /** body frame **/
        if (LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
        {
          int size = feats_undistort->points.size();
          PointCloudXYZI::Ptr laserCloudBody(new PointCloudXYZI(size, 1));
          for (int i = 0; i < size; i++)
          {
            RGBpointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudBody->points[i]);
          }
          *pcl_wait_save_intensity += *laserCloudBody;
          scan_wait_num++;
          cout << "save body frame points: " << pcl_wait_save_intensity->points.size() << endl;
        }
        pcd_save_interval = 1;
        break;

      default:
        pcd_save_interval = 1;
        scan_wait_num++;
        break;
    }
    if ((pcl_wait_save->size() > 0 || pcl_wait_save_intensity->size() > 0) && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
    {
      string all_points_dir(string(string(ROOT_DIR) + "Log/pcd/") + ss_time.str() + string(".pcd"));

      pcl::PCDWriter pcd_writer;

      cout << "current scan saved to " << all_points_dir << endl;
      if (pcl_wait_save->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
        PointCloudXYZI().swap(*pcl_wait_save);
      }
      if(pcl_wait_save_intensity->points.size() > 0)
      {
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_intensity);
        PointCloudXYZI().swap(*pcl_wait_save_intensity);
      }
      scan_wait_num = 0;
    }
    
    if(LidarMeasures.lio_vio_flg == LIO || LidarMeasures.lio_vio_flg == LO)
    {
      Eigen::Quaterniond q(_state.rot_end);
      fout_lidar_pos << std::fixed << std::setprecision(6);
      fout_lidar_pos <<  LidarMeasures.measures.back().lio_time << " " << _state.pos_end[0] << " " << _state.pos_end[1] << " " << _state.pos_end[2] << " " << q.x() << " " << q.y() << " " << q.z()
          << " " << q.w() << " " << endl;
    }
  }
}


void LIVMapper::publish_effect_world(const std::vector<PointToPlane> &ptpl_list)
{
  int effect_feat_num = ptpl_list.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(effect_feat_num, 1));
  for (int i = 0; i < effect_feat_num; i++)
  {
    laserCloudWorld->points[i].x = ptpl_list[i].point_w_[0];
    laserCloudWorld->points[i].y = ptpl_list[i].point_w_[1];
    laserCloudWorld->points[i].z = ptpl_list[i].point_w_[2];
  }
  sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
  pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
  laserCloudFullRes3.header.stamp = this->now();
  laserCloudFullRes3.header.frame_id = "camera_init";
  pubLaserCloudEffect->publish(laserCloudFullRes3);
}

template <typename T> void LIVMapper::set_posestamp(T &out)
{
  out.position.x = _state.pos_end(0);
  out.position.y = _state.pos_end(1);
  out.position.z = _state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void LIVMapper::publish_odometry()
{
  auto stamp = this->now();

  if (localization_mode_en_) {
    // === Localization mode: publish map->odom and odom->body ===
    odomAftMapped.header.frame_id = map_frame_;
    odomAftMapped.child_frame_id = body_frame_;
    odomAftMapped.header.stamp = stamp;
    set_posestamp(odomAftMapped.pose.pose);

    if (publish_tf_) {
      // 1. map -> odom (localization correction transform)
      geometry_msgs::msg::TransformStamped map_to_odom;
      map_to_odom.header.stamp = stamp;
      map_to_odom.header.frame_id = map_frame_;
      map_to_odom.child_frame_id = odom_frame_;
      // For simplicity, map->odom contains the state estimation result
      // (in a more advanced implementation, this would be the correction factor)
      map_to_odom.transform.translation.x = _state.pos_end(0);
      map_to_odom.transform.translation.y = _state.pos_end(1);
      map_to_odom.transform.translation.z = _state.pos_end(2);
      map_to_odom.transform.rotation.w = geoQuat.w;
      map_to_odom.transform.rotation.x = geoQuat.x;
      map_to_odom.transform.rotation.y = geoQuat.y;
      map_to_odom.transform.rotation.z = geoQuat.z;
      tf_broadcaster_->sendTransform(map_to_odom);

      // 2. odom -> body (identity for now, could be IMU propagate increment)
      geometry_msgs::msg::TransformStamped odom_to_body;
      odom_to_body.header.stamp = stamp;
      odom_to_body.header.frame_id = odom_frame_;
      odom_to_body.child_frame_id = body_frame_;
      odom_to_body.transform.translation.x = 0.0;
      odom_to_body.transform.translation.y = 0.0;
      odom_to_body.transform.translation.z = 0.0;
      odom_to_body.transform.rotation.w = 1.0;
      odom_to_body.transform.rotation.x = 0.0;
      odom_to_body.transform.rotation.y = 0.0;
      odom_to_body.transform.rotation.z = 0.0;
      tf_broadcaster_->sendTransform(odom_to_body);
    }
  } else {
    // === SLAM mode: publish camera_init -> body ===
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = stamp;
    set_posestamp(odomAftMapped.pose.pose);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = "camera_init";
      transform.child_frame_id = "aft_mapped";
      transform.transform.translation.x = _state.pos_end(0);
      transform.transform.translation.y = _state.pos_end(1);
      transform.transform.translation.z = _state.pos_end(2);
      transform.transform.rotation.w = geoQuat.w;
      transform.transform.rotation.x = geoQuat.x;
      transform.transform.rotation.y = geoQuat.y;
      transform.transform.rotation.z = geoQuat.z;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  pubOdomAftMapped->publish(odomAftMapped);
}

void LIVMapper::publish_mavros()
{
  msg_body_pose.header.stamp = this->now();
  msg_body_pose.header.frame_id = "camera_init";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher->publish(msg_body_pose);
}

void LIVMapper::publish_path()
{
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = this->now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath->publish(path);
}