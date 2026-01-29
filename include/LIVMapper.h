/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "preprocess.h"
#include "voxel_map.h"
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include <pcl/filters/voxel_grid.h>
#include <utils/color.h>
#include <tf2_ros/transform_broadcaster.h>

class LIVMapper : public rclcpp::Node
{
public:
  LIVMapper();
  ~LIVMapper();
  void initializeSubscribersAndPublishers();
  void initializeComponents();
  void initializeFiles();
  void run();
  void gravityAlignment();
  void handleFirstFrame();
  void stateEstimationAndMapping();
  void handleLIO();
  void savePCD();
  void processImu();
  
  bool sync_packages(LidarMeasureGroup &meas);
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  void imu_prop_callback();
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  void pointBodyToWorld(const PointType &pi, PointType &po);
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  void RGBpointBodyLidarToIMU(PointType const *const pi, PointType *const po);
  void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg_in);
  void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in);
  void publish_frame_world();
  void publish_effect_world(const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry();
  void publish_mavros();
  void publish_path();
  void readParameters();
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, std::shared_ptr<VoxelOctoTree>> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_save_type = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  bool new_imu = false, state_update_flg = false, imu_prop_enable = true, ekf_finish_once = false;
  deque<sensor_msgs::msg::Imu> prop_imu_buffer;
  sensor_msgs::msg::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::msg::Odometry imu_prop_odom;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false, gravity_align_finished = false;

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int imu_int_frame = 3;
  int lidar_en = 1;
  bool is_first_frame = false;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_buffer;
  vector<double> extrinT;
  vector<double> extrinR;

  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body;
  PointCloudXYZI::Ptr feats_down_world;
  PointCloudXYZI::Ptr pcl_w_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_pub;
  PointCloudXYZI::Ptr pcl_wait_save;
  PointCloudXYZI::Ptr pcl_wait_save_intensity;
  std::vector<pointWithVar> _pv_list;

  ofstream fout_pre, fout_out, fout_lidar_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;

  V3D euler_cur;

  LidarMeasureGroup LidarMeasures;
  StatesGroup _state;
  StatesGroup  state_propagat;

  nav_msgs::msg::Path path;
  nav_msgs::msg::Odometry odomAftMapped;
  geometry_msgs::msg::Quaternion geoQuat;
  geometry_msgs::msg::PoseStamped msg_body_pose;

  PreprocessPtr p_pre;
  ImuProcessPtr p_imu;
  VoxelMapManagerPtr voxelmap_manager;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr plane_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl;
  rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pubNormal;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDyn;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynRmed;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudDynDbg;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mavros_pose_publisher;
  rclcpp::TimerBase::SharedPtr imu_prop_timer;

  int frame_num = 0;
  double aver_time_consu = 0;
  double aver_time_icp = 0;
  double aver_time_map_inre = 0;
  bool colmap_output_en = false;
};
#endif