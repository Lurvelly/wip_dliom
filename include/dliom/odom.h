/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dliom/dliom.h"
#include "dliom/gnss.h"

class dliom::OdomNode {

public:

  OdomNode(ros::NodeHandle node_handle);
  ~OdomNode();

  void start();

private:

  struct State;
  struct ImuMeas;
  struct Keyframe;
  struct Pose;
  struct Similarity;

  void getParams();

  void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& pc);
  void callbackImu(const sensor_msgs::Imu::ConstPtr& imu);
  void callbackGNSS(const sensor_msgs::NavSatFix::ConstPtr& gnss);

  void publishPose(const ros::TimerEvent& e);

  void publishToROS(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
  void publishCloud(pcl::PointCloud<PointType>::ConstPtr published_cloud, Eigen::Matrix4f T_cloud);
  void publishKeyframe(const Keyframe kf, ros::Time timestamp);

  void getScanFromROS(const sensor_msgs::PointCloud2ConstPtr& pc);
  void preprocessPoints();
  void deskewPointcloud();
  void initializeInputTarget();
  void setInputSource();

  void initializeDLIOM();

  void getNextPose();
  bool imuMeasFromTimeRange(double start_time, double end_time,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& begin_imu_it,
                            boost::circular_buffer<ImuMeas>::reverse_iterator& end_imu_it);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImu(double start_time, Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                 const std::vector<double>& sorted_timestamps);
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>>
    integrateImuInternal(Eigen::Quaternionf q_init, Eigen::Vector3f p_init, Eigen::Vector3f v_init,
                         const std::vector<double>& sorted_timestamps,
                         boost::circular_buffer<ImuMeas>::reverse_iterator begin_imu_it,
                         boost::circular_buffer<ImuMeas>::reverse_iterator end_imu_it);
  void propagateGICP();

  void propagateState();
  void updateState();
  void addKeyframe();

  void setAdaptiveParams();
  void setKeyframeCloud();

  void computeMetrics();
  void computeSpaciousness();
  void computeDensity();

  sensor_msgs::Imu::Ptr transformImu(const sensor_msgs::Imu::ConstPtr& imu);

  bool newKeyframe();
  void computeConvexHull();
  void computeConcaveHull();
  void pushSubmapIndices(const std::vector<float>& dists, int k, const std::vector<int>& frames);
  void correctPoses();
  void buildSubmap(State vehicle_state);
  void buildJaccardSubmap(State vehicle_state, pcl::PointCloud<PointType>::ConstPtr cloud);
  void updateKeyframesAndGraph();
  void updateKeyframes();
  void buildKeyframesAndSubmap(State vehicle_state);
  void pauseSubmapBuildIfNeeded();
  void updateBackendData();
  void poseGraphOptimization();
  void updateGraphOptimization();

  void debug();

  ros::NodeHandle nh;
  ros::Timer publish_timer;

  // Subscribers
  ros::Subscriber lidar_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber gnss_sub;

  // Publishers
  ros::Publisher odom_pub;
  ros::Publisher pose_pub;
  ros::Publisher path_pub;
  ros::Publisher kf_pose_pub;
  ros::Publisher global_kf_pose_pub;
  ros::Publisher kf_cloud_pub;
  ros::Publisher deskewed_pub;
  ros::Publisher jaccard_pub;
  ros::Publisher jaccard_constraints_pub;
  // ros::Publisher gnss_constraints_pub;
  
  // ROS Msgs
  nav_msgs::Odometry odom_ros;
  geometry_msgs::PoseStamped pose_ros;
  nav_msgs::Path path_ros;
  geometry_msgs::PoseArray kf_pose_ros;
  geometry_msgs::PoseArray global_kf_pose_ros;
  pcl::PointCloud<pcl::PointXYZ> jaccard_ros;
  visualization_msgs::MarkerArray jaccard_constraints_ros;
  visualization_msgs::Marker jaccard_constraints_marker;
  
  // Flags
  std::atomic<bool> dliom_initialized;
  std::atomic<bool> first_valid_scan;
  std::atomic<bool> first_imu_received;
  std::atomic<bool> imu_calibrated;
  std::atomic<bool> submap_hasChanged;
  std::atomic<bool> gicp_hasConverged;
  std::atomic<bool> deskew_status;
  std::atomic<int> deskew_size;

  // Threads
  std::thread publish_thread;
  std::thread publish_keyframe_thread;
  std::thread metrics_thread;
  std::thread debug_thread;
  std::thread graph_thread; 
 
  // Trajectory
  std::vector<std::pair<Eigen::Vector3f, Eigen::Quaternionf>> trajectory;
  double length_traversed;

  // Keyframes
  std::vector<Keyframe> keyframes;
  std::vector<Keyframe> global_keyframes;
  std::vector<ros::Time> keyframe_timestamps;
  std::vector<ros::Time> global_keyframe_timestamps;
  std::vector<std::shared_ptr<const nano_gicp::CovarianceList>> keyframe_normals;
  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> keyframe_transformations;
  int closest_idx;
  std::mutex keyframes_mutex;

  // Factors
  void addOdomFactor();
  void addLoopFactor();
  void addGNSSFactor();
  gtsam::Pose3 state2gtsam(const Pose& p);

  // Sensor Type
  dliom::SensorType sensor;

  // Frames
  std::string odom_frame;
  std::string baselink_frame;
  std::string lidar_frame;
  std::string imu_frame;

  // Preprocessing
  pcl::CropBox<PointType> crop;
  pcl::VoxelGrid<PointType> voxel;

  // Point Clouds
  pcl::PointCloud<PointType>::ConstPtr original_scan;
  pcl::PointCloud<PointType>::ConstPtr deskewed_scan;
  pcl::PointCloud<PointType>::ConstPtr current_scan;

  // Keyframes
  pcl::PointCloud<PointType>::ConstPtr keyframe_cloud;
  int num_processed_keyframes;

  pcl::ConvexHull<PointType> convex_hull;
  pcl::ConcaveHull<PointType> concave_hull;
  std::vector<int> keyframe_convex;
  std::vector<int> keyframe_concave;

  // Submap
  pcl::PointCloud<PointType>::ConstPtr submap_cloud;
  std::shared_ptr<const nano_gicp::CovarianceList> submap_normals;
  std::shared_ptr<const nanoflann::KdTreeFLANN<PointType>> submap_kdtree;

  std::vector<Similarity> submap_kf_curr;
  std::vector<Similarity> submap_kf_prev;
  boost::circular_buffer<std::vector<Similarity>> kf_sim_buffer;
  std::mutex mtx_kf_sim;

  bool new_submap_is_ready;
  std::future<void> submap_future;
  std::condition_variable submap_build_cv;
  bool main_loop_running;
  std::mutex main_loop_running_mutex;

  // Timestamps
  ros::Time scan_header_stamp;
  double scan_stamp;
  double prev_scan_stamp;
  double scan_dt;
  std::vector<double> comp_times;
  std::vector<double> imu_rates;
  std::vector<double> lidar_rates;

  double first_scan_stamp;
  double elapsed_time;

  // GICP
  nano_gicp::NanoGICP<PointType, PointType> gicp;
  nano_gicp::NanoGICP<PointType, PointType> gicp_temp;

  // Transformations
  Eigen::Matrix4f T, T_prior, T_corr;
  Eigen::Quaternionf q_final;

  Eigen::Vector3f origin;

  struct Extrinsics {
    struct SE3 {
      Eigen::Vector3f t;
      Eigen::Matrix3f R;
    };
    SE3 baselink2imu;
    SE3 baselink2lidar;
    Eigen::Matrix4f baselink2imu_T;
    Eigen::Matrix4f baselink2lidar_T;
  } extrinsics;

  // IMU
  ros::Time imu_stamp;
  double first_imu_stamp;
  double prev_imu_stamp;
  double imu_dp, imu_dq_deg;

  struct ImuMeas {
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f ang_vel;
    Eigen::Vector3f lin_accel;
  } imu_meas;

  boost::circular_buffer<ImuMeas> imu_buffer;
  std::mutex mtx_imu;
  std::condition_variable cv_imu_stamp;

  static bool comparatorImu(ImuMeas m1, ImuMeas m2) {
    return (m1.stamp < m2.stamp);
  };

  // GNSS
  double gnss_stamp;
  double prev_gnss_stamp;

  struct GNSSMeas {
      ENU meas;
      double time;
      Eigen::Vector3d cov;
      int kf_id_match = -1;
  } gnss_meas;

  dliom::LocalCartesianGNSS gnss_converter;
  boost::circular_buffer<GNSSMeas> gnss_buffer;
  std::mutex mtx_gnss;
  bool matchGNSSWithKf(GNSSMeas& meas);
  std::atomic<double> avg_gnss_rate;
  std::vector<double> gnss_rates;
  bool gnss_aligned;

  // Geometric Observer
  struct Geo {
    bool first_opt_done;
    std::mutex mtx;
    double dp;
    double dq_deg;
    Eigen::Vector3f prev_p;
    Eigen::Quaternionf prev_q;
    Eigen::Vector3f prev_vel;
  } geo;

  // State Vector
  struct ImuBias {
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
  };

  struct Frames {
    Eigen::Vector3f b;
    Eigen::Vector3f w;
  };

  struct Velocity {
    Frames lin;
    Frames ang;
  };

  struct State {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
    Velocity v;
    ImuBias b; // imu biases in body frame
  } state;

  struct Pose {
    Eigen::Vector3f p; // position in world frame
    Eigen::Quaternionf q; // orientation in world frame
  } lidarPose, imuPose;

  struct Keyframe {
    Pose pose;
    pcl::PointCloud<Point>::ConstPtr cloud;
    pcl::octree::OctreePointCloudSearch<PointType>::Ptr tree;
  };

  struct Similarity {
    int index;
    float similarity;
  };

  // Metrics
  struct Metrics {
    std::vector<float> spaciousness;
    std::vector<float> density;
  } metrics;

  std::string cpu_type;
  std::vector<double> cpu_percents;
  clock_t lastCPU, lastSysCPU, lastUserCPU;
  int numProcessors;

  // GTSAM
  bool is_loop;
  int n_factor;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values estimate;
  gtsam::Values optimized_estimate;
  gtsam::ISAM2 optimizer;

  // Parameters
  std::string version_;
  int num_threads_;

  bool deskew_;

  double gravity_;

  bool adaptive_params_;

  double obs_submap_thresh_;
  double obs_keyframe_thresh_;
  double obs_keyframe_lag_;

  double keyframe_thresh_dist_;
  double keyframe_thresh_rot_;
  double jaccard_corr_thresh_;
  double jaccard_sim_thresh_;

  int submap_knn_;
  int submap_kcv_;
  int submap_kcc_;
  double submap_concave_alpha_;

  bool densemap_filtered_;
  bool wait_until_move_;

  double crop_size_;

  bool vf_use_;
  double vf_res_;

  bool imu_calibrate_;
  bool calibrate_gyro_;
  bool calibrate_accel_;
  bool gravity_align_;
  double imu_calib_time_;
  int imu_buffer_size_;
  Eigen::Matrix3f imu_accel_sm_;

  int gicp_min_num_points_;
  int gicp_k_correspondences_;
  double gicp_max_corr_dist_;
  int gicp_max_iter_;
  double gicp_transformation_ep_;
  double gicp_rotation_ep_;
  double gicp_init_lambda_factor_;

  double geo_Kp_;
  double geo_Kv_;
  double geo_Kq_;
  double geo_Kab_;
  double geo_Kgb_;
  double geo_abias_max_;
  double geo_gbias_max_;

  int alignment_threshold_;
  double thresh_cov_;

};
