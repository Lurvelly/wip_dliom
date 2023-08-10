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

class dliom::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  void start();

private:

  void getParams();

  void publishTimer(const ros::TimerEvent& e);
  void callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe);

  bool savePcd(dliom::save_pcd::Request& req,
               dliom::save_pcd::Response& res);

  ros::NodeHandle nh;
  ros::Timer publish_timer;

  ros::Subscriber keyframe_sub;
  ros::Publisher map_pub;

  ros::ServiceServer save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double publish_freq_;
  double leaf_size_;

};
