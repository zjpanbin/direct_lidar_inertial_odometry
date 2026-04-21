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

#include "dlio/dlio.h"
#include <direct_lidar_inertial_odometry/srv/save_pcd.hpp>

class dlio::MapNode : public rclcpp::Node {

public:

  MapNode(const rclcpp::NodeOptions& options);
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::msg::PointCloud2::SharedPtr keyframe);

  void savePcd(const std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePcd::Request> req,
               std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePcd::Response> res);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_sub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub;

  rclcpp::Service<direct_lidar_inertial_odometry::srv::SavePcd>::SharedPtr save_pcd_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  double leaf_size_;

};
