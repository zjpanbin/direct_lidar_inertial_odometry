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

#include "dlio/map.h"

#include <filesystem>

dlio::MapNode::MapNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("dlio_map", options) {

  this->getParams();

  this->keyframe_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "keyframes", 10,
    std::bind(&dlio::MapNode::callbackKeyframe, this, std::placeholders::_1));

  this->map_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("map", 100);

  this->save_pcd_srv = this->create_service<direct_lidar_inertial_odometry::srv::SavePcd>(
    "save_pcd",
    std::bind(&dlio::MapNode::savePcd, this,
      std::placeholders::_1, std::placeholders::_2));

  this->dlio_map = pcl::PointCloud<PointType>::Ptr (std::make_shared<pcl::PointCloud<PointType>>());

  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams() {

  this->declare_parameter<std::string>("dlio.odom.odom_frame", "odom");
  this->odom_frame = this->get_parameter("dlio.odom.odom_frame").as_string();

  this->declare_parameter<double>("dlio.map.sparse.leafSize", 0.5);
  this->leaf_size_ = this->get_parameter("dlio.map.sparse.leafSize").as_double();

  // Get Node NS and Remove Leading Character
  std::string ns = this->get_namespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;

}

void dlio::MapNode::start() {
}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::msg::PointCloud2::SharedPtr keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (std::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->dlio_map += *keyframe_pcl;

  // publish full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::msg::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);
    map_ros.header.stamp = this->now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub->publish(map_ros);
  }

}

void dlio::MapNode::savePcd(
  const std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePcd::Request> req,
  std::shared_ptr<direct_lidar_inertial_odometry::srv::SavePcd::Response> res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (std::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

  float leaf_size = req->leaf_size;
  std::string p = req->save_path;

  if (!std::filesystem::is_directory(p)) {
    RCLCPP_INFO(this->get_logger(), "Could not find directory %s", p.c_str());
    res->success = false;
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Saving map to %s/dlio_map.pcd with leaf size %s... ",
    p.c_str(), to_string_with_precision(leaf_size, 2).c_str());

  // voxelize map
  pcl::VoxelGrid<PointType> vg;
  vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  vg.setInputCloud(m);
  vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res->success = ret == 0;

  if (res->success) {
    RCLCPP_INFO(this->get_logger(), "done");
  } else {
    RCLCPP_INFO(this->get_logger(), "failed");
  }

}
