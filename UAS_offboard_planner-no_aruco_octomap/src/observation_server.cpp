/**
 * @file observation_server.cpp
 * @brief 
 * @version 0.3.0
 * @date 2021-04-16
 * 
 * @copyright Copyright (c) 2021 QUT.
 * 
 */
#include <aruco_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/package.h> // For finding package path
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <memory>
#include <vector>
#include "uas_offboard_planner/observation_module.h"
#include "common/conversor.hpp"
using namespace conversor;

double uav_pos_x_ = 0.0, uav_pos_y_ = 0.0, uav_pos_z_ = 0.0; // world frame take off and hover position
bool is_unknown_ = false, detected_obs_ = false;

double target_coord_x_ = 0.0, target_coord_y_ = 0.0, target_coord_z_ = 0.0;
double target_heading_ = 0.0;
bool is_target_detected_ = false;
double num_detections_ = 0.0, detection_rate_ = 0.0; // variables to handle target detection rate.

long num_octomap_msg_ = 0;

std::unique_ptr<octomap::OcTree> obs_octree_;
// std::string octomap_filename_ = "";

// predicted position
geometry_msgs::Pose uav_pose_;

std::vector<aruco_msgs::Marker, std::allocator<aruco_msgs::Marker>> markers_;
double confidence_ = 0.0;

// Sets the current position and checks if the current goal has been reached
void positionCallback(const geometry_msgs::PoseStamped &msg)
{
  uav_pose_ = msg.pose;
  uav_pos_x_ = msg.pose.position.x;
  uav_pos_y_ = msg.pose.position.y;
  uav_pos_z_ = msg.pose.position.z;
}

// Check if the current path is blocked
void octomapFullCallback(const octomap_msgs::Octomap &msg)
{
  if (num_octomap_msg_++ % 10 > 0)
  {
    return; // We get too many of those messages. Only process 1/10 of them
  }
  octomap::AbstractOcTree *absTree = octomap_msgs::msgToMap(msg);
  obs_octree_.reset(dynamic_cast<octomap::OcTree *>(absTree));

  // octomap::point3d origin(uav_pos_x_, uav_pos_y_, uav_pos_z_);
  // geometry_msgs::Quaternion q = uav_pose_.orientation;
  // octomath::Quaternion qo(q.w, q.x, q.y, q.z);
  // octomap::point3d direction(1.0, 0.0, 0.0);              // search in the x direction
  // octomap::point3d direction_rot = qo.rotate(direction);  // rotate to the uav orientation
  // octomap::point3d end;
  // // will need to be transformed to world coordinates
  // detected = obs_octree_->castRay(origin, direction_rot, end, true, 5.0);  // assuming unknown cells as free cells
  // if (!detected)
  // {  // if not detected an obstacle check if the node is unknown
  //   octomap::OcTreeNode *node_res = obs_octree_->search(end);
  //   if (!node_res)
  //   {
  //     ROS_WARN("Unknown space in front of UAV");
  //   }
  // }
  // else
  // {
  //   ROS_WARN("Detected obstacle");
  // }
}

// TODO: revise camera tf and outputs from aruco_ros.
void arucoBoardPoseCallback(const aruco_msgs::MarkerArray &msg)
{
  markers_ = msg.markers;

  target_coord_x_ = 0.0, target_coord_y_ = 0.0, target_coord_z_ = 0.0, target_heading_ = 0.0;
  confidence_ = 0.0;
  int numMarkers = markers_.size();
  if (numMarkers > 0)
  {
    // is_target_detected_ = true;
    num_detections_++; // increment the instances of positive detections.
    for (int i = 0; i < numMarkers; i++)
    {
      target_coord_x_ += markers_[i].pose.pose.position.x;
      target_coord_y_ += markers_[i].pose.pose.position.y;
      target_coord_z_ += markers_[i].pose.pose.position.z;
      target_heading_ += asin(markers_[i].pose.pose.orientation.z / 2);
      confidence_ += markers_[i].confidence;
    }
    target_coord_x_ /= numMarkers;
    target_coord_y_ /= numMarkers;
    target_coord_z_ /= numMarkers;
    target_heading_ /= numMarkers;
    confidence_ /= numMarkers;
  }
  // else
  // {
  //   is_target_detected_ = false;
  // }
  detection_rate_++; // increment the number of read data.
  // TODO: revise camera tf and outputs from aruco_ros. Heading missing for full transformation.
  target_coord_x_ += uav_pos_x_;
  // TODO: revise camera tf and outputs from aruco_ros. Heading missing for full transformation.
  target_coord_y_ += uav_pos_y_;
}

//  TODO: update target heading to match UAV's only if the target is found.
bool observe_CB(uas_offboard_planner::observation_module::Request &req, uas_offboard_planner::observation_module::Response &res)
{
  octomap::point3d origin(uav_pos_x_, uav_pos_y_, uav_pos_z_);
  geometry_msgs::Quaternion q = uav_pose_.orientation;
  octomath::Quaternion qo(q.w, q.x, q.y, q.z);
  octomap::point3d direction(1.0, 0.0, 0.0);             // search in the x direction
  octomap::point3d direction_rot = qo.rotate(direction); // rotate to the uav orientation
  octomap::point3d end;
  // will need to be transformed to world coordinates
  detected_obs_ = obs_octree_->castRay(origin, direction_rot, end, true, 5.0); // assuming unknown cells as free cells
  if (!detected_obs_)
  { // if not detected an obstacle check if the node is unknown
    octomap::OcTreeNode *node_res = obs_octree_->search(end);
    if (node_res == nullptr)
    {
      ROS_WARN("Unknown space in front of UAV");
    }
    else
    {
      ROS_WARN("Space in front is free");
    }
  }
  else
  {
    ROS_WARN("Detected obstacle");
  }
  // check uav position from mavros
  res.uav_x = uav_pos_x_;
  res.uav_y = uav_pos_y_;
  res.uav_z = uav_pos_z_;
  // check if it sees obstacle in front
  res.obstacle = detected_obs_;
  if ((float)(num_detections_ / detection_rate_) > 0.4)
  {
    res.targetFound = true;
  }
  else
  {
    res.targetFound = false;
  }
  num_detections_ = 0.0;
  detection_rate_ = 0.0; // reset counters
  // res.targetFound = is_target_detected_;
  res.targetX = target_coord_x_;
  res.targetY = target_coord_y_;
  res.targetZ = target_coord_z_;
  res.targetHeading = target_heading_;
  // obs_octree_->writeBinary(octomap_filename_);
  return true;
}

void targetFlagCallback(const std_msgs::Bool &msg)
{
  is_target_detected_ = msg.data;
  if (msg.data == true)
  {
    num_detections_++;
  }
  detection_rate_++;
}

void targetCoordsCallback(const geometry_msgs::PoseStamped &msg)
{
  target_coord_x_ = msg.pose.position.x;
  target_coord_y_ = msg.pose.position.y;
  target_coord_z_ = msg.pose.position.z;
}

void targetHeadingCallback(const std_msgs::Float32 &msg)
{
  target_heading_ = msg.data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "obs_server_node");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("observation_service", observe_CB);

  // Get ROS parameters for this node

  // Local UAV pose topic to subscribe.
  std::string pose_topic;
  parseROSParam<std::string>(n, "/flight_setup/pose_topic", pose_topic, "/mavros/local_position/pose");

  // Subscribers
  ros::Subscriber octomap_full_sub = n.subscribe("/octomap_full", 1, octomapFullCallback);
  ros::Subscriber ground_truth_sub = n.subscribe("/mavros/local_position/pose", 1, positionCallback);

  ros::Subscriber target_flag_sub = n.subscribe("/vision/target/isDetected", 1, targetFlagCallback);
  ros::Subscriber target_coords_sub = n.subscribe("/vision/target/coords", 1, targetCoordsCallback);
  ros::Subscriber target_heading_sub = n.subscribe("/vision/target/heading", 1, targetHeadingCallback);

  ros::Subscriber aruco_board_pose_sub = n.subscribe("/aruco_marker_publisher/markers", 1, arucoBoardPoseCallback);

  ROS_INFO("Ready to generate Observations from %s", pose_topic.c_str());
  ros::spin();
  return 0;
}
