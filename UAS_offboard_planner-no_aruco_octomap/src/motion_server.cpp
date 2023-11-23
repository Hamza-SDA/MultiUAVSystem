/**
 * @file motion_server.cpp
 * @brief 
 * @version 0.3.0
 * @date 2021-04-16
 * 
 * @copyright Copyright (c) 2021 QUT.
 * 
 */
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include "uas_offboard_planner/motion_module.h"
#include "common/conversor.hpp"
using namespace conversor;

std::unique_ptr<octomap::OcTree> octree_; // using octomap for map representation and querying.
bool isLocationOccupied(const geometry_msgs::Point &objectPos)
{
  double z_value;
  objectPos.z < 0.3 ? z_value = 0.0 : z_value = objectPos.z; // Discard noisy occupancy nodes from the ground.
  std::unique_ptr<octomap::OcTreeNode> node_res(octree_->search(objectPos.x, objectPos.y, z_value, 16));
  if (node_res)
  {
    return octree_->isNodeOccupied(node_res.get());
  }
  else
  {
    return false; // if NULL, the space is unknown. Assuming unknown cells as free cells.
  }
}

mavros_msgs::State current_state_;
void stateCallback(const mavros_msgs::State &msg)
{
  current_state_ = msg;
}

// UAV pose command to be sent through MAVROS.
geometry_msgs::PoseStamped pose_cmd_;
// Received UAV pose setpoint from POMDP solver.
geometry_msgs::PoseStamped setpoint_pose_;
// Received UAV pose setpoint from POMDP solver.
geometry_msgs::PoseStamped prev_set_pose_;
// UAV pose message prior to takeoff.
geometry_msgs::PoseStamped takeoff_pose_;
// Flag whether the UAV has reached the takeoff position waypoint.
bool reached_takeoff_wp_;
bool start_server_ = false;
// Position tolerance (in metres) between actual UAV position and next position waypoint.
double waypoint_error_;
// Lower and Upper boundaries in metres (local position).
double min_x_ = 0.0, max_x_ = 0.0, min_y_ = 0.0, max_y_ = 0.0, min_z_ = 0.0, max_z_ = 0.0;

bool serverCallback(uas_offboard_planner::motion_module::Request &req, uas_offboard_planner::motion_module::Response &res)
{
  geometry_msgs::Point robotPos;
  robotPos.x = req.x_pos, robotPos.y = req.y_pos, robotPos.z = req.z_pos;
  if (isLocationOccupied(robotPos))
  {
    ROS_WARN("Position command [%.2f, %.2f, %.2f] will crash the UAV!", robotPos.x, robotPos.y, robotPos.z);
    robotPos = prev_set_pose_.pose.position; // This will send the previous UAV position (hold).
    ROS_WARN("Hovering UAV instead ([%.2f, %.2f, %.2f]).", robotPos.x, robotPos.y, robotPos.z);
  }
  else
  {
    if (robotPos.x < min_x_)
    {
      ROS_WARN("Position in X is %.2f. Sending %.2f instead.", robotPos.x, min_x_);
      robotPos.x = min_x_;
    }
    else if (robotPos.x > max_x_)
    {
      ROS_WARN("Position in X is %.2f. Sending %.2f instead.", robotPos.x, max_x_);
      robotPos.x = max_x_;
    }
    if (robotPos.y < min_y_)
    {
      ROS_WARN("Position in Y is %.2f. Sending %.2f instead.", robotPos.y, min_y_);
      robotPos.y = min_y_;
    }
    else if (robotPos.y > max_y_)
    {
      ROS_WARN("Position in Y is %.2f. Sending %.2f instead.", robotPos.y, max_y_);
      robotPos.y = max_y_;
    }
    if (robotPos.z < min_z_)
    {
      ROS_WARN("Position in Z is %.2f. Sending %.2f instead.", robotPos.z, min_z_);
      robotPos.z = min_z_;
    }
    else if (robotPos.z > max_z_)
    {
      ROS_WARN("Position in Z is %.2f. Sending %.2f instead.", robotPos.z, max_z_);
      robotPos.z = max_z_;
    }
  }
  prev_set_pose_.pose = setpoint_pose_.pose; // Update previous values.

  setpoint_pose_.header.stamp = ros::Time::now();
  setpoint_pose_.pose.position = robotPos;
  tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, req.z_rot);
  quaternionTFToMsg(q, pose_cmd_.pose.orientation);
  ROS_DEBUG("UAV Pos. = [%.2f, %.2f, %.2f], Yaw = %.0f deg.", setpoint_pose_.pose.position.x,
            setpoint_pose_.pose.position.y, setpoint_pose_.pose.position.z, req.z_rot * 180 / M_PI);
  res.started = start_server_;
  return true;
}

void octomapCallback(const octomap_msgs::Octomap &msg)
{
  std::unique_ptr<octomap::AbstractOcTree> absTree(octomap_msgs::msgToMap(msg));
  octree_.reset(dynamic_cast<octomap::OcTree *>(absTree.get()));
}

// Sets the current position and checks if the current goal has been reached
void robotPoseCallback(const geometry_msgs::PoseStamped &msg)
{
  if (!current_state_.armed)
  {
    takeoff_pose_.header.stamp = ros::Time::now();
    takeoff_pose_.pose.position.x = msg.pose.position.x;
    takeoff_pose_.pose.position.y = msg.pose.position.y;
    takeoff_pose_.pose.orientation = msg.pose.orientation;
  }

  geometry_msgs::Point p1 = takeoff_pose_.pose.position;
  geometry_msgs::Point p2 = msg.pose.position;
  reached_takeoff_wp_ = waypoint_error_ > (std::fabs(p1.x - p2.x) + std::fabs(p1.y - p2.y) + std::fabs(p1.z - p2.z));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_server_node");
  ros::NodeHandle nh;

  // Get ROS parameters for this node
  parseROSParam(nh, "/flight_setup/limit_x/min", min_x_, -1.5);
  parseROSParam(nh, "/flight_setup/limit_x/max", max_x_, 1.5);
  parseROSParam(nh, "/flight_setup/limit_y/min", min_y_, -1.5);
  parseROSParam(nh, "/flight_setup/limit_y/max", max_y_, 1.5);
  parseROSParam(nh, "/flight_setup/limit_z/min", min_z_, 0.0);
  parseROSParam(nh, "/flight_setup/limit_z/max", max_z_, 2.5);
  parseROSParam(nh, "/flight_setup/takeoff_altitude", takeoff_pose_.pose.position.z, 2.0);
  parseROSParam(nh, "/flight_setup/waypoint_error", waypoint_error_, 0.2);
  // Publishing Frequency in Hz.
  double offboard_freq = 0.0;
  // Local UAV pose topic to subscribe.
  std::string pose_topic;
  parseROSParam(nh, "/flight_setup/offboard_freq", offboard_freq, 5.0);
  parseROSParam<std::string>(nh, "/flight_setup/pose_topic", pose_topic, "/mavros/local_position/pose");

  ROS_INFO("*************************************");
  ROS_INFO("Min. Limit in X:    %.2f\t (m)", min_x_);
  ROS_INFO("Max. Limit in X:    %.2f\t (m)", max_x_);
  ROS_INFO("Min. Limit in Y:    %.2f\t (m)", min_y_);
  ROS_INFO("Max. Limit in Y:    %.2f\t (m)", max_y_);
  ROS_INFO("Min. Limit in Z:    %.2f\t (m)", min_z_);
  ROS_INFO("Max. Limit in Z:    %.2f\t (m)", max_z_);
  ROS_INFO("Takeoff altitude:   %.2f\t (m)", takeoff_pose_.pose.position.z);
  ROS_INFO("Waypoint tolerance: %.2f\t (m)", waypoint_error_);
  ROS_INFO("Offboard frequency: %.2f\t (Hz)", offboard_freq);
  ROS_INFO("Pose topic:         %s", pose_topic.c_str());
  ROS_INFO("*************************************");

  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, stateCallback);
  // Subscribe to MAVROS local position to estimate take-off coordinates (No GPS)
  ros::Subscriber local_robot_pos_sub = nh.subscribe(pose_topic, 1, &robotPoseCallback);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  ros::ServiceServer service = nh.advertiseService("motion_service", serverCallback);

  // Subscribe to octomap to ensure that robot will not crash with an obstacle
  ros::Subscriber octomap_full_sub_ = nh.subscribe("/octomap_full", 1, octomapCallback);

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(offboard_freq);

  // wait for FCU connection
  while (ros::ok() && !current_state_.connected)
  {
    ros::spinOnce();
    rate.sleep();
    ROS_INFO_ONCE("Connecting to FCU ... ");
  }

  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    local_pos_pub.publish(takeoff_pose_);
    ros::spinOnce();
    rate.sleep();
  }

  // The initial MAVROS command is takeoff waypoint.
  pose_cmd_.pose = takeoff_pose_.pose;

  // Request change to Offboard mode and once sucessful, arm the UAV.
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();
  while (ros::ok() && !current_state_.armed)
  {
    ROS_INFO_ONCE("Requesting OFFBOARD mode ... ");
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
    {
      ROS_INFO_ONCE("Please switch manually to OFFBOARD mode.");
      last_request = ros::Time::now();
    }
    else
    {
      if (current_state_.mode == "OFFBOARD")
      {
        ROS_WARN_ONCE("***********************************");
        ROS_WARN_ONCE("**  You have manually switched   **");
        ROS_WARN_ONCE("** to OFFBOARD mode. Be careful! **");
        ROS_WARN_ONCE("***********************************");

        if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
          ROS_INFO_ONCE("Arming UAV ... ");
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO_ONCE("UAV armed!");
            start_server_ = true;
          }
          last_request = ros::Time::now();
        }
      }
    }
    pose_cmd_.header.stamp = ros::Time::now();
    local_pos_pub.publish(pose_cmd_);
    ros::spinOnce();
    rate.sleep();
  }

  while (ros::ok() && current_state_.armed && !reached_takeoff_wp_ && current_state_.mode == "OFFBOARD")
  {
    ROS_INFO_ONCE("UAV is reaching takeoff coordinates!");
    local_pos_pub.publish(takeoff_pose_);
    ros::spinOnce();
    rate.sleep();
  }

  // Subscribing to /mavros/local_position/pose is no longer necessary
  local_robot_pos_sub.shutdown();

  pose_cmd_.pose.position.x = 0;
  pose_cmd_.pose.position.y = 0;

  while (ros::ok() && current_state_.armed && current_state_.mode == "OFFBOARD")
  {
    pose_cmd_.header.stamp = ros::Time::now();
    pose_cmd_.pose = setpoint_pose_.pose;
    local_pos_pub.publish(pose_cmd_);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
