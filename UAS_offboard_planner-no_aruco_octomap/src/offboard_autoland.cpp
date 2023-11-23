/**
 * @file offboard_autoland.cpp
 * @brief
 * @version 0.3.0
 * @date 2021-04-16
 *
 * @copyright Copyright (c) 2021 QUT.
 *
 */
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "common/conversor.hpp"
using namespace conversor;

mavros_msgs::State current_state_;
void state_cb(const mavros_msgs::State &msg)
{
  current_state_ = msg;
}

geometry_msgs::PoseStamped current_pose_;
void pose_cb(const geometry_msgs::PoseStamped &msg)
{
  current_pose_ = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offb_node");
  ros::NodeHandle nh;

  // Local UAV pose topic to subscribe.
  std::string pose_topic;
  parseROSParam<std::string>(nh, "/flight_setup/pose_topic", pose_topic, "/mavros/local_position/pose");
  double pub_freq;
  parseROSParam(nh, "/flight_setup/offboard_freq", pub_freq, 5.0);

  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);

  // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10, pose_cb);

  ros::Subscriber pose_sub = nh.subscribe(pose_topic, 10, pose_cb);

  ros::Publisher local_control_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  // ros::Subscriber velocity_sub = nh.subscribe<interface_pkg::ControlVelocities>("interface/control", 10, control_cb);
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(pub_freq);

  // wait for FCU connection
  while (ros::ok() && !current_state_.connected)
  {
    ROS_INFO("Attempting to connect to the FCU ...");
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("CONNECTED!");

  mavros_msgs::SetMode auto_land;
  auto_land.request.custom_mode = "AUTO.LAND";

  // mavros_msgs::CommandTOL land_cmd;
  // land_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  ROS_INFO("PRESS ENTER TO LAND AT ANY TIME");

  std::cin.ignore();

  ROS_INFO("Land Attempt");
  while (ros::ok())
  {
    if (current_state_.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(auto_land) && auto_land.response.mode_sent)
      {
        ROS_INFO("Autoland enabled");
      }
      last_request = ros::Time::now();
    }

    // control.position.z = current_model_state.pose[length-1].position.z;
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
