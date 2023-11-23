/**
 * @file offboard_control.cpp
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
#include <mavros_msgs/ParamGet.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/ParamValue.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <fstream>
#include <iostream>
#include "common/conversor.hpp"
using namespace conversor;

mavros_msgs::State current_state_;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state_ = *msg;
}

geometry_msgs::PoseStamped current_pose_;
void pose_cb(const geometry_msgs::PoseStamped &msg)
{
  current_pose_.pose = msg.pose;
}

double waypoint_error_;
bool isUAVInWaypoint(const geometry_msgs::Point &p1, const geometry_msgs::Point &p2)
{
  return waypoint_error_ > std::fabs(p1.x - p2.x) + std::fabs(p1.y - p2.y) + std::fabs(p1.z - p2.z);
}

ros::ServiceClient write_param_client_;
mavros_msgs::ParamSet set_param_fcu_;
mavros_msgs::ParamValue default_vel_max_;
void mySigintHandler(int sig)
{
  // Restore default 'MPC_XY_VEL_MAX' param
  /*set_param_fcu_.request.value = default_vel_max_;
  while (!write_param_client_.call(set_param_fcu_) && !set_param_fcu_.response.success)
    ;*/

  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "offboard_ctl_node", ros::init_options::NoSigintHandler);
  //ros::init(argc, argv, "offboard_ctl_node");
  ros::NodeHandle nh;

  signal(SIGINT, mySigintHandler);

  // Get ROS parameters for this node
  geometry_msgs::Point lower_limit, upper_limit;
  parseROSParam(nh, "/flight_setup/limit_x/min", lower_limit.x, -1.5);
  parseROSParam(nh, "/flight_setup/limit_x/max", upper_limit.x, 1.5);
  parseROSParam(nh, "/flight_setup/limit_y/min", lower_limit.y, -1.5);
  parseROSParam(nh, "/flight_setup/limit_y/max", upper_limit.y, 1.5);
  parseROSParam(nh, "/flight_setup/limit_z/min", lower_limit.z, 0.5);
  parseROSParam(nh, "/flight_setup/limit_z/max", upper_limit.z, 2.0);
  double pub_freq;
  parseROSParam(nh, "/flight_setup/offboard_freq", pub_freq, 5.0);
  double takeoff_altitude;
  parseROSParam(nh, "/flight_setup/takeoff_altitude", takeoff_altitude, 1.0);
  parseROSParam(nh, "/flight_setup/waypoint_error", waypoint_error_, 0.5);
  std::string pose_topic;
  parseROSParam<std::string>(nh, "/flight_setup/pose_topic", pose_topic, "/mavros/local_position/pose");

  ROS_INFO("------------------------");
  ROS_INFO("Flight Setup Parameters");
  ROS_INFO("------------------------");
  ROS_INFO("Safe_Min_X:         %.2f (m)", lower_limit.x);
  ROS_INFO("Safe_Max_X:         %.2f (m)", upper_limit.x);
  ROS_INFO("Safe_Min_Y:         %.2f (m)", lower_limit.y);
  ROS_INFO("Safe_Max_Y:         %.2f (m)", upper_limit.y);
  ROS_INFO("Safe_Min_Z:         %.2f (m)", lower_limit.z);
  ROS_INFO("Safe_Max_Z:         %.2f (m)", upper_limit.z);
  ROS_INFO("Takeoff Altitude:   %.2f (m)", takeoff_altitude);
  ROS_INFO("Waypoint error:     %.2f (m)", waypoint_error_);
  ROS_INFO_STREAM("Subscribing to " + pose_topic + " for position information");

  ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);

  ros::Subscriber pose_sub = nh.subscribe(pose_topic, 10, pose_cb);

  // ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);

  ros::Publisher local_control_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

  // Control FCU maximum velocities
  /*ros::ServiceClient read_param_client = nh.serviceClient<mavros_msgs::ParamGet>("mavros/param/get");
  mavros_msgs::ParamGet get_param_fcu;
  get_param_fcu.request.param_id = "MPC_XY_VEL_MAX";
  while (!read_param_client.call(get_param_fcu) && !get_param_fcu.response.success)
    ;
  default_vel_max_ = get_param_fcu.response.value;
  ROS_INFO("------------------------");
  ROS_INFO("Default 'MPC_XY_VEL_MAX' param: %.2f m/s", default_vel_max_.real);

  write_param_client_ = nh.serviceClient<mavros_msgs::ParamSet>("mavros/param/set");
  set_param_fcu_.request.param_id = get_param_fcu.request.param_id;*/

  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(pub_freq);

  std::string sequence_file;
  parseROSParam<std::string>(nh, "/flight_setup/sequence_file", sequence_file, "hover_20sec.txt");
  std::string file = ros::package::getPath("uas_offboard_planner") + "/parameters/" + sequence_file;

  std::ifstream inFile;
  inFile.open(file.c_str());
  if (!inFile)
  {
    ROS_ERROR("Unable to open control sequence file: %s", file.c_str());
    ros::shutdown();
  }

  unsigned int num_commands = 0;
  std::ifstream f(file.c_str());
  std::string line;
  for (unsigned int i = 0; std::getline(f, line); ++i)
  {
    line != "" ? num_commands++ : num_commands;
  }

  ROS_INFO("------------------------");
  ROS_INFO("Number of Commands: %d", num_commands);
  double command[num_commands][7];

  ROS_INFO("Reading in Command Sequences:");
  for (int i = 0; i < num_commands; i++)
  {
    ROS_INFO("-------------------");
    ROS_INFO("Command %d", i + 1);
    std::string cmd_type;
    inFile >> cmd_type;
    if (cmd_type == "pos")
      command[i][0] = 0;
    else if (cmd_type == "vel")
      command[i][0] = 1;
    else if (cmd_type == "dual")
      command[i][0] = 2;
    else
    {
      ROS_ERROR("Format error in sequence file!");
      ROS_ERROR("Ensure to declare 'pos', 'vel', our 'dual' commands.");
      ros::shutdown();
    }

    if (command[i][0] == 0)
    {
      ROS_INFO("Command Type: Position");
      inFile >> command[i][1];
      ROS_INFO("X Position:   %.2f m", command[i][1]);
      inFile >> command[i][2];
      ROS_INFO("Y Position:   %.2f m", command[i][2]);
      inFile >> command[i][3];
      ROS_INFO("Z Position:   %.2f m", command[i][3]);
      inFile >> command[i][4];
      ROS_INFO("Yaw:          %.2f deg", command[i][4]);
      command[i][4] = command[i][4] * M_PI / 180; // Converting Degrees to Radians
      inFile >> command[i][5];
      ROS_INFO("Duration:     %.2f s", command[i][5]);
    }
    else if (command[i][0] == 1)
    {
      ROS_INFO("Command Type: Velocity");
      inFile >> command[i][1];
      ROS_INFO("X Velocity:   %.2f m/s", command[i][1]);
      inFile >> command[i][2];
      ROS_INFO("Y Velocity:   %.2f m/s", command[i][2]);
      inFile >> command[i][3];
      ROS_INFO("Z Position:   %.2f m", command[i][3]);
      inFile >> command[i][4];
      ROS_INFO("Yaw Rate:     %.2f deg/s", command[i][4]);
      command[i][4] = command[i][4] * M_PI / 180; // Converting Degrees to Radians
      inFile >> command[i][5];
      ROS_INFO("Duration:     %.2f s", command[i][5]);
    }
    else
    {
      ROS_INFO("Command Type:     Pos. and Vel.");
      inFile >> command[i][1];
      ROS_INFO("X Position:       %.2f m", command[i][1]);
      inFile >> command[i][2];
      ROS_INFO("Y Position:       %.2f m", command[i][2]);
      inFile >> command[i][3];
      ROS_INFO("Z Position:       %.2f m", command[i][3]);
      inFile >> command[i][4];
      ROS_INFO("Horizontal  Vel.: %.2f m/s", command[i][4]);
      if (command[i][4] <= 0.0)
      {
        ROS_ERROR("Horizontal velocity less or equal to zero.");
        ros::shutdown();
        exit(0);
      }
      inFile >> command[i][5];
      ROS_INFO("Yaw:              %.2f deg", command[i][5]);
      command[i][5] = command[i][5] * M_PI / 180; // Converting Degrees to Radians
      inFile >> command[i][6];
      ROS_INFO("Duration:         %.2f s", command[i][6]);
    }
  }
  ROS_INFO("-------------------");

  // wait for FCU connection
  while (ros::ok() && !current_state_.connected)
  {
    ROS_INFO_ONCE("Connecting to FCU ...");
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("CONNECTED!");

  mavros_msgs::PositionTarget control;
  control.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

  // More information on type_mask can be found at https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
  control.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                       mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                       mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                       mavros_msgs::PositionTarget::IGNORE_YAW | mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

  control.position.z = takeoff_altitude;
  control.position.x = current_pose_.pose.position.x;
  control.position.y = current_pose_.pose.position.y;
  // control.position.x = 0;
  // control.position.y = 0;
  // send a few setpoints before starting
  for (int i = 100; ros::ok() && i > 0; --i)
  {
    control.header.stamp = ros::Time::now();
    local_control_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::SetMode auto_land;
  auto_land.request.custom_mode = "AUTO.LAND";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  // mavros_msgs::CommandTOL land_cmd;
  // land_cmd.request.value = true;

  ros::Time last_request = ros::Time::now();

  while (ros::ok() && !current_state_.armed)
  {
    ROS_INFO_ONCE("Requesting OFFBOARD mode ... ");
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)))
    {
      ROS_INFO_ONCE("------------------------------------");
      ROS_INFO_ONCE("You can now switch to OFFBOARD mode.");
      last_request = ros::Time::now();
    }
    else
    {
      if (current_state_.mode == "OFFBOARD")
      {
        ROS_WARN_ONCE("********************");
        ROS_WARN_ONCE("**  Switched to   **");
        ROS_WARN_ONCE("** OFFBOARD mode. **");
        ROS_WARN_ONCE("********************");

        if (!current_state_.armed && (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
          ROS_INFO_ONCE("Arming UAV ... ");
          if (arming_client.call(arm_cmd) && arm_cmd.response.success)
          {
            ROS_INFO_ONCE("UAV armed!");
          }
          last_request = ros::Time::now();
        }
      }
    }
    control.position.x = current_pose_.pose.position.x;
    control.position.y = current_pose_.pose.position.y;
    control.header.stamp = ros::Time::now();
    local_control_pub.publish(control);

    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::Point takeoff_setpoint;
  takeoff_setpoint = control.position;
  takeoff_setpoint.z = takeoff_altitude;
  while (ros::ok() && current_state_.mode == "OFFBOARD" &&
         !isUAVInWaypoint(current_pose_.pose.position, takeoff_setpoint))
  {
    ROS_INFO_ONCE("Waiting until UAV reaches takeoff setpoint ... ");
    control.header.stamp = ros::Time::now();
    local_control_pub.publish(control);

    ros::spinOnce();
    rate.sleep();
  }

  unsigned int command_count = 0;
  int key = 0;
  bool reached_waypoint = false, show_wp_msg = false, performed_action = false;
  double duration_waypoint = 0.0;

  while (ros::ok() && current_state_.armed && current_state_.mode == "OFFBOARD" && command_count < num_commands)
  {
    if (command[command_count][0] == 0 && !reached_waypoint)
    {
      ROS_INFO_COND(!show_wp_msg, "Reaching next waypoint ...");
      show_wp_msg = true;

      // More information on type_mask can be found at
      // https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
      control.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE);
      control.position.x = command[command_count][1];
      control.position.y = command[command_count][2];
      control.position.z = command[command_count][3];
      control.yaw = command[command_count][4];
      duration_waypoint = command[command_count][5];
      reached_waypoint = isUAVInWaypoint(control.position, current_pose_.pose.position);
      set_param_fcu_.request.value = default_vel_max_;
    }
    else if (command[command_count][0] == 1)
    {
      // More information on type_mask can be found at
      // https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
      control.type_mask = (mavros_msgs::PositionTarget::IGNORE_PX | mavros_msgs::PositionTarget::IGNORE_PY |
                           mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY |
                           mavros_msgs::PositionTarget::IGNORE_AFZ | mavros_msgs::PositionTarget::IGNORE_YAW);
      control.velocity.x = command[command_count][1];
      control.velocity.y = command[command_count][2];
      control.position.z = command[command_count][3];
      control.yaw_rate = command[command_count][4];
      duration_waypoint = command[command_count][5];
      set_param_fcu_.request.value = default_vel_max_;
    }
    else if (command[command_count][0] == 2)
    {
      // More information on type_mask can be found at
      // https://mavlink.io/en/messages/common.html#POSITION_TARGET_TYPEMASK
      control.type_mask = (mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                           mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                           mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                           mavros_msgs::PositionTarget::IGNORE_YAW_RATE);
      control.position.x = command[command_count][1];
      control.position.y = command[command_count][2];
      control.position.z = command[command_count][3];
      set_param_fcu_.request.value.real = command[command_count][4];
      control.yaw = command[command_count][5];
      duration_waypoint = command[command_count][6];
    }
    //while (!write_param_client_.call(set_param_fcu_) && !set_param_fcu_.response.success)
    //  ;
    if (command[command_count][0] != 0 || reached_waypoint)
    {
      if (!performed_action)
      {
        ROS_INFO("Performing action %.0d for %.2f seconds.", command_count + 1, duration_waypoint);
        performed_action = true;
        last_request = ros::Time::now();
      }

      if ((ros::Time::now() - last_request) > ros::Duration(duration_waypoint))
      {
        command_count++;
        reached_waypoint = false, show_wp_msg = false, performed_action = false;
      }
    }

    if (current_pose_.pose.position.x < lower_limit.x || current_pose_.pose.position.x > upper_limit.x ||
        current_pose_.pose.position.y < lower_limit.y || current_pose_.pose.position.y > upper_limit.y ||
        current_pose_.pose.position.z < lower_limit.z || current_pose_.pose.position.z > upper_limit.z)
    {
      ROS_WARN("EXITED SAFE AREA, LANDING UAV!");
      break;
    }
    else
    {
      control.header.stamp = ros::Time::now();
      local_control_pub.publish(control);
    }

    ros::spinOnce();
    rate.sleep();
  }

  // Restore default 'MPC_XY_VEL_MAX' param
  set_param_fcu_.request.value = default_vel_max_;
  //while (!write_param_client_.call(set_param_fcu_) && !set_param_fcu_.response.success)
  //  ;

  offb_set_mode.request.custom_mode = "AUTO.LAND";
  ROS_INFO("Land Attempt");
  while (ros::ok() && current_state_.mode == "OFFBOARD")
  {
    if (current_state_.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
    {
      if (set_mode_client.call(auto_land) && auto_land.response.mode_sent)
      {
        ROS_INFO("Autoland enabled!");
      }
      last_request = ros::Time::now();
    }

    control.header.stamp = ros::Time::now();
    local_control_pub.publish(control);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
