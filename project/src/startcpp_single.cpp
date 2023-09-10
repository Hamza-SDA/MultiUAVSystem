#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

// Creating subscribers, publishers and variables
ros::Subscriber state_sub;
ros::Publisher local_pos_pub;
ros::Publisher position_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber posexyz;

double current_x;
double current_y;
double current_z;
int mission_progress = 0;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x = msg->pose.position.x;	// Forward / Backward
    current_y = msg->pose.position.y;	// Left / Right
    current_z = msg->pose.position.z;	// Altitude
}

int main(int argc, char **argv)
{
    ROS_INFO("Initialization");
    ros::init(argc, argv, "startcpp_single");
    ros::NodeHandle nh;
    
    // Establishing publishers and subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    posexyz = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, poseCallBack);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 20Hz
    ros::Rate rate(20.0);
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU Connected");
    
    // Setting altitude position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    // Setting to "OFFBOARD" flight mode
    mavros_msgs::SetMode offb_mode;
    offb_mode.request.base_mode = 0;
    offb_mode.request.custom_mode = "OFFBOARD";
    
    // Arming drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    
    // Enabling Manual mode selection
    while(ros::ok() && !current_state.armed)
    {
    	ROS_INFO("Requesting OFFBOARD mode");
    	if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)) )
    	{
    		ROS_INFO("You can now switch to OFFBOARD mode.");
    		last_request = ros::Time::now();
    	}
    	else
    	{
    		if( current_state.mode == "OFFBOARD" )
    		{
    			ROS_INFO("Switched to OFFBOARD mode");

    			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(10.0)) )
    			{
    				ROS_INFO("Arming UAV");
    				if( arming_client.call(arm_cmd) && arm_cmd.response.success )
    				{
    					ROS_INFO("UAV Armed");
    				}
    				last_request = ros::Time::now();
    			}
    		}
    	}

    	ros::spinOnce();
    	rate.sleep();
    }

    // OFFBOARD Flight Mission
    while(ros::ok() && current_state.armed && current_state.mode == "OFFBOARD")
    {
        // Setting mode to OFFBOARD if not already set
        //if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( set_mode_client.call(offb_mode) && offb_mode.response.mode_sent)
        //    {                ROS_INFO("Offboard enabled");            }
        //    last_request = ros::Time::now();
        //}

        // Arming drone if not already armed
        //if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        //    {                    ROS_INFO("Vehicle armed");                }
        //    mission_progress++;
        //    ROS_INFO("Position Home");
        //    last_request = ros::Time::now();
        //}

        // Moving the drone based on altitude
        if( mission_progress == 1 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(10.0)))
        {
            pose.pose.position.x = 2;
            pose.pose.position.y = 2;
            mission_progress++;
            ROS_INFO("Position 1");
        }
        if( mission_progress == 2 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(15.0)))
        {
            pose.pose.position.x = 2;
            pose.pose.position.y = -2;
            mission_progress++;
            ROS_INFO("Position 2");
        }
        if( mission_progress == 3 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(20.0)))
        {
            pose.pose.position.x = -2;
            pose.pose.position.y = -2;
            mission_progress++;
            ROS_INFO("Position 3");
        }
        if( mission_progress == 4 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(25.0)))
        {
            pose.pose.position.x = -2;
            pose.pose.position.y = 2;
            mission_progress++;
            ROS_INFO("Position 4");
        }
        if( mission_progress == 5 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(30.0)))
        {
            pose.pose.position.x = 0;
            pose.pose.position.y = 0;
            mission_progress++;
            ROS_INFO("Return to Home");
        }

        // Landing
        if( mission_progress == 6 && current_z < 2 && (ros::Time::now() - last_request > ros::Duration(35.0)))
        {
            pose.pose.position.z = 0.5;
            mission_progress++;
            ROS_INFO("Landing");
            break;
        }

 

        local_pos_pub.publish(pose);
        ROS_INFO("x = %.2f, y = %.2f, z = %.2f", current_x, current_y, current_z);

        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
