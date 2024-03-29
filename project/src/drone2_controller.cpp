#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <project/Drone_pos.h>



// Creating subscribers, publishers and variables
ros::Subscriber state_sub;
ros::Subscriber posexyz;
ros::Subscriber alt_change;
ros::Subscriber lead_pos;

ros::Publisher local_pos_pub;
ros::Publisher position_pub;
ros::Publisher global_pos_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

double current_x, current_y, current_z;
double init_x, init_y, init_z;
double rel_x, rel_y, rel_z;
double lead_x, lead_y;
double drone_change;
int mission_progress = 0;
bool started;



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void poseCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x = msg->pose.position.x;	// Local x Position (Forward / Backward)
    current_y = msg->pose.position.y;	// Local y Position (Left / Right)
    current_z = msg->pose.position.z;	// Local z Position (Altitude)
    
    rel_x = current_x + init_x;	// Global x Position (Forward / Backward)
    rel_y = current_y + init_y;	// Global y position (Left / Right)
    rel_z = current_z + init_z;	// Global z position (Altitude)
    
    // ROS_INFO("x = %.2f, y = %.2f, z = %.2f", rel_x, rel_y, rel_z);
}

void alt_change_cb(const project::Drone_pos::ConstPtr& msg) {
    drone_change = msg->d2_change;
}

void lead_pos_cb(const project::Drone_pos::ConstPtr& msg) {
    lead_x = msg->d1_x;
    lead_y = msg->d1_y;
}



int main(int argc, char **argv)
{
    ROS_INFO("Initialization");
    ros::init(argc, argv, "drone2_controller");
    ros::NodeHandle nh;
    started = false;
    
    // Retrieving the initial drone position from the launch file
    nh.getParam("/uav1/drone2_controller_node/init_x/", init_x);
    nh.getParam("/uav1/drone2_controller_node/init_y/", init_y);
    nh.getParam("/uav1/drone2_controller_node/init_z/", init_z);

    // This needs to be set manually if the launch file is not used.
    // init_x = -2;
    // init_y = 3;
    // init_z = 0;
    
    // Establishing publishers and subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    global_pos_pub = nh.advertise<project::Drone_pos>("/uav1/drone2_global_pos", 10);
    
    posexyz = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, poseCallBack);
    alt_change = nh.subscribe<project::Drone_pos>("/drone_alt_change", 10, alt_change_cb);
    lead_pos = nh.subscribe<project::Drone_pos>("/uav0/drone1_global_pos", 10, lead_pos_cb);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 5Hz
    ros::Rate rate(5.0);
    
    ROS_INFO("Waiting for FCU connection...");
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Drone 2 FCU Connected");
    
    // Setting altitude position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    // Sending some setpoints before mission start
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    
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
    	if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)) )
    	{
    		ROS_INFO("You can now switch Drone 2 to OFFBOARD mode.");
    		last_request = ros::Time::now();
    	}
    	else
    	{
    		if( current_state.mode == "OFFBOARD" )
    		{
    			ROS_INFO("Drone 2 Switched to OFFBOARD mode");

    			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)) )
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

        local_pos_pub.publish(pose);
        
        project::Drone_pos Drone_pos;
        Drone_pos.d2_x = rel_x;
        Drone_pos.d2_y = rel_y;
        Drone_pos.d2_z = rel_z;
        global_pos_pub.publish(Drone_pos);

    	ros::spinOnce();
    	rate.sleep();
    }
    
    started = true;
    
    // Flight Mission
    while(ros::ok() && current_state.armed && current_state.mode == "OFFBOARD")
    {
        // Setting mode to OFFBOARD if not already set
        //if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( set_mode_client.call(offb_mode) && offb_mode.response.mode_sent)
        //    {                ROS_INFO("Drone 2 Offboard enabled");            }
        //    last_request = ros::Time::now();
        //}
        //
        // Arming drone if not already armed
        //if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        //    {                    ROS_INFO("Drone 2 Vehicle armed");                }
        //    mission_progress++;
        //    ROS_INFO("Drone 2 Position Home");
        //    last_request = ros::Time::now();
        //}
        
        // Starting mission param
        if(started==true)
        {
            mission_progress = 1;
            started = false;
        }
        
        // Following lead drone with collision avoidance logic
        if ( mission_progress > 0 && (ros::Time::now() - last_request > ros::Duration(35.0)))
        {
            pose.pose.position.x = (lead_x - init_x) - 2;
            pose.pose.position.y = (lead_y - init_y) - 2;
            pose.pose.position.z = drone_change;
        }

        // Publishing topics
        local_pos_pub.publish(pose);
        
        project::Drone_pos Drone_pos;
        Drone_pos.d2_x = rel_x;
        Drone_pos.d2_y = rel_y;
        Drone_pos.d2_z = rel_z;
        global_pos_pub.publish(Drone_pos);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
