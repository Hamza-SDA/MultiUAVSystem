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

ros::Publisher local_pos_pub;
ros::Publisher position_pub;
ros::Publisher global_pos_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

double current_x, current_y, current_z;
double init_x, init_y, init_z;
double rel_x, rel_y, rel_z;
double drone_change;
int mission_progress = 0;
bool started;



// CALLBACKS
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
    drone_change = msg->d1_change;
}



// MAIN LOOP
int main(int argc, char **argv)
{
    ROS_INFO("Initialization");
    ros::init(argc, argv, "drone1_controller");
    ros::NodeHandle nh;
    started = false;
    
    
    // Retrieving the initial drone position from the launch file
    nh.getParam("/uav0/drone1_controller_node/init_x/", init_x);
    nh.getParam("/uav0/drone1_controller_node/init_y/", init_y);
    nh.getParam("/uav0/drone1_controller_node/init_z/", init_z);

    // This needs to be set manually if the launch file is not used.
    // init_x = -2;
    // init_y = 1;
    // init_z = 0;
    
    // Establishing publishers and subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("/uav0/mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav0/mavros/setpoint_position/local", 10);
    global_pos_pub = nh.advertise<project::Drone_pos>("/uav0/drone1_global_pos", 10);
    
    posexyz = nh.subscribe<geometry_msgs::PoseStamped>("/uav0/mavros/local_position/pose", 10, poseCallBack);
    alt_change = nh.subscribe<project::Drone_pos>("/drone_alt_change", 10, alt_change_cb);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav0/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav0/mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 5Hz
    ros::Rate rate(5.0);
    
    ROS_INFO("Waiting for FCU connection...");
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("Drone 1 FCU Connected");
    
    
    
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
    		ROS_INFO("You can now switch Drone 1 to OFFBOARD mode.");
    		last_request = ros::Time::now();
    	}
    	else
    	{
    		if( current_state.mode == "OFFBOARD" )
    		{
    			ROS_INFO("Drone 1 Switched to OFFBOARD mode");

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

	// Publishing topics
        local_pos_pub.publish(pose);
        
        project::Drone_pos Drone_pos;
        Drone_pos.d1_x = rel_x;
        Drone_pos.d1_y = rel_y;
        Drone_pos.d1_z = rel_z;
        global_pos_pub.publish(Drone_pos);

    	ros::spinOnce();
    	rate.sleep();
    }
    
    started = true;
    
    
    
    // Flight Mission
    while(ros::ok() && current_state.armed && current_state.mode == "OFFBOARD")
    {
    	// 6x8m flight area
    
    
        // Setting mode to OFFBOARD if not already set
        //if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( set_mode_client.call(offb_mode) && offb_mode.response.mode_sent)
        //    {                ROS_INFO("Drone 1 Offboard enabled");            }
        //    last_request = ros::Time::now();
        //}
        //
        // Arming drone if not already armed
        //if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        //{
        //    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
        //    {                    ROS_INFO("Drone 1 Vehicle armed");                }
        //    mission_progress++;
        //    ROS_INFO("Drone 1 Position Home");
        //    last_request = ros::Time::now();
        //}
        
        // Starting mission param
        if(started==true)
        {
            mission_progress = 1;
            started = false;
        }
        
        // Sending drone to Lead formation position (0, 0)
        if ( mission_progress > 0 && (ros::Time::now() - last_request > ros::Duration(25.0)))
        {
            pose.pose.position.x = current_x - rel_x;
            pose.pose.position.y = current_y - rel_y;
            pose.pose.position.z = drone_change;
        }
        
        // Publishing topics
        local_pos_pub.publish(pose);
        
        project::Drone_pos Drone_pos;
        Drone_pos.d1_x = rel_x;
        Drone_pos.d1_y = rel_y;
        Drone_pos.d1_z = rel_z;
        global_pos_pub.publish(Drone_pos);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}
