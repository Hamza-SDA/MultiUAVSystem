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
ros::Publisher global_pos_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;


// Creating all variables
std::string uav_num;
std::string uav_name;
std::string get_init_x;
std::string get_init_y;
std::string get_init_z;
std::string get_mavros_state;
std::string send_local_pos;
std::string send_global_pos;
std::string get_posexyz;
std::string set_arming;
std::string set_mode;

double current_x, current_y, current_z;
double init_x, init_y, init_z;
double rel_x, rel_y, rel_z;
double lead_x, lead_y;
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
    if( uav_num == "0" )
    {
    	drone_change = msg->d1_change;
    }
    if( uav_num == "1" )
    {
    	drone_change = msg->d2_change;
    }
    if( uav_num == "2" )
    {
    	drone_change = msg->d3_change;
    }
    
}

void lead_pos_cb(const project::Drone_pos::ConstPtr& msg) {
    lead_x = msg->d1_x;
    lead_y = msg->d1_y;
}



// MAIN LOOP
int main(int argc, char **argv)
{
    ROS_INFO("Initialization");
    ros::init(argc, argv, "drone_controller");
    ros::NodeHandle nh;
    started = false;
    
    // Setting the UAV name to be the argument from the launch file
    uav_num = argv[1];
    uav_name = "uav" + uav_num;
    
    ROS_INFO("%s", uav_name.c_str());
    
    // Retrieving the initial drone position from the launch file
    get_init_x = "/" + uav_name + "/drone_controller_node/init_x/";
    get_init_y = "/" + uav_name + "/drone_controller_node/init_y/";
    get_init_z = "/" + uav_name + "/drone_controller_node/init_z/";
    nh.getParam(get_init_x, init_x);
    nh.getParam(get_init_y, init_y);
    nh.getParam(get_init_z, init_z);
    
    // Establishing publishers and subscribers
    get_mavros_state = "/" + uav_name + "/mavros/state";
    send_local_pos = "/" + uav_name + "/mavros/setpoint_position/local";
    send_global_pos = "/" + uav_name + "/drone_global_pos";
    state_sub = nh.subscribe<mavros_msgs::State>(get_mavros_state, 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(send_local_pos, 10);
    global_pos_pub = nh.advertise<project::Drone_pos>(send_global_pos, 10);
    
    get_posexyz = "/" + uav_name + "/mavros/local_position/pose";
    posexyz = nh.subscribe<geometry_msgs::PoseStamped>(get_posexyz, 10, poseCallBack);
    alt_change = nh.subscribe<project::Drone_pos>("/drone_alt_change", 10, alt_change_cb);
    lead_pos = nh.subscribe<project::Drone_pos>("/uav0/drone_global_pos", 10, lead_pos_cb);
    
    
    set_arming = "/" + uav_name + "/mavros/cmd/arming";
    set_mode = "/" + uav_name + "/mavros/set_mode";
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(set_arming);
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_mode);
    
    
    //the setpoint publishing rate MUST be faster than 5Hz
    ros::Rate rate(5.0);
    
    ROS_INFO("Waiting for FCU connection...");
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("UAV%s FCU Connected", uav_num.c_str());
    
    
    
    // Setting altitude position
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;
    
    // Sending some setpoints before mission start
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
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
    	if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(2.0)) )
    	{
    		ROS_INFO("You can now switch UAV%s to OFFBOARD mode.", uav_num.c_str());
    		ROS_INFO("%s", current_state.mode.c_str());
    		last_request = ros::Time::now();
    	}
    	else
    	{
    		if( current_state.mode == "OFFBOARD" )
    		{
    			ROS_INFO("UAV%s Switched to OFFBOARD mode", uav_num.c_str());

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
        
        if( uav_num == "0" )
        {
        	Drone_pos.d1_x = rel_x;
        	Drone_pos.d1_y = rel_y;
        	Drone_pos.d1_z = rel_z;
        }
        if( uav_num == "1" )
        {
        	Drone_pos.d2_x = rel_x;
        	Drone_pos.d2_y = rel_y;
        	Drone_pos.d2_z = rel_z;
        }
        if( uav_num == "2" )
        {
        	Drone_pos.d3_x = rel_x;
        	Drone_pos.d3_y = rel_y;
        	Drone_pos.d3_z = rel_z;
        }
        global_pos_pub.publish(Drone_pos);
        
    	ros::spinOnce();
    	rate.sleep();
    }
    
    started = true;
    
    
    
    // Flight Mission
    while(ros::ok() && current_state.armed /*&& current_state.mode == "OFFBOARD"*/)
    {
    	// 6x8m flight area
        
        // Starting mission param
        if(started==true)
        {
            mission_progress = 1;
            started = false;
        }
        
        if (ros::ok() && current_state.armed && current_state.mode == "OFFBOARD")
        {
        	// Sending drone to Lead formation position (0, 0)
        	if ( mission_progress > 0 && uav_num == "0" && (ros::Time::now() - last_request > ros::Duration(25.0)))
        	{
            	pose.pose.position.x = current_x - rel_x;
            	pose.pose.position.y = current_y - rel_y;
            	pose.pose.position.z = drone_change;
        	}
        	
        	// UAV1 following lead drone with collision avoidance logic
        	if ( mission_progress > 0 && uav_num == "1" && (ros::Time::now() - last_request > ros::Duration(35.0)))
        	{
            	pose.pose.position.x = (lead_x - init_x) - 2;
            	pose.pose.position.y = (lead_y - init_y) - 2;
            	pose.pose.position.z = drone_change;
        	}
        	
        	// UAV2 following lead drone with collision avoidance logic
        	if ( mission_progress > 0 && uav_num == "2" && (ros::Time::now() - last_request > ros::Duration(35.0)))
        	{
            	pose.pose.position.x = (lead_x - init_x) + 2;
            	pose.pose.position.y = (lead_y - init_y) - 2;
            	pose.pose.position.z = drone_change;
        	}
        	
        	// Publishing topics
        	local_pos_pub.publish(pose);
        	project::Drone_pos Drone_pos;
        	
        	if( uav_num == "0" )
        	{
        		Drone_pos.d1_x = rel_x;
        		Drone_pos.d1_y = rel_y;
        		Drone_pos.d1_z = rel_z;
        	}
        	if( uav_num == "1" )
        	{
        		Drone_pos.d2_x = rel_x;
        		Drone_pos.d2_y = rel_y;
        		Drone_pos.d2_z = rel_z;
        	}
        	if( uav_num == "2" )
        	{
        		Drone_pos.d3_x = rel_x;
        		Drone_pos.d3_y = rel_y;
        		Drone_pos.d3_z = rel_z;
        	}
        	global_pos_pub.publish(Drone_pos);
        	
        }
        
        // Attempt to make drone compatible with manual flight controller
        if (ros::ok() && current_state.armed && current_state.mode != "OFFBOARD")
	{
	    // Publishing topics
	    local_pos_pub.publish(pose);
	    project::Drone_pos Drone_pos;
	    
	    if( uav_num == "0" )
	    {
		    Drone_pos.d1_x = rel_x;
		    Drone_pos.d1_y = rel_y;
		    Drone_pos.d1_z = rel_z;
	    }
	    if( uav_num == "1" )
	    {
		    Drone_pos.d2_x = rel_x;
		    Drone_pos.d2_y = rel_y;
		    Drone_pos.d2_z = rel_z;
	    }
	    if( uav_num == "2" )
	    {
		    Drone_pos.d3_x = rel_x;
		    Drone_pos.d3_y = rel_y;
		    Drone_pos.d3_z = rel_z;
	    }
	    global_pos_pub.publish(Drone_pos);
	    
	}
	
	ros::spinOnce();
	rate.sleep();
        
    }
    
    return 0;
}
