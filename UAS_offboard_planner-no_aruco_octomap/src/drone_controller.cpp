#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <uas_offboard_planner/Drone_pos.h>



// Creating subscribers, publishers and variables
ros::Subscriber state_sub;
ros::Subscriber mavros_pos;
ros::Subscriber alt_change;
ros::Subscriber lead_pos

ros::Publisher local_pos_pub;
ros::Publisher position_pub;
ros::Publisher global_pos_pub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;


// Creating all variables
std::string uav_num;
std::string uav_name;
std::string get_mavros_state;
std::string get_mavros_pos;
std::string send_local_pos;
std::string send_global_pos;
std::string get_posexyz;
std::string set_arming;
std::string set_mode;

double current_x, current_y, current_z;
double drone_change = 2;
int mission_progress = 0;
bool started;



// CALLBACKS
// Grabs the current state of the UAV
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// Looking into the mavros position of the drone based on the vicon position and sets it to current position
void mavros_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_x = msg->pose.position.x;	// Local x Position (Forward / Backward)
    current_y = msg->pose.position.y;	// Local y Position (Left / Right)
    current_z = msg->pose.position.z;	// Local z Position (Altitude)
    
    // ROS_INFO("x = %.2f, y = %.2f, z = %.2f", current_x, current_y, current_z);
}

// Grabbing the drone_change position (I.e. the altitude change for the vehicle calculated from the collision avoidance node)
void alt_change_cb(const uas_offboard_planner::Drone_pos::ConstPtr& msg) {
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

void lead_pos_cb(const uas_offboard_planner::Drone_pos::ConstPtr& msg) {
    lead_x = msg->d1_x;
    lead_y = msg->d1_y;
}


// MAIN LOOP
int main(int argc, char **argv)
{
    ROS_INFO("Initialization");
    ros::init(argc, argv, "drone_controller");
    ros::NodeHandle nh;
    
    // Setting started to false so that the mission loop doesn't begin until the vehicle is started.
    started = false;
    
    // Setting the UAV name to be the argument from the launch file
    uav_num = argv[1];
    uav_name = "uav" + uav_num;
    
    // Establishing publishers and subscribers
    get_mavros_state = "/" + uav_name + "/mavros/state";
    state_sub = nh.subscribe<mavros_msgs::State>(get_mavros_state, 10, state_cb);				// State Subscriber
    alt_change = nh.subscribe<uas_offboard_planner::Drone_pos>("/drone_alt_change", 10, alt_change_cb);	// Altitude change Subscriber from Collision Avoidance node
    
    get_mavros_pos = "/" + uav_name + "/mavros/vision_pose/pose";
    send_local_pos = "/" + uav_name + "/mavros/setpoint_position/local";
    send_global_pos = "/" + uav_name + "/drone_global_pos";
    mavros_pos = nh.subscribe<geometry_msgs::PoseStamped>(get_mavros_pos, 10, mavros_pos_cb);		// Mavros vehicle position Subscriber
    lead_pos = nh.subscribe<uas_offboard_planner::Drone_pos>("/uav0/drone_global_pos", 10, lead_pos_cb);	// Lead vehicle position Subscriber
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(send_local_pos, 10);				// Local vehicle position Publisher
    global_pos_pub = nh.advertise<uas_offboard_planner::Drone_pos>(send_global_pos, 10);			// Global vehicle position Publisher
    														// (Not actually gps position, just Mavros vehicle position)
    set_arming = "/" + uav_name + "/mavros/cmd/arming";
    set_mode = "/" + uav_name + "/mavros/set_mode";
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>(set_arming);					// ROS arming service
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(set_mode);					// ROS set mode service
    
    //the setpoint publishing rate MUST be faster than 5Hz
    ros::Rate rate(5.0);
    
    
    
    ROS_INFO("-----------------------------");
    ROS_INFO("Waiting for FCU connection...");
    ROS_INFO("-----------------------------");
    
    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    
    ROS_INFO("-----------------------------");
    ROS_INFO("UAV%s FCU Connected", uav_num.c_str());
    ROS_INFO("-----------------------------");
    
    
    
    // Setting the initial vehicle position when armed.
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = current_x;
    pose.pose.position.y = current_y;
    pose.pose.position.z = 2;
    
    // Sending some setpoints before mission start (Important for manual mode selection)
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
    	if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(1.0)) )
    	{
    		ROS_INFO("-----------------------------");
    		ROS_INFO("You can now switch UAV%s to OFFBOARD mode.", uav_num.c_str());
    		last_request = ros::Time::now();
    	}
    	else
    	{
    		if( current_state.mode == "OFFBOARD" )
    		{
    			ROS_INFO("UAV%s Switched to OFFBOARD mode", uav_num.c_str());

    			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)) )
    			{
    				ROS_INFO("-----------------------------");
    				ROS_INFO("Arming UAV");
    				
    				if( arming_client.call(arm_cmd) && arm_cmd.response.success )
    				{
    					ROS_INFO("-----------------------------");
    					ROS_INFO("UAV Armed");
    				}
    				last_request = ros::Time::now();
    			}
    		}
    	}

	// Publishing topics to msgs
        local_pos_pub.publish(pose);
        uas_offboard_planner::Drone_pos Drone_pos;
        
        if( uav_num == "0" )
        {
        	Drone_pos.d1_x = current_x;
        	Drone_pos.d1_y = current_y;
        	Drone_pos.d1_z = current_z;
        }
        if( uav_num == "1" )
        {
        	Drone_pos.d2_x = current_x;
        	Drone_pos.d2_y = current_y;
        	Drone_pos.d2_z = current_z;
        }
        if( uav_num == "2" )
        {
        	Drone_pos.d3_x = current_x;
        	Drone_pos.d3_y = current_y;
        	Drone_pos.d3_z = current_z;
        }
        global_pos_pub.publish(Drone_pos);

    	ros::spinOnce();
    	rate.sleep();
    }
    
    // The vehicle has started
    started = true;
    
    
    
    // Flight Mission
    while(ros::ok() && current_state.armed /*&& current_state.mode == "OFFBOARD"*/)
    {
    	// 6x8m flight area was used for testing
    
        // Starting mission progress
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
            		pose.pose.position.x = 0;
            		pose.pose.position.y = 0;
            		pose.pose.position.z = drone_change;
    	    		OS_INFO("-----------------------------");
    	    		ROS_INFO("Sending UAV0 to position (0, 0)");
        	}
        	
        	// UAV1 trailing behind Lead drone with collision avoidance logic
        	if ( mission_progress > 0 && uav_num == "1" && (ros::Time::now() - last_request > ros::Duration(35.0)))
        	{
            		pose.pose.position.x = lead_x - 2;
           		pose.pose.position.y = lead_y - 2;
            		pose.pose.position.z = drone_change;
    	    		ROS_INFO("-----------------------------");
    	    		ROS_INFO("Sending UAV1 to follow Lead drone");
        	}
        	
        	// UAV2 trailing behind Lead drone with collision avoidance logic
        	if ( mission_progress > 0 && uav_num == "2" && (ros::Time::now() - last_request > ros::Duration(35.0)))
        	{
            		pose.pose.position.x = lead_x - 2;
            		pose.pose.position.y = lead_y + 2;
            		pose.pose.position.z = drone_change;
    	    		ROS_INFO("-----------------------------");
    	    		ROS_INFO("Sending UAV2 to follow Lead drone");
        	}
        	
        	// Here is where new code would be placed for the mission objectives of the mission.
        	
        	
        	// Publishing topics to msgs
        	local_pos_pub.publish(pose);
        	uas_offboard_planner::Drone_pos Drone_pos;
        	
        	if( uav_num == "0" )
        	
        		Drone_pos.d1_x = current_x;
        		Drone_pos.d1_y = current_y;
        		Drone_pos.d1_z = current_z;
        	}
        	if( uav_num == "1" )
        	{
        		Drone_pos.d2_x = current_x;
        		Drone_pos.d2_y = current_y;
        		Drone_pos.d2_z = current_z;
        	}
        	if( uav_num == "2" )
        	{
        		Drone_pos.d3_x = current_x;
        		Drone_pos.d3_y = current_y;
        		Drone_pos.d3_z = current_z;
        	}
        	global_pos_pub.publish(Drone_pos);
        	
        }
        
        // Attempt to make drone compatible with manual flight controller
        if (ros::ok() && current_state.armed && current_state.mode != "OFFBOARD")
	{
	    	// Publishing topics to msgs
        	local_pos_pub.publish(pose);
        	uas_offboard_planner::Drone_pos Drone_pos;
        	
        	if( uav_num == "0" )
        	
        		Drone_pos.d1_x = current_x;
        		Drone_pos.d1_y = current_y;
        		Drone_pos.d1_z = current_z;
        	}
        	if( uav_num == "1" )
        	{
        		Drone_pos.d2_x = current_x;
        		Drone_pos.d2_y = current_y;
        		Drone_pos.d2_z = current_z;
        	}
        	if( uav_num == "2" )
        	{
        		Drone_pos.d3_x = current_x;
        		Drone_pos.d3_y = current_y;
        		Drone_pos.d3_z = current_z;
        	}
        	global_pos_pub.publish(Drone_pos);
	    
	}
        
        ros::spinOnce();
        rate.sleep();
        
    }
    
    return 0;
}
