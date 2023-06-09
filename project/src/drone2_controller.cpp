#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
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
    
    // Retrieving the initial drone position from the launch file
    // nh.getParam("/uav1/drone1_controller_node/init_x/", init_x);
    // nh.getParam("/uav1/drone1_controller_node/init_y/", init_y);
    // nh.getParam("/uav1/drone1_controller_node/init_z/", init_z);
    
    init_x = -3;
    init_y = -1;
    init_z = 0;
    
    // Establishing publishers and subscribers
    state_sub = nh.subscribe<mavros_msgs::State>("/uav1/mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/uav1/mavros/setpoint_position/local", 10);
    global_pos_pub = nh.advertise<project::Drone_pos>("/uav1/drone2_global_pos", 10);
    
    ros::Publisher cmdVel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel", 10);
    
    posexyz = nh.subscribe<geometry_msgs::PoseStamped>("/uav1/mavros/local_position/pose", 10, poseCallBack);
    alt_change = nh.subscribe<project::Drone_pos>("/drone_alt_change", 10, alt_change_cb);
    lead_pos = nh.subscribe<project::Drone_pos>("/uav0/drone1_global_pos", 10, lead_pos_cb);
    
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/uav1/mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/uav1/mavros/set_mode");
    
    //the setpoint publishing rate MUST be faster than 20Hz
    ros::Rate rate(20.0);
    
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
    
    // Setting to "OFFBOARD" flight mode
    mavros_msgs::SetMode offb_mode;
    offb_mode.request.base_mode = 0;
    offb_mode.request.custom_mode = "OFFBOARD";
    
    // Arming drone
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    ros::Time last_request = ros::Time::now();
    
    // Flight Mission
    while(ros::ok())
    {
        // Setting mode to OFFBOARD if not already set
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_mode) && offb_mode.response.mode_sent)
            {                ROS_INFO("Drone 2 Offboard enabled");            }
            last_request = ros::Time::now();
        }
        
        // Arming drone if not already armed
        if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {                    ROS_INFO("Drone 2 Vehicle armed");                }
            mission_progress++;
            ROS_INFO("Drone 2 Position Home");
            last_request = ros::Time::now();
        }
        
        // Landing
        //if ( mission_progress == 2 && current_z > 2 && (ros::Time::now() - last_request > ros::Duration(25.0)))
        //{
        //    pose.pose.position.z = 0.5;
        //    mission_progress++;
        //    ROS_INFO("Drone 2 Landing");
        //    break;
        //}
        
        // Following lead drone with collision avoidance logic
        if ( mission_progress > 0 && (ros::Time::now() - last_request > ros::Duration(15.0)))
        {
            pose.pose.position.x = (lead_x - init_x) - 1;
            pose.pose.position.y = (lead_y - init_y) - 1;
            pose.pose.position.z = drone_change;
        }
        
        // ROS_INFO("%.2f, %.2f, %.2f", current_x, rel_x, rel_y);

        // Publishing position topics
        local_pos_pub.publish(pose);
        
        geometry_msgs::Twist cmdVel;
        cmdVel.linear.x = 0.2;
        cmdVel_pub.publish(cmdVel);
        
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
