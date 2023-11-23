#include <ros/ros.h>
#include <cmath>
#include <uas_offboard_planner/Drone_pos.h>

ros::Subscriber drone1_sub;
ros::Subscriber drone2_sub;
ros::Subscriber drone3_sub;
ros::Publisher drone_change;

double d1_x, d1_y, d1_z;
double d2_x, d2_y, d2_z;
double d3_x, d3_y, d3_z;
double d12, d23, d31;
double d1_change, d2_change, d3_change;

uas_offboard_planner::Drone_pos Drone_global_pos;
void d1_pos_cb(const uas_offboard_planner::Drone_pos::ConstPtr& msg)
{
    // Drone 1 position
    d1_x = msg->d1_x;
    d1_y = msg->d1_y;
    d1_z = msg->d1_z;
    
    // ROS_INFO("Drone 1 pos: %f, %f, %f", d1_x, d1_y, d1_z);
    
}

void d2_pos_cb(const uas_offboard_planner::Drone_pos::ConstPtr& msg)
{
    // Drone 2 position
    d2_x = msg->d2_x;
    d2_y = msg->d2_y;
    d2_z = msg->d2_z;
    
    // ROS_INFO("Drone 2 pos: %f, %f, %f", d2_x, d2_y, d2_z);
    
}

void d3_pos_cb(const uas_offboard_planner::Drone_pos::ConstPtr& msg)
{
    // Drone 3 position
    d3_x = msg->d3_x;
    d3_y = msg->d3_y;
    d3_z = msg->d3_z;
    
    // ROS_INFO("Drone 3 pos: %f, %f, %f", d3_x, d3_y, d3_z);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_avoidance");
    ros::NodeHandle nh;
    
    // Subscribing to the global drone positions
    drone1_sub = nh.subscribe<uas_offboard_planner::Drone_pos>("/uav0/drone_global_pos", 10, d1_pos_cb);
    drone2_sub = nh.subscribe<uas_offboard_planner::Drone_pos>("/uav1/drone_global_pos", 10, d2_pos_cb);
    drone3_sub = nh.subscribe<uas_offboard_planner::Drone_pos>("/uav2/drone_global_pos", 10, d3_pos_cb);
    drone_change = nh.advertise<uas_offboard_planner::Drone_pos>("/drone_alt_change", 10);
    
    ros::Rate rate(20.0);
    
    ros::Time last_request = ros::Time::now();
    
    double min_dist = 1;
    
    while (ros::ok())
    {
    	
    	d12 = std::sqrt(std::pow(d1_x - d2_x, 2) + std::pow(d1_y - d2_y, 2));
    	d23 = std::sqrt(std::pow(d2_x - d3_x, 2) + std::pow(d2_y - d3_y, 2));
    	d31 = std::sqrt(std::pow(d3_x - d1_x, 2) + std::pow(d3_y - d1_y, 2));
    	
    	
    	// The rest of this code still applies to the OFFBOARD mode of the drones.
    	if (d12 > min_dist && d23 > min_dist && d31 > min_dist)
    	{
    	    d1_change = 2;
    	    d2_change = 2;
    	    d3_change = 2;
    	}
    	if (d12 < min_dist && d23 < min_dist && d31 < min_dist)
    	{
    	    d1_change = 3;
    	    d2_change = 2;
    	    d3_change = 1;
    	}
    	if (d12 < min_dist)
    	{
    	    d1_change = 3;
    	    d2_change = 1;
    	    d3_change = 2;
    	}
    	if (d23 < min_dist)
    	{
    	    d1_change = 2;
    	    d2_change = 3;
    	    d3_change = 1;
    	}
    	if (d31 < min_dist)
    	{
    	    d3_change = 3;
    	    d1_change = 1;
    	    d2_change = 2;
    	}
    	
    	// Code here will be introduced here to be used when UAV0 is in position mode so the other drones will follow.
    	// Psudo
    	// If the state of UAV0 is not in OFFBOARD				// Need to introduce UAV0 state subscriber and callback.
    	// 	The altitude of drone 2 and 3 will mimic drone 1.
    	// end
    	
    	// Publishing the altitude adjustment for each drone
    	uas_offboard_planner::Drone_pos altitude;
    	altitude.d1_change = d1_change;
    	altitude.d2_change = d2_change;
    	altitude.d3_change = d3_change;
    	drone_change.publish(altitude);
    	
    	
    	
    	if( ros::Time::now() - last_request > ros::Duration(1.0) )
    	{
    		ROS_INFO("--------------------------------------------");
    		ROS_INFO("Distances: %.2f, %.2f, %.2f", d12, d23, d31);
    		ROS_INFO("Changes: D1-%.2f, D2-%.2f, D3-%.2f", d1_change, d2_change, d3_change);
    		last_request = ros::Time::now();
    	}
    	
    	rate.sleep();
    	ros::spinOnce();
    	
    }
    
    return 0;
}
