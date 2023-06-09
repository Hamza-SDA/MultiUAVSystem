#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

void globalPositionCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    //ROS_INFO("UAV0 Global Position - Latitude: %f, Longitude: %f, Altitude: %f", msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav0_position_tracker");
    ros::NodeHandle nh;

    ros::Subscriber global_position_sub = nh.subscribe<sensor_msgs::NavSatFix>
    ("/uav0/mavros/global_position/global", 10, globalPositionCallback);

    ros::spin();

    return 0;
}
