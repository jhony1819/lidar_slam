#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Received laser scan data!");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "my_lidar_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/scan", 1000, lidarCallback);

    ros::spin();

    return 0;
}

