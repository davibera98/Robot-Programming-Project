#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

geometry_msgs::Twist keyboard_velocity;
bool velocity_received = false;

void keyboard_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    keyboard_velocity = *msg;

    if(keyboard_velocity.linear.x != 0 || keyboard_velocity.angular.z != 0){
        velocity_received = true;  
    }
}

void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
    if (!velocity_received) return;
    ROS_INFO("Laser is working");
    velocity_received = false;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;


    ros::Subscriber keyboard_vel_sub = nh.subscribe("keyboard_vel_call", 1, keyboard_vel_callback);

    ros::Subscriber laser_sub = nh.subscribe("base_scan", 1, laser_callback);
    ROS_INFO("Node started");

    ros::spin();

    return 0;
}