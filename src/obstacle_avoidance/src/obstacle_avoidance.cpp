#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
        

geometry_msgs::Twist keyboard_velocity;

void keyboard_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    keyboard_velocity = *msg;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;


    ros::Subscriber keyboard_vel_sub = nh.subscribe("keyboard_vel_call", 1, keyboard_velocity_callback);
    ROS_INFO("Node started");

    ros::spin();

    return 0;
}