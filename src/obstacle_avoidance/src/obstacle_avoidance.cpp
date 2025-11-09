#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <cmath>

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


    //First try

    //take the index and the distance of the closest obstacle from the laser
    int index = -1;
    float min_distance = 100000000;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        //distance of the ray at index i
        float actual_distance = msg->ranges[i]; 
        if (!std::isinf(actual_distance) && !std::isnan(actual_distance) && actual_distance < min_distance) {
            min_distance = actual_distance;
            index = i;
        }
    }

    if (index == -1){ 
        ROS_INFO("Nessun ostacolo rilevato!"); 
        return; 
    }
    ROS_INFO("Ostacolo rilevato");

    //direction of the closest obstacle from the laser
    float obstacle_direction = msg->angle_min + (index * msg->angle_increment);
    
    //Transforming the obstacle in (x,y) respect to the robot

    //coordinates of the closest robot respect to the laser frame
    geometry_msgs::PointStamped obstacle_from_laser;
    obstacle_from_laser.header = msg->header;
    //computing the coordinate x and y 
    obstacle_from_laser.point.x = min_distance * cos(obstacle_direction);
    obstacle_from_laser.point.y = min_distance * sin(obstacle_direction);
    obstacle_from_laser.point.z = 0.0;






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