#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_datatypes.h>
#include <cmath>


using namespace std;

const float AVOIDANCE_DISTANCE = 0.4;
const float REPULSIVE_FORCE = 0.1;


double first_yaw = 0.0;     
bool set_yaw = false;
const double ROTATION_GOAL = M_PI;  
const float ROTATION_SPEED = 1.0; // rad/s


enum RobotState {ACTIVE, ROTATING, GOING_BACK};
RobotState robot_state = ACTIVE;

ros::Publisher pub_velocity;
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
    //ROS_INFO("Laser is working");
    velocity_received = false;


    //First try
    
    //take the index and the distance of the closest obstacle from the laser
    int index = -1;
    float min_distance = 100000000;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        //distance of the ray at index i
        float actual_distance = msg->ranges[i]; 
        if (!isinf(actual_distance) && !isnan(actual_distance) && actual_distance < min_distance) {
            min_distance = actual_distance;
            index = i;
        }
    }

    if (index == -1){ 
        ROS_INFO("Nessun ostacolo rilevato!"); 
        return; 
    }
    //ROS_INFO("Ostacolo rilevato");

    //direction of the closest obstacle from the laser
    float obstacle_direction = msg->angle_min + (index * msg->angle_increment);
    
    //Transforming the obstacle in (x,y) respect to the robot

    //coordinates of the closest robot respect to the laser frame
    geometry_msgs::PointStamped obstacle_from_laser;
    obstacle_from_laser.header = msg->header;
    //computing the coordinate x and y in laser frame
    obstacle_from_laser.point.x = min_distance * cos(obstacle_direction);
    obstacle_from_laser.point.y = min_distance * sin(obstacle_direction);
    obstacle_from_laser.point.z = 0.0;


    static tf::TransformListener listener;
    geometry_msgs::PointStamped obstacle_from_robot;

    try {
        listener.waitForTransform("base_footprint", obstacle_from_laser.header.frame_id, ros::Time(0), ros::Duration(1.0));
        listener.transformPoint("base_footprint", ros::Time(0), obstacle_from_laser, obstacle_from_laser.header.frame_id, obstacle_from_robot);
    } catch(tf::TransformException &e) {
        ROS_ERROR("Transform error: %s", e.what());
        return;
    }


    float distance_from_robot = sqrt(pow(obstacle_from_robot.point.x, 2) + pow(obstacle_from_robot.point.y, 2));


    geometry_msgs::Twist msg_send;

    if(robot_state == ACTIVE){
        ROS_INFO("SONO AL SICURO");
        if(distance_from_robot < AVOIDANCE_DISTANCE){
            msg_send.linear.x = 0.0;
            msg_send.linear.y = 0.0;
            msg_send.angular.z = 0.0;
            robot_state = ROTATING; 
            ROS_INFO("Entro in rotazione");
        }
        else{
        float force_x = - (obstacle_from_robot.point.x / distance_from_robot) * REPULSIVE_FORCE;
        float force_y = - (obstacle_from_robot.point.y / distance_from_robot) * REPULSIVE_FORCE;

        msg_send.linear.x = keyboard_velocity.linear.x + force_x;
        msg_send.linear.y = keyboard_velocity.linear.y + force_y;
        msg_send.angular.z = keyboard_velocity.angular.z;
    }
        

    }

    else if(robot_state == ROTATING){
        msg_send.linear.x = 0.0;
        msg_send.linear.y = 0.0;
        ROS_INFO("SONO IN ROTAZIONE");


        tf::StampedTransform transform;
        try{
            listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);
        } catch(tf::TransformException &e){
            ROS_ERROR("%s", e.what());
            return;
        }

        double current_yaw = tf::getYaw(transform.getRotation());
        ROS_INFO("current_yaw = %.3f, first_yaw = %.3f", current_yaw, first_yaw);

        
        if(!set_yaw){
            first_yaw = current_yaw;
            set_yaw = true;
            ROS_INFO("Imposto first_yaw = %.3f", first_yaw);
        }

        double delta_yaw = fabs(current_yaw - first_yaw);
        ROS_INFO("delta_yaw = %.3f", delta_yaw);

        if(delta_yaw < ROTATION_GOAL){
            msg_send.angular.z = ROTATION_SPEED;
            
        } 

        else{
            ROS_INFO("Rotazione completata, passo a GOING_BACK");
            msg_send.angular.z = 0.0;
            robot_state = GOING_BACK;
            set_yaw = false; 
        }

    }


    else {  

        ROS_INFO("STO TORNANDO INDIETRO");
        msg_send.linear.x = 0.5;  
        msg_send.linear.y = 0.0;
        msg_send.angular.z = 0.0;  

        if(distance_from_robot >= AVOIDANCE_DISTANCE){
            robot_state = ACTIVE;
        }
    }

    pub_velocity.publish(msg_send);

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "obstacle_avoidance");
    ros::NodeHandle nh;


    ros::Subscriber keyboard_vel_sub = nh.subscribe("keyboard_vel_call", 1, keyboard_vel_callback);

    ros::Subscriber laser_sub = nh.subscribe("base_scan", 1, laser_callback);
    ROS_INFO("Node started");

    pub_velocity = nh.advertise<geometry_msgs::Twist>("cmd_vel",1);

    ros::spin();

    return 0;
}