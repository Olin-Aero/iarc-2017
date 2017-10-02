#include "ros_robot.h"
#include <geometry_msgs/Pose2D.h>

ROSRobot::ROSRobot(ros::NodeHandle& nh,
        std::string name,
        std::string img,
        float x,
        float y,
        float t
        ){
    pub = nh.advertise<geometry_msgs::Pose2D>("", 5);
    sub = nh.subscribe("cmd_vel", 10, &ROSRobot::cmd_vel_cb, this);
}

void ROSRobot::publish(){

}

void ROSRobot::cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg){
    float vx = msg->linear.x;
    float vy = msg->linear.y;
    float w = msg->angular.z;
    robot.set_vel(vx,vy,w);
}
