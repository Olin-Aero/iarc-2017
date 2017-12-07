#include "ros_robot.h"
#include <geometry_msgs/Pose2D.h>

ROSRobot::ROSRobot(
        ros::NodeHandle& nh,
        const std::string name
        ):Robot(name){
    pub = nh.advertise<geometry_msgs::Pose2D>(name + "/pose", 5);
    sub = nh.subscribe(name + "/cmd_vel", 10, &ROSRobot::cmd_vel_cb, this);
}

ROSRobot::~ROSRobot(){
    //pub-sub should auto shutdown anyways
}

void ROSRobot::publish(){
    //broadcast tf
    tf::Transform xform;
    xform.setOrigin(tf::Vector3(x,y,0.0));
    tf::Quaternion q;
    q.setRPY(0,0,t);
    xform.setRotation(q);
    br.sendTransform(tf::StampedTransform(xform, ros::Time::now(), "map", name));

    //also publish
    pos_msg.x=x;
    pos_msg.y=y;
    pos_msg.theta=t;
    pub.publish(pos_msg);
}

void ROSRobot::cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg){
    float v = msg->linear.x;
    float w = msg->angular.z;
    set_vel(v,w);
}
