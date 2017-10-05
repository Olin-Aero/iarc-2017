#ifndef __ROS_ROBOT_H__
#define __ROS_ROBOT_H__

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "robot.h"

class ROSRobot: public Robot{
    protected:
        ros::Publisher pub;
        ros::Subscriber sub;
        tf::TransformBroadcaster br;
        geometry_msgs::Pose2D pos_msg;
    public:
        ROSRobot(ros::NodeHandle& nh, const std::string name);
        virtual ~ROSRobot();
        void publish();
        void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);
};

#endif
