#ifndef __ROS_ROBOT_H__
#define __ROS_ROBOT_H__

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "robot.h"

class ROSRobot{
    protected:
        Robot robot;
        ros::Publisher pub;
        ros::Subscriber sub;
    public:
        ROSRobot(ros::NodeHandle& nh);
        ~ROSRobot();

        void publish();
        void cmd_vel_cb(const geometry_msgs::TwistConstPtr& msg);
}
#endif
