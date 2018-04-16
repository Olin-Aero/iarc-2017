#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.srv import SetMode, CommandTOL
from std_msgs.msg import Empty


class PixhawkConnector(object):
    def __init__(self):
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=0)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.sleep(0.2)
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.land = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        self.takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)

        rospy.loginfo('Services connected')

        self.vel = Twist()
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.on_vel)

        self.takeoff_sub = rospy.Subscriber('/takeoff', Empty, self.on_takeoff)
        self.land_sub = rospy.Subscriber('/land', Empty, self.on_land)


    def setup_pixhawk(self):
        ok = self.set_mode(custom_mode='GUIDED')
        if not ok.mode_sent:
            rospy.logerr("Unable to set Pixhawk mode")

    def on_takeoff(self):
        res = self.takeoff()
        rospy.loginfo("Took Off:", res)
        pass

    def on_land(self):
        res = self.land()
        rospy.loginfo("Landed:", res)
        pass

    def on_vel(self, msg):
        """
        :param Twist msg:
        :return: None
        """
        self.vel = msg
        self.vel_pub.publish(self.vel)

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    rospy.init_node('pixhawk_connector')
    PixhawkConnector().run()
