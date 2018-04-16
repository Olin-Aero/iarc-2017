#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from mavros_msgs.srv import SetMode
from std_msgs.msg import Empty


class PixhawkConnector(object):
    def __init__(self):
        self.vel = Twist()
        self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.on_vel)

        self.takeoff_sub = rospy.Subscriber('/takeoff', Empty, self.on_takeoff)
        self.land_sub = rospy.Subscriber('/land', Empty, self.on_land)

        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=0)

        rospy.wait_for_service('/mavros/set_mode')
        rospy.loginfo('set_mode service connected')
        self.set_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode)

    def setup_pixhawk(self):
        ok = self.set_mode(custom_mode='GUIDED')
        if not ok.mode_sent:
            rospy.logerr("Unable to set Pixhawk mode")

    def on_takeoff(self):
        pass

    def on_land(self):
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
