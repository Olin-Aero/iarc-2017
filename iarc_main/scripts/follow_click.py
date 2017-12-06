#!/usr/bin/env python2
"""
Tells the drone to hover over the point clicked in rviz, at the commanded height.
Uses /move_base_simple/goal as input
Publishes to /follow/cmd_pos
"""

import rospy
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure

from geometry_msgs.msg import PoseStamped


class ClickFollower(object):
    def __init__(self):
        self.config = DDynamicReconfigure()

        self.config.add_variable("rate", "Frequency to republish updates, in hertz",
                                 rospy.get_param('~rate', 10.0), 0.2, 200.0)

        self.config.add_variable("height", "Height to fly above base frame",
                                 rospy.get_param('~height', 2.0), 0.0, 5.0)

        self.pub = rospy.Publisher('cmd_pos', PoseStamped, queue_size=10)

        self.sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.onmsg)

        self.last_msg = PoseStamped()
        self.has_msg = False

        self.start_ddynrec()

    def onmsg(self, msg):
        """
        :type msg: PoseStamped
        """
        self.last_msg = msg
        self.has_msg = True

        msg.pose.position.z = self.config.height
        msg.header.stamp = rospy.Time(0)

        self.pub.publish(msg)

    def run(self):
        last_rate = None
        r = rospy.Rate(1)  # This is dynamically overridden to be the rate dynamic config

        while not rospy.is_shutdown():
            if self.config.rate != last_rate:
                last_rate = self.config.rate
                r = rospy.Rate(last_rate)

                print "Rate changed to {}".format(last_rate)

            if self.has_msg:
                # Republish the last received message
                self.onmsg(self.last_msg)

            r.sleep()


    def start_ddynrec(self):
        """
        Helper function to start the ddynamic reconfigure server with a callback
        function that updates the self.ddynrec attribute.
        """
        def callback(config, level):
            """
            A callback function used to as the parameter in the ddynrec.start() function.
            This custom callback function updates the state of self.ddynrec so we can
            refer to its variables whenever we have access to it.
            """
            rospy.loginfo("Received reconfigure request: " + str(config))
            # Update all variables
            var_names = self.config.get_variable_names()
            for var_name in var_names:
                self.config.__dict__[var_name] = config[var_name]
            return config

        self.config.start(callback)


if __name__ == '__main__':
    rospy.init_node('follow_clicks')
    ClickFollower().run()

