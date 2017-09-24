#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

class Example:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Example')
        rospy.Subscriber("/user_input", Float64, self.user_input_callback)

    def user_input_callback(self, msg):
        output = Twist()
        output.linear = Vector3(msg.data, 0, 0)
        self.pub.publish(output)
  
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    ex = Example()
    ex.run()
