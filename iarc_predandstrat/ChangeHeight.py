#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import math
from geometry_msgs.msg import Twist, Vector3

class Example:
    def __init__(self):
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Example')
        rospy.Subscriber("/drone/height", Float64, self.user_input_callback)
        time.sleep(1)

    def user_input_callback(self, msg):
        print msg.data



  
    def run(self):   
        self.pub0.publish(output)
    def changeHeight(height):
        if(height > x):
            while(x<height)
                output = Twist()
                output.linear = Vector3(0, 0, 1)
                output.angular = Vector3(0,0,0)
                self.heightPub.publish(output)
        else:
            while(x>height)
                output = Twist()
                output.linear = Vector3(0, 0, -1)
                output.angular = Vector3(0,0,0)
                self.pub0.publish(output)
    def toGroundFloor():
        changeHeight(0)
    def tapRoomba():
        changeHeight(0.92)
    def toCruising():
        changeHeight(2.8)
if __name__ == '__main__':
    ex = Example()
    ex.run()
