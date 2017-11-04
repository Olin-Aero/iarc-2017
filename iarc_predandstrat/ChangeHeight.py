#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import math
from geometry_msgs.msg import Twist, Vector3

class ChangingHeight:
    def __init__(self):
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmdHeight = rospy.Publisher('/cmd_height', Float64, queue_size=10)
        rospy.init_node('Example')        
        rospy.Subscriber("/drone/height", Float64, self.callback)
        rospy.sleep(0.1)
    def callback(self, msg):
        self.actualHeight = msg.data
    def changeHeight(self,height):
        heightRequested = height
        currentHeight = self.actualHeight
        if(currentHeight < heightRequested - 0.05):
            output = Twist()
            output.linear = Vector3(0, 0, 2)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
        elif(currentHeight > heightRequested + 0.05):
            output = Twist()
            output.linear = Vector3(0, 0, -2)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
        else:
            output = Twist()
            output.linear = Vector3(0, 0, 0)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
    def heightLoop(self,height):
        while(True):
            self.changeHeight(height)
            rospy.sleep(.1)
            #print(self.actualHeight)
if __name__ == '__main__':
    ex = ChangingHeight()
    ex.heightLoop(3.0)
    rospy.spin()