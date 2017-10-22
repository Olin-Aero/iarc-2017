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
        #self.listener = tf.TransformListener()
        self.ActualHeight = 4
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmdHeight = rospy.Publisher('/cmd_height', Float64, queue_size=10)
        rospy.init_node('Example')        
        rospy.Subscriber("/drone/height", Float64, self.recordActualHeight)
        time.sleep(1)
    def user_input_callback(self, msg):
        print msg.data
  #  def recordNeededHeight(self,msg):
   #     self.neededHeight = msg.data
        print ("me too")
    def recordActualHeight(self,msg):
        self.actualHeight = msg.data
    def run(self):
        outing = Float64()
        outing.data = 2.0
        self.cmdHeight.publish(outing)   
        #self.changeHeight()
    def changeHeight(self,height):
        heightRequested = height
        currentHeight = self.neededHeight
        if(currentHeight < heightRequested - 0.05):
            output = Twist()
            output.linear = Vector3(0, 0, 1)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
            print ("I'm here")
        elif(currentHeight > heightRequested + 0.05):
            output = Twist()
            output.linear = Vector3(0, 0, -1)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
            print ("I'm there")
        else:
            output = Twist()
            output.linear = Vector3(0, 0, 0)
            output.angular = Vector3(0,0,0)
            self.heightPub.publish(output)
            print ("I'm over here")
  #  def toGroundFloor():
       # changeHeight(0)
   # def tapRoomba():
        #changeHeight(0.92)
   # def toCruising():
        #changeHeight(2.8)
if __name__ == '__main__':
    ex = ChangingHeight()
    ex.run()
    rospy.spin()