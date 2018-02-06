#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import math
from geometry_msgs.msg import Twist, Vector3
from ChangeHeight import ChangingHeight
from follow_roomba import following, roomba

class PushRoomba:
	def __init__(self):
		self.ButtonHit = False
		rospy.Subscriber("/drone/height", Float64, self.callback)
		self.random = ChangingHeight()
		self.follower = following()
	def callback(self,msg):
		self.actualHeight = msg.data
	def run(self,roomba):
		#follow the robot code
		self.follower.follow_roomba(roomba)
		#if(not self.ButtonHit && distancefromroomba < 0.17):
		#	self.random.changeHeight(0.05)
		#else:
		#	self.random.changeHeight(3.0)
		#print(self.actualHeight)
		#if(self.actualHeight < 0.1):
		#	self.ButtonHit = True
	def runLoop(self):
		while(True):
			self.run("target1")
			rospy.sleep(.1)
if __name__ == '__main__':
    ex = PushRoomba()
    ex.runLoop()
    rospy.spin()

