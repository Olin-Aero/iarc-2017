#!/usr/bin/env python
import rospy
import random
from geometry_msgs.msg import Twist, Vector3
#from neato_node import Bump
class RealRoomba():
	def __init__(self):
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.init_node('RoombaCode')
		rospy.Timer(rospy.Duration(5.),self.Turn)
	#def listener(self):
		# In ROS, nodes are uniquely named. If two nodes with the same
		# node are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
	#	rospy.init_node('listener', anonymous=True)
		#rospy.Subscriber('bump',neato_node/Bump,self.callback)
	def callback(self,data):
		print(data)
		output = Twist()
		output.linear = Vector3(0.33,0,0)
	def practice(self):
		print(rospy.get_rostime().secs)
		output = Twist()
		output.linear = Vector3(0.33,0,0)
		output.angular = Vector3(0,0,0)
		self.pub.publish(output)
		rospy.sleep(4.5)
	def Turn(self,event):
		# ros::Time start_time = ros::Time::now()
		# ros::Duration timeout(2.0)
		# while(ros::Time::now() - start_time < timeout):
		output = Twist()
		output.linear = Vector3(0,0,0)
		output.angular = Vector3(0,0,0.5)
		self.pub.publish(output)
if __name__ == '__main__':
	ex = RealRoomba()
	ex.practice()
	rospy.spin()