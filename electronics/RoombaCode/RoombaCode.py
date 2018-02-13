#!/usr/bin/env python
import rospy
import random
import math
from geometry_msgs.msg import Twist, Vector3
from neato_node.msg import Bump
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
import tf
from nav_msgs.msg import Odometry
class RealRoomba():

	def __init__(self):
		self.countby5 = 0
		self.roomba_speed = 0.10
		self.roomba_turn_speed = -1.0
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

		self.last_bump = Bump()
		rospy.Subscriber('bump',Bump,self.callback)
		
		rospy.init_node('RoombaCode')
		rospy.Timer(rospy.Duration(5.),self.Turn)
		self.practice()
		self.tfFrames = tf.TransformListener()
		rospy.on_shutdown(self.stop)

	def callback(self,data):
		self.last_bump = data

	def practice(self):
		output = Twist()
		output.linear = Vector3(self.roomba_speed,0,0)
		output.angular = Vector3(0,0,0)
		self.pub.publish(output)
	def Turn(self,event):
		if(self.countby5 < 3):
			output = Twist()
			output.linear = Vector3(0,0,0)
			binaryNumber = random.randint(0,1)
			if(binaryNumber == 1):
				output.angular = Vector3(0,0,-self.roomba_turn_speed)
			else:
				output.angular = Vector3(0,0,self.roomba_turn_speed)
			randomAngle = random.uniform(0,0.333333)
			self.pub.publish(output)
			rospy.sleep(randomAngle / (-self.roomba_turn_speed))
			self.countby5 += 1
		else:
			self.turn180()
			self.countby5 = 0
		output = Twist()
		output.linear = Vector3(self.roomba_speed,0,0)
		output.angular = Vector3(0,0,0)
		self.pub.publish(output)

	def stop(self):
		self.pub.publish(Twist())
	def convert_pose_to_xy_and_theta(self,pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x,
							pose.orientation.y,
							pose.orientation.z,
							pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])
	def angle_diff(self,a, b):
		""" Calculates the difference between angle a and angle b (both should be in radians)
        the difference is always based on the closest rotation from angle a to angle b
        examples:
            angle_diff(.1,.2) -> -.1
            angle_diff(.1, 2*math.pi - .1) -> .2
            angle_diff(.1, .2+2*math.pi) -> -.1
    	"""
		a = self.angle_normalize(a)
		b = self.angle_normalize(b)
		d1 = a-b
		d2 = 2*math.pi - math.fabs(d1)
		if d1 > 0:
			d2 *= -1.0
		if math.fabs(d1) < math.fabs(d2):
			return d1
		else:
			return d2

	def angle_normalize(self,z):
		""" convenience function to map an angle to the range [-pi,pi] """
		return math.atan2(math.sin(z), math.cos(z))
	def turn180(self):
		print "Doing turn now!"
		trans, rot = self.tfFrames.lookupTransform('/odom','/base_link',rospy.Time(0))
		Angle = euler_from_quaternion(rot)
		origAng = Angle[2]
		output = Twist()
		#print(euler_from_quaternion(rot))
		output.linear = Vector3(0,0,0)
		output.angular = Vector3(0,0,self.roomba_turn_speed)
		self.pub.publish(output)
		trans, rot = self.tfFrames.lookupTransform('/odom','/base_link',rospy.Time(0))
		Angle = euler_from_quaternion(rot)
		newAng = Angle[2]
		r = rospy.Rate(50)
		while(self.angle_diff(newAng,origAng) > -2.6) and not rospy.is_shutdown():
			self.pub.publish(output)
			trans, rot = self.tfFrames.lookupTransform('/odom','/base_link',rospy.Time(0))
			Angle = euler_from_quaternion(rot)
			newAng = Angle[2]
			r.sleep()
			#print(self.angle_diff(newAng,origAng))
		
		output = Twist()
		output.linear = Vector3(self.roomba_speed,0,0)
		output.angular = Vector3(0,0,0)
		self.pub.publish(output)
	def turn180time(self):
			output = Twist()
			output.linear = Vector3(0,0,0)
			output.angular = Vector3(0,0,self.roomba_turn_speed)
			self.pub.publish(output)
			rospy.sleep(3 / self.roomba_turn_speed)

	def run(self):
		r = rospy.Rate(100)
		while not rospy.is_shutdown():

			if self.last_bump.leftFront == 1 or self.last_bump.rightFront == 1:
				self.turn180()
				
			r.sleep()

if __name__ == '__main__':
	ex = RealRoomba()
	ex.run()