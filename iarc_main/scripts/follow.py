#!/usr/bin/env python
import rospy
import math
from tf import TransformListener
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3

class FollowBehavior:
    def __init__(self):
        rospy.init_node('FollowBehavior')
        self.pub = rospy.Publisher('/follow/cmd_vel', Twist, queue_size=10)
        self.tf = TransformListener()
        
        # Wait until the transformListener initialize
        rospy.sleep(0.5)
  
    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
        	if self.tf.frameExists("base_link") and self.tf.frameExists("target0"):
	            # Get the location of the target relative to the drone
	            t = self.tf.getLatestCommonTime("base_link", "target0")
	            position, quaternion = self.tf.lookupTransform("base_link", "target0", t)
	            print position

	            # Plan the motion
	            cmd = Twist()

	            # maxvelocity of our drone
	            maxvelocity = 1

	            # diagonalvelocity = the velocity we would get if we use position[0] and position[1]
	            diagonalvelocity = math.sqrt(position[0]**2 + position[1]**2)

	            # Scale down our velocity_x and velocity_y so that our diagonalvelocity won't exceed maxvelocity
	            # But if diagonalvelocity is already < max_velocity then use diagonalvelocity instead (slow down when approach nearer)
	            cmd.linear.x = position[0] if diagonalvelocity < maxvelocity else position[0] / diagonalvelocity * maxvelocity
	            cmd.linear.y = position[1] if diagonalvelocity < maxvelocity else position[1] / diagonalvelocity * maxvelocity
	            
	            # Turn our drone to the position_x/y of the target
	            cmd.angular.z = math.atan2(position[1], position[0])

	            # Publish the motion
	            self.pub.publish(cmd)

        	r.sleep()

if __name__ == '__main__':
    ex = FollowBehavior()
    ex.run()
