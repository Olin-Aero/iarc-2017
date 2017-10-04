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
        r = rospy.Rate(20)

        maxvelocity = rospy.get_param('~max_velocity', 1.0)
        kpturn = rospy.get_param('~kp_turn', 1.0)

        integral_x = 0.0
        previous_error_x = 0.0
        integral_y = 0.0
        previous_error_y = 0.0
        dt = 0.2
        Kp = 1.2
        Ki = 0.2
        Kd = 1

        while not rospy.is_shutdown():
        	if self.tf.frameExists("base_link") and self.tf.frameExists("target0"):
	            # Get the location of the target relative to the drone
	            t = self.tf.getLatestCommonTime("base_link", "target0")
	            position, quaternion = self.tf.lookupTransform("base_link", "target0", t)
                print position

	            # Plan the motion
                cmd = Twist()

                # Calculate DIP velocity_x
                error_x = position[0]
                integral_x = integral_x + error_x * dt
                derivative_x = (error_x - previous_error_x)/dt
                dip_x = Kp * error_x + Ki * integral_x + Kd * derivative_x

                # Calculate DIP velocity_y
                error_y = position[1]
                integral_y = integral_y + error_y * dt
                derivative_y = (error_y - previous_error_y)/dt
                dip_y = Kp * error_y + Ki * integral_y + Kd * derivative_y

                # Combined velocity
                dip_diagonal = math.sqrt(dip_x**2 + dip_y**2)

                if dip_diagonal < maxvelocity:
                    # If diagonalvelocity_ is already < max_velocity
                    # use dip velocity
                    cmd.linear.x = dip_x
                    cmd.linear.y = dip_y
                else:
                    # Else use max velocity
                    diagonalvelocity = math.sqrt(position[0]**2 + position[1]**2)
                    cmd.linear.x = position[0] / diagonalvelocity * maxvelocity
                    cmd.linear.y = position[1] / diagonalvelocity * maxvelocity

                    # Reset integral and derivative until we start using it
                    integral_x = 0.0
                    previous_error_x = 0.0
                    integral_y = 0.0
                    previous_error_y = 0.0

                # Turn our drone to the linear_x/y of
                cmd.angular.z = kpturn * math.atan2(cmd.linear.y, cmd.linear.x)

                # Publish the motion
                self.pub.publish(cmd)

                r.sleep()

if __name__ == '__main__':
    ex = FollowBehavior()
    ex.run()
