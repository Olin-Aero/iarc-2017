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

        maxvelocity = rospy.get_param('~max_velocity', 1.0) # Max velocity the drone can reach
        kpturn = rospy.get_param('~kp_turn', 1.0) # Proportional for turning
        kp = rospy.get_param('~kp', 1.0) # Proportional
        ki = rospy.get_param('~ki', 0.2) # Integral
        kd = rospy.get_param('~kd', 0.0) # Derivative: kd is not currently used

        last_time = rospy.Time.now()
        integral_x = 0.0
        previous_error_x = 0.0
        integral_y = 0.0
        previous_error_y = 0.0

        while not rospy.is_shutdown():
        	if self.tf.frameExists("base_link") and self.tf.frameExists("target0"):
	            # Get the location of the target relative to the drone
	            t = self.tf.getLatestCommonTime("base_link", "target0")
	            position, quaternion = self.tf.lookupTransform("base_link", "target0", t)
                print position

	            # Plan the motion
                cmd = Twist()

                now = rospy.Time.now()
                dt = (now - last_time).to_sec()
                last_time = now

                # In case something weird happens with the loop freezing
                dt = min(dt, 0.5)
                dt = max(dt, 0.01)

                # Calculate DIP velocity_x
                error_x = position[0]
                integral_x = integral_x + error_x * dt
                derivative_x = (error_x - previous_error_x)/dt
                previous_error_x = error_x
                dip_x = kp * error_x + ki * integral_x + kd * derivative_x

                # Calculate DIP velocity_y
                error_y = position[1]
                integral_y = integral_y + error_y * dt
                derivative_y = (error_y - previous_error_y)/dt
                previous_error_y = error_y
                dip_y = kp * error_y + ki * integral_y + kd * derivative_y

                # Combined velocity
                dip_diagonal = math.sqrt(dip_x**2 + dip_y**2)

                if dip_diagonal < maxvelocity:
                    # If dip_diagonal is already < max_velocity
                    # use dip velocity
                    cmd.linear.x = dip_x
                    cmd.linear.y = dip_y
                else:
                    # Else use max velocity
                    diagonalvelocity = math.sqrt(position[0]**2 + position[1]**2)
                    cmd.linear.x = position[0] / diagonalvelocity * maxvelocity
                    cmd.linear.y = position[1] / diagonalvelocity * maxvelocity

                    # Reset integral until we start using it
                    integral_x = 0.0
                    integral_y = 0.0

                # Turn the drone to the target's direction
                cmd.angular.z = kpturn * math.atan2(cmd.linear.y, cmd.linear.x)

                # Publish the motion
                self.pub.publish(cmd)

                r.sleep()

if __name__ == '__main__':
    ex = FollowBehavior()
    ex.run()
