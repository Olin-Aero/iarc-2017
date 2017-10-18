import math
import rospy
from geometry_msgs.msg import Twist
from tf import TransformListener


class RoombaState(object):
    def __init__(self, pos=None, heading=0, clock=0):
        if pos is None:
            pos = [0, 0]
        self.pos = pos
        self.heading = heading  # number from 0 to 2*pi
        self.clock = clock


def follow_roomba(roomba_state, relative_x=0, relative_y=0, aborted=False):
    if not aborted:
        # Do something
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        r = rospy.Rate(20)
        tf = TransformListener()
        # Wait until the transformListener initialize
        rospy.sleep(0.5)

        maxvelocity = rospy.get_param('~max_velocity', 1.0)  # Max velocity the drone can reach
        kpturn = rospy.get_param('~kp_turn', 1.0)  # Proportional for turning
        kp = rospy.get_param('~kp', 1.0)  # Proportional
        ki = rospy.get_param('~ki', 0.2)  # Integral
        kd = rospy.get_param('~kd', 0.0)  # Derivative: kd is not currently used

        last_time = rospy.Time.now()
        integral_x = 0.0
        previous_error_x = 0.0
        integral_y = 0.0
        previous_error_y = 0.0
        while not rospy.is_shutdown() and not aborted:
            cmd = Twist()

            t = tf.getLatestCommonTime("drone", "map")
            position, quaternion = tf.lookupTransform("drone", "map", t)
            des_x = position[0] - roomba_state.pos[0] + relative_x
            des_y = position[1] - roomba_state.pos[1] + relative_y
            if des_y + des_y == 0:
                return True
            des_pos = [des_x, des_y]

            now = rospy.Time.now()
            dt = (now - last_time).to_sec()
            last_time = now

            # In case something weird happens with the loop freezing
            dt = min(dt, 0.5)
            dt = max(dt, 0.01)

            # Calculate DIP velocity_x
            error_x = des_pos[0]
            integral_x = integral_x + error_x * dt
            derivative_x = (error_x - previous_error_x) / dt
            previous_error_x = error_x
            dip_x = kp * error_x + ki * integral_x + kd * derivative_x

            # Calculate DIP velocity_y
            error_y = des_pos[1]
            integral_y = integral_y + error_y * dt
            derivative_y = (error_y - previous_error_y) / dt
            previous_error_y = error_y
            dip_y = kp * error_y + ki * integral_y + kd * derivative_y

            # Combined velocity
            dip_diagonal = math.sqrt(dip_x ** 2 + dip_y ** 2)

            if dip_diagonal < maxvelocity:
                # If dip_diagonal is already < max_velocity
                # use dip velocity
                cmd.linear.x = dip_x
                cmd.linear.y = dip_y
            else:
                # Else use max velocity
                diagonalvelocity = math.sqrt(des_pos[0] ** 2 + des_pos[1] ** 2)
                cmd.linear.x = des_pos[0] / diagonalvelocity * maxvelocity
                cmd.linear.y = des_pos[1] / diagonalvelocity * maxvelocity

            cmd.angular.z = kpturn * math.atan2(cmd.linear.y, cmd.linear.x)
            pub.publish(cmd)
            r.sleep()

    else:  # Action Interrupted
        return False


if __name__ == '__main__':
    rospy.init_node('FollowBehavior')
    roomba_state = RoombaState([10, 10], 0, 0)
    print follow_roomba(roomba_state, 0, 0, False)
