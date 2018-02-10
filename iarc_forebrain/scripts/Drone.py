import math

import rospy
import tf
import tf.transformations
from ardrone_autonomy.msg import Navdata
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from iarc_arbiter.msg import RegisterBehavior
from iarc_main.msg import Roomba
from numpy import pi
from std_msgs.msg import String, Empty, Header


class Drone:
    ROOMBA_HEIGHT = 0.1  # Height of a Roomba top in meters
    MAX_VERTICAL_VEL = 2.0
    MIN_VERTICAL_VEL = 0.8
    NORMAL_HEIGHT = 2.5

    def __init__(self, target=None):
        self.should_hit_button = False
        self.should_land_front = False
        self.vel3d = Twist()

        # World state
        self._remembers_flying = False

        # Remembered control state
        self.last_height = 0.0
        self.current_target = target  # Roomba class

        self.prev_target_facing_angle = None  # in radian

        self.FRAME_ID = "base_link"
        self.tf = tf.TransformListener()

        self.posPub = rospy.Publisher('/forebrain/cmd_pos', PoseStamped, queue_size=0)
        self.velPub = rospy.Publisher('/forebrain/cmd_vel', Twist, queue_size=0)
        self.takeoffPub = rospy.Publisher('/forebrain/cmd_takeoff', Empty, queue_size=0)
        self.landPub = rospy.Publisher('/forebrain/cmd_land', Empty, queue_size=0)

        rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='forebrain')
        rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('forebrain')

        rospy.Subscriber("/cmd_vel", Twist, self.record_vel)

        self.navdata = None
        rospy.Subscriber('/ardrone/navdata', Navdata, self.onNavdata)

    def is_flying(self):
        """
        Tells whether the drone is currently flying. That includes the state where it is in the process
        of landing, but not the state where it is in the process of taking off.
        :return (Bool): is the drone flying?
        """
        if self.navdata is not None:
            # List of states taken from
            # http://ardrone-autonomy.readthedocs.io/en/latest/reading.html#legacy-navigation-data
            # "flying" = Flying or Hovering or Landing or Looping
            return self.navdata.state in [3, 7, 4, 8, 9]
        else:
            return self._remembers_flying

    def takeoff(self, height=NORMAL_HEIGHT):
        """
        Commands the drone to takeoff from ground level. Directly commands the low-level controls,
        and might need changes as hardware gets upgraded.
        TODO: Use the height immediately
        :param (float) height: Height in meters
        :return: None
        """
        self.last_height = height

        self.takeoffPub.publish(Empty())
        self._remembers_flying = True

    def land(self):
        """
        Commands the drone to takeoff from ground level. Directly commands the low-level controls,
        and might need changes as hardware gets upgraded.
        :param (float) height: Height in meters
        :return: None
        """
        self.last_height = 0

        self.landPub.publish(Empty())
        self._remembers_flying = False

    def hover(self, time, height=None):
        """
        Publishes 0 velocity for the given duration
        TODO: Store the starting position, and stay there
        :param time:
        :param height:
        :return:
        """
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        if type(time) != rospy.Duration:
            time = rospy.Duration.from_sec(time)

        hover_start_time = rospy.Time.now()

        start_pos = self.get_pos('odom')
        start_pos.pose.position.z = height

        # Always use the latest available information
        start_pos.header.stamp = rospy.Time(0)

        r = rospy.Rate(10)
        while rospy.Time.now() - hover_start_time < time:
            self.posPub.publish(start_pos)
            r.sleep()

    def get_pos(self, frame='map'):
        """
        Gets the position of the drone in the map (or relative to some other coordinate frame)

        :param frame: The world frame in which to return the result
        :return (PoseStamped): The position of the drone at the latest available time
        """
        time = self.tf.getLatestCommonTime(frame, self.FRAME_ID)
        position, quaternion = self.tf.lookupTransform(frame, self.FRAME_ID, time)

        return PoseStamped(
            header=Header(
                stamp=time,
                frame_id=frame
            ),
            pose=Pose(
                position=Point(
                    x=position[0], y=position[1], z=position[2]
                ),
                orientation=Quaternion(
                    x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
                )
            )
        )

    def distance_from(self, frame):
        """
        Returns the linear distance along the ground between the robot and the origin of the provided TF frame.
        TODO: Allow PoseStamped as imput, rather than just using the origin.
        :param (str) frame: The TF frame to measure to
        :return:
        """
        linear = self.get_pos(frame).pose.linear

        return math.sqrt(linear.x**2+linear.y**2)

    def onNavdata(self, msg):
        """
        :param (Navdata) msg:
        """
        self.navdata = msg

############ Old stuff ############

    def record_vel(self, msg):
        # TODO: Use Odometry topic for velocity information
        self.vel3d = msg

    def follow_roomba(self, roomba=None, des_x=0.0, des_y=0.0, des_z=None):
        """
        Follow roomba X with desired position des_x, des_y
        :param roomba: an roomba object
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if roomba is None:
            if self.current_target is None:
                return None
            roomba = self.current_target

        if des_z is not None:
            self.last_height = des_z

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = self.last_height
        pose_stamped.header.frame_id = roomba.frame_id

        self.posPub.publish(pose_stamped)

    def follow_roomba_global(self, des_x=0.0, des_y=0.0, des_z=None):
        """
        Follow roomba X with desired position des_x, des_y, ignore roomba's orientation
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """
        if self.current_target is None:
            return None

        position = self.position_of_roomba()

        if des_z is not None:
            self.last_height = position[2] + des_z

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = position[0] + des_x
        pose_stamped.pose.position.y = position[1] + des_y
        pose_stamped.pose.position.z = self.last_height
        pose_stamped.header.frame_id = "map"

        self.posPub.publish(pose_stamped)

    def move(self, des_x=0.0, des_y=0.0, des_z=None):
        """
        Tell the drone to move relative to itself, going to the height target if des_z
        is not specified
        :param des_x: desired position x
        :param des_y: desired position y
        :param des_z: desired position z
        """

        if des_z is None:
            des_z = self.last_height - self.height()

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = des_z
        pose_stamped.header.frame_id = self.FRAME_ID

        self.posPub.publish(pose_stamped)

    def change_height(self, height):
        """
        Change drone's height to height
        :param height: Desired height, in meters
        """
        self.last_height = height
        self.move()

    def stand_still(self):
        """
        Make our drone stand still
        NOTE: commands zero velocity, overriding vertical height setpoints
        :return: None
        """
        self.velPub.publish(Twist())

    def push_button(self):
        """
        Make our drone push button. Go together with a boolean should_hit_button
        should_hit_button is True, execute this method
        Otherwise: do nothing
        :return: Void
        """
        if self.distance_from_target() is None:
            print "No target to hit button"
            self.should_hit_button = False

        self.follow_roomba_global()
        if self.should_hit_button:
            # Hit the button
            self.change_height(self.ROOMBA_HEIGHT - 0.1)
            if self.distance_from_target() < 0.1:

                # 0.1 is roomba's height
                if self.height() <= self.ROOMBA_HEIGHT:
                    print "Button hit"
                    self.should_hit_button = False
                    self.change_height(self.NORMAL_HEIGHT)

    def land_front(self):
        """
        Make our drone land in front of the current target. Go together with a boolean should_land_front
        should_land_front is True, execute this method
        Otherwise: do nothing
        :return: Void
        """
        if self.distance_from_target() is None:
            print "No target to stand in front"
            self.should_land_front = False

        if self.should_land_front:
            # Land in front
            if self.vel3d.linear.x != 0 or self.vel3d.linear.y != 0:
                # We are moving
                self.follow_roomba_global(des_y=-0.6, des_z=0.0)

            elif self.height() <= 0.1:
                if self.distance_from_target() <= 0.35:
                    print "Roomba hit! Now,taking off"
                    self.should_land_front = False
                    self.follow_roomba_global(des_y=-0.6, des_z=0.0)
                elif self.distance_from_target() <= 0.8:
                    print "Landed in front, waiting for roomba"
                    self.stand_still()

    def distance_from_target(self):
        """
        Get the distance from the target relative to drone
        :return: Float: distance
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform(self.FRAME_ID, self.current_target.frame_id, rospy.Time(0))
        distance = math.sqrt(position[0] ** 2 + position[1] ** 2)
        return distance

    def height(self):
        position, quaternion = self.tf.lookupTransform('map', self.FRAME_ID, rospy.Time(0))
        return position[2]

    def position_of_roomba(self):
        """
        Gets the relative position of the current target to the drone
        :return: List[float]: relative coordinates
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        return position

    def target_facing_angle(self):
        """
        :return: Float: The angle the target is facing from -PI to PI
        :return: True: if target is rotating, False: otherwise
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        euler = tf.transformations.euler_from_quaternion(quaternion)
        is_rotating = euler[-1] != self.prev_target_facing_angle
        # print euler[-1] / pi * 180
        self.prev_target_facing_angle = euler[-1]
        return euler[-1], is_rotating

    def check_back_to_height(self):
        """
        Check if our drone is back to normal height
        :return: True: if already back to normal height, False: otherwise
        """
        if self.height() <= self.NORMAL_HEIGHT:
            return False
        # print "Back to normal height"
        return True

    def check_push_button(self):
        """
        Check if the target is facing to the right direction
        If the target is heading to a wrong direction, drone should push button
        :return: True: need to push button, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False
        return -pi / 3 <= angle <= pi / 3 or angle >= 2 * pi / 3 or angle <= -2 * pi / 3

    def check_land_front(self):
        """
        Check if the target is heading towards Red region, angle from -pi/4 to -3 pi/4
        If the target is heading to Red region, drone should land in front
        :return: True: need to land in front, False: otherwise
        """
        angle, is_rotating = self.target_facing_angle()
        if angle is None or is_rotating:
            return False

        return -2 * pi / 3 < angle < -pi / 3

    def decision_making(self):
        """
        Three decisions currently: push button -> land in front -> just follow
        :return: Void
        """
        if self.check_push_button():
            self.should_hit_button = True
        else:
            self.should_hit_button = False
            if self.check_land_front():
                self.should_land_front = True
            else:
                self.should_land_front = False

    """
    These methods are for testing purposes
    """

    def test_follow_roomba(self):
        self.test_change_height(3)
        r = rospy.Rate(10)

        while True:
            r.sleep()
            self.follow_roomba_global(des_x=0, des_y=0)

    def test_change_height(self, height=0.0):
        error = height - self.height()
        r = rospy.Rate(10)

        while abs(error) >= 0.5:
            r.sleep()
            print "error", error
            self.change_height(height)
            error = height - self.height()
        self.stand_still()

    def test_push_button(self):
        self.test_change_height(height=2.0)
        r = rospy.Rate(10)

        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()

            if not is_rotating or self.should_hit_button:
                if self.check_push_button():
                    self.should_hit_button = True
                else:
                    self.should_hit_button = False

                # Perform push_button if applicable
                if self.should_hit_button:
                    self.push_button()
                    self.follow_roomba_global()

                # Otherwise perform back_to_normal_height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)
            else:
                self.should_hit_button = False
                self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)

    def test_land_front_n_hit_btn(self):
        self.test_change_height(self.NORMAL_HEIGHT)
        r = rospy.Rate(10)

        while True:
            angle, is_rotating = self.target_facing_angle()
            r.sleep()

            if not is_rotating or self.should_hit_button:
                self.decision_making()

                # Perform push_button if applicable
                if self.should_hit_button:
                    self.push_button()

                # Perform land_in_front if applicable
                elif self.should_land_front:
                    self.land_front()

                # Otherwise perform back_to_normal_height
                else:
                    self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)
            else:
                self.should_hit_button = False
                self.should_land_front = False
                self.follow_roomba_global(des_x=0, des_y=-0.5, des_z=self.NORMAL_HEIGHT)


if __name__ == '__main__':
    rospy.init_node('Example')
    rospy.sleep(0.5)
    roomba = Roomba("target4")
    drone = Drone(target=roomba)

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    # drone.test_change_height(drone.NORMAL_HEIGHT)
    # drone.test_follow_roomba()
    # drone.test_push_button()
    drone.test_land_front_n_hit_btn()
