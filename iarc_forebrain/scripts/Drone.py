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

    def __init__(self, target=None, tf=None):
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

        if tf is None:
            self.tf = tf.TransformListener()
        else:
            self.tf = tf

        self.posPub = rospy.Publisher('/forebrain/cmd_pos', PoseStamped, queue_size=0)
        self.velPub = rospy.Publisher('/forebrain/cmd_vel', Twist, queue_size=0)
        self.takeoffPub = rospy.Publisher('/forebrain/cmd_takeoff', Empty, queue_size=0)
        self.landPub = rospy.Publisher('/forebrain/cmd_land', Empty, queue_size=0)

        rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(
            name='forebrain', fast=True)
        rospy.sleep(0.1)
        rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish(
            'forebrain')

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

    def takeoff(self, height=None, tol=0.2):
        """
        Commands the drone to takeoff from ground level, and blocks until it has done so.
        :param (float) height: Height in meters
        :param (float) tol: Tolerance to desired height to wait until reaching. Set to 0 to disable.
        :return: None
        """
        if self.last_height is None:
            self.last_height = self.NORMAL_HEIGHT
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        self.takeoffPub.publish(Empty())

        if tol != 0:
            r = rospy.Rate(10)
            while height - self.get_altitude() > tol:
                self.hover(time=0, height=height)
                r.sleep()

        self._remembers_flying = True

    def land(self, block=True):
        """
        Commands the drone to takeoff from ground level. Directly commands the low-level controls,
        and might need changes as hardware gets upgraded.
        :param (bool) block: Wait until vehicle reaches ground?
        :return: None
        """
        self.last_height = 0

        self.landPub.publish(Empty())

        if block:
            r = rospy.Rate(10)
            while self.get_altitude() > 0.2:
                self.landPub.publish(Empty())
                r.sleep()
            rospy.sleep(0.5)

        self._remembers_flying = False

    def hover(self, time=0, height=None):
        """
        Publishes 0 velocity for the given duration
        TODO: Store the starting position, and stay there
        :param (float | rospy.Duration) time: If nonzero, hovers for this amount of time.
        :param height:
        :return:
        """
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        if type(time) != rospy.Duration:
            time = rospy.Duration.from_sec(time)

        if time.to_sec() == 0:
            self.velPub.publish(Twist())
            return
        else:
            hover_start_time = rospy.Time.now()

            start_pos = self.get_pos('odom')
            start_pos.pose.position.z = height

            # Always use the latest available information
            start_pos.header.stamp = rospy.Time(0)

            r = rospy.Rate(10)
            self.posPub.publish(start_pos)
            while rospy.Time.now() - hover_start_time < time:
                r.sleep()
                self.posPub.publish(start_pos)

    def move_to(self, des_x=0.0, des_y=0.0, frame='map', height=None, tol=0.2):
        """
        Tells the drone to move to a specific position on the field, and blocks until the drone is
        within tol of the target, counting vertical and horizontal distance
        :param des_x:
        :param des_y:
        :param frame:
        :param height:
        :param tol:
        :return:
        """

        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            rel_pos = self.get_pos(frame).pose.position
            dist = math.sqrt(
                (rel_pos.x - des_x) ** 2 +
                (rel_pos.y - des_y) ** 2 +
                (rel_pos.z - height) ** 2)

            if dist <= tol:
                break

            self.move_towards(des_x, des_y, frame, height)

            r.sleep()

    def move_towards(self, des_x=0.0, des_y=0.0, frame='map', height=None):
        """
        Tells the drone to move towards a specific position on the field, then returns
        :param height: Flight altitude, meters. Defaults to previously commanded height.
        :param frame: The tf frame associated with the target
        :param des_x: desired position x
        :param des_y: desired position y
        """

        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        pose_stamped = PoseStamped()
        pose_stamped.pose.position.x = des_x
        pose_stamped.pose.position.y = des_y
        pose_stamped.pose.position.z = height
        pose_stamped.header.frame_id = frame

        self.posPub.publish(pose_stamped)

    def redirect_180(self, roomba, front_dist=1.0, rest_time=5.0, height=None):
        """
        Redirects the target rooba 180 degrees by landing in front of it.
        :param Roomba roomba: The target to redirect
        :param float front_dist: How far in front to land
        :param float rest_time: How long to sit on the ground(seconds)
        :param float height: How high to fly after taking off again
        :return bool: Success?
        """
        if height is None:
            height = self.last_height
        else:
            self.last_height = height

        self.move_to(front_dist, 0, roomba.frame_id, height=0.5)

        rospy.sleep(0.5)

        self.land()

        rospy.sleep(rest_time)

        self.takeoff(height)
        self.hover(0, height)

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

    def get_altitude(self):
        return self.get_pos(frame='odom').pose.position.z

    def distance_from(self, frame):
        """
        Returns the linear distance along the ground between the robot and the origin of the provided TF frame.
        TODO: Allow PoseStamped as imput, rather than just using the origin.
        :param (str) frame: The TF frame to measure to
        :return:
        """
        linear = self.get_pos(frame).pose.linear

        return math.sqrt(linear.x ** 2 + linear.y ** 2)

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

    def land_front_old(self):
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

    def position_of_roomba(self):
        """
        Gets the relative position of the current target to the drone
        :return: List[float]: relative coordinates
        """
        if self.current_target is None:
            return None

        position, quaternion = self.tf.lookupTransform("map", self.current_target.frame_id, rospy.Time(0))
        return position

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
