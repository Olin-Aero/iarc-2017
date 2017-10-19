import rospy
from geometry_msgs.msg import Twist, PoseStamped
from iarc_arbiter.msg import RegisterBehavior
from std_msgs.msg import String


class Roomba(object):
    def __init__(self, frame_id=None):
        """
        This class will be re-defined later for everyone to use in predandstrat
        :param frame_id: id used in header of PoseStamped
        """
        self.frame_id = frame_id


def follow_roomba(roomba, des_x=0, des_y=0, des_z=0):
    """
    Follow roomba X with desired position des_x, des_y
    :param roomba: an roomba object
    :param des_x: desired position x
    :param des_y: desired position y
    :param aborted: if aborted, stop following
    :return:
    """
    test_msg = PoseStamped()
    test_msg.pose.position.x = des_x
    test_msg.pose.position.y = des_y
    test_msg.pose.position.z = des_z
    test_msg.header.frame_id = roomba.frame_id

    r = rospy.Rate(10)
    pub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        r.sleep()
        pub.publish(test_msg)


if __name__ == '__main__':
    # Init FollowBehavior node
    rospy.init_node('FollowBehavior')

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # TODO: For testing purposes only, will delete those lines of code below
    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    # Create a target object just for testing
    target = Roomba("target0")
    follow_roomba(roomba=target, des_x=0, des_y=0)
