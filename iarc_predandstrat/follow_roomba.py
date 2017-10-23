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


def follow_roomba(roomba, des_x=0, des_y=0, des_z=0, pub=None):
    """
    Follow roomba X with desired position des_x, des_y
    :param roomba: an roomba object
    :param des_x: desired position x
    :param des_y: desired position y
    :param des_z: desired position z
    """

    pose_stamped = PoseStamped()
    pose_stamped.pose.position.x = des_x
    pose_stamped.pose.position.y = des_y
    pose_stamped.pose.position.z = des_z
    pose_stamped.header.frame_id = roomba.frame_id
    if pub is not None:
        pub.publish(pose_stamped)


if __name__ == '__main__':
    # Create a target object just for testing
    target = Roomba("target0")

    # Init FollowBehavior node
    rospy.init_node('FollowBehavior')

    # Register this behavior (FollowBehavior) to the arbiter
    rospy.Publisher('/arbiter/register', RegisterBehavior, latch=True, queue_size=10).publish(name='follow')

    # This command tell the arbiter that this behavior will take command from now on and abort other behaviors
    rospy.Publisher('/arbiter/activate_behavior', String, latch=True, queue_size=10).publish('follow')

    # Create a follow publisher
    pub = rospy.Publisher('/follow/cmd_pos', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        follow_roomba(roomba=target, des_x=0, des_y=0, pub=pub)
