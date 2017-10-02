#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from iarc_arbiter.srv import *

import transformers

rospy.init_node('arbiter')


class Arbiter:
    def __init__(self):
        self.null_behavior = Behavior(self, namespace='', name='null', friendly_name='UndefinedBehavior')

        self.behaviors = [self.null_behavior]
        self.active_behavior = self.null_behavior
        self.choose_leader()

        # Transformers are functions capable of processing incoming data in a variety of formats.
        self.transformers = {
            'cmd_vel_raw': (Twist, transformers.cmd_vel_raw),
            'cmd_vel': (Twist, transformers.cmd_vel),
            'cmd_takeoff': (Empty, transformers.cmd_takeoff),
            'cmd_land': (Empty, transformers.cmd_land),
        }

        self.secondaries = []

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=10)

        self.status_pub = rospy.Publisher('status', String, queue_size=10)
        self.active_pub = rospy.Publisher('active_behavior', String, queue_size=10)

        rospy.Service('register', Register, self.handle_register)

    def handle_register(self, req):
        """
        :type req: RegisterRequest
        """
        print req
        if not req.namespace:
            rospy.logerr("Behavior cannot be created with empty namespace")
            return RegisterResponse()

        behavior = Behavior(self, req.namespace, req.name, req.pretty_name)
        self.behaviors.append(behavior)
        rospy.loginfo("Created behavior {}".format(behavior))

        return RegisterResponse(name=behavior.name)

    def subscribe_all(self):
        for b in self.behaviors:
            if b.namespace in (None, '', '/'):
                # This behavior is internal to the arbiter, and should not be subscribed.
                continue

            topics = rospy.get_published_topics(b.namespace)
            for t in topics:
                name = t[0][len(b.namespace):].strip('/')
                if name in self.transformers and name not in b.subscribers:
                    b.subscribe(name)

    def process_command(self, behavior, cmd):
        """
        process_command gets called after a message gets received from the currently active behavior.

        :param behavior: The behavior initiating the request
        :type cmd: transformers.Command
        """

        for func in self.secondaries:
            cmd = func(cmd)

        if cmd.takeoff:
            self.takeoff_pub.publish(Empty())
            self.vel_pub.publish(Twist())
        elif cmd.land:
            self.land_pub.publish(Empty())
            self.vel_pub.publish(Twist())
        else:
            self.vel_pub.publish(cmd.vel)

        self.active_pub.publish(String(behavior.name))
        rospy.loginfo_throttle(1, "Command published by {}".format(behavior.name))

    def choose_leader(self):
        # TODO: placeholder, make votes and stuff count
        self.active_behavior.is_leader = False
        self.active_behavior = self.behaviors[-1]
        self.active_behavior.is_leader = True

    def publish_status(self):
        self.status_pub.publish(String(str(self.behaviors)))

    def run(self):
        r = rospy.Rate(20)
        r_scan = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.choose_leader()
            self.publish_status()
            self.null_behavior.handle_message('cmd_vel', Twist())

            if r_scan.remaining() < rospy.Duration(0):
                self.subscribe_all()
                r_scan.sleep()

            r.sleep()


class Behavior:
    def __init__(self, arbiter, namespace, name=None, friendly_name=None, intrinsic_vote=0.0):
        """
        :param arbiter: The Arbiter to which this belongs
        :param namespace: If empty, the behavior is treated as being internal to the Arbiter.
        :param name: The name used to refer to this elsewhere in ROS
        :param friendly_name: The name displayed, in CapitalCamelCase
        :param intrinsic_vote: A "default" vote associated with this behavior
        """
        if namespace and (namespace[0] != '/'):
            namespace = '/' + namespace
        if not name:
            name = namespace.strip('/')
        if not friendly_name:
            friendly_name = name

        self.name = name
        self.arbiter = arbiter
        self.namespace = namespace
        self.friendly_name = friendly_name
        self.intrinsic_vote = intrinsic_vote
        self.subscribers = dict()

        self.is_leader = False
        self.last_msg_time = rospy.Time(0)

    def handle_message(self, topic, msg):
        msg_type, transformer = self.arbiter.transformers[topic]

        self.last_msg_time = rospy.Time.now()
        if self.is_leader:
            standardized = transformer(msg)
            self.arbiter.process_command(self, standardized)

    def subscribe(self, topic):
        if topic not in self.arbiter.transformers:
            rospy.logerr_throttle(1, "Unable to subscribe to topic {} for {}: unknown type".format(topic, self.name))
            return

        msg_type, transformer = self.arbiter.transformers[topic]

        def callback(msg):
            self.handle_message(topic, msg)

        sub = rospy.Subscriber("{}/{}".format(self.namespace, topic), msg_type, callback)
        self.subscribers[topic] = sub

        rospy.loginfo("Subscribed to {}/{}".format(self.namespace, topic))


if __name__ == '__main__':
    Arbiter().run()