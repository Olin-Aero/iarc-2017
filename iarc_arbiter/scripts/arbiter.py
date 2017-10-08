#!/usr/bin/env python2
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String
from iarc_arbiter.srv import *

import transformers

rospy.init_node('arbiter')


class Arbiter:
    def __init__(self):
        self.null_behavior = Behavior(self.process_command, namespace='', name='zero', friendly_name='StopBehavior')

        self.behaviors = {'zero': self.null_behavior}
        self.active_behavior_name = ''
        self.set_active_behavior('zero')

        # Transformers are functions capable of processing incoming data in a variety of formats.
        self.transformers = {
            'cmd_vel_raw': (Twist, transformers.cmd_vel_raw),
            'cmd_vel': (Twist, transformers.cmd_vel),
            'cmd_takeoff': (Empty, transformers.cmd_takeoff),
            'cmd_land': (Empty, transformers.cmd_land),
        }

        self.secondaries = []

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=0)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=0)

        self.status_pub = rospy.Publisher('debug', String, queue_size=10)
        self.active_pub = rospy.Publisher('active_behavior', String, queue_size=10)

        rospy.Service('register', Register, self.handle_register)

        rospy.Subscriber('activate', String, self.handle_activate)

    def handle_activate(self, msg):
        self.set_active_behavior(msg.data)

    def set_active_behavior(self, name):
        self.active_behavior_name = name
        rospy.loginfo_throttle(1.0, '{} selected as active behavior'.format(name))

    def handle_register(self, req):
        """
        :type req: RegisterRequest
        """
        print req
        if not req.namespace:
            rospy.logerr("Behavior cannot be created with empty namespace")
            return RegisterResponse()

        behavior = Behavior(self.process_command, req.namespace, req.name, req.pretty_name)
        behavior.subscribe(self.transformers)
        self.behaviors[req.name] = behavior
        rospy.loginfo("Created behavior {}".format(behavior))

        return RegisterResponse(name=behavior.name)

    def process_command(self, behavior, topic, raw_cmd):
        """
        process_command gets called after a message gets received from the currently active behavior.

        :param (str) behavior: The name of the behavior initiating the command
        :param (str) topic: The topic (without namespace) to which the command was sent
        :type raw_cmd: ROS message
        """
        if behavior != self.active_behavior_name:
            return False

        _, transformer = self.transformers[topic]

        # Convert to a transformers.Command
        cmd = transformer(raw_cmd)

        # Apply secondary behaviors
        for func in self.secondaries:
            cmd = func(cmd)

        # Publish the result to the ROS network
        if cmd.takeoff:
            self.takeoff_pub.publish(Empty())
            self.vel_pub.publish(Twist())
        elif cmd.land:
            self.land_pub.publish(Empty())
            self.vel_pub.publish(Twist())
        else:
            self.vel_pub.publish(cmd.vel)

        # Log it
        self.active_pub.publish(String(behavior))
        rospy.loginfo_throttle(1, "Command published by {}".format(behavior))

        return True

    def publish_status(self):
        self.status_pub.publish(String(str(self.behaviors)))

    def run(self):
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.publish_status()
            self.null_behavior.handle_message('cmd_vel', Twist())

            r.sleep()


class Behavior:
    def __init__(self, callback, namespace, name=None, friendly_name=None):
        """
        :param callback: A function that takes this behavior and the standardized message as arguments
        :param namespace: If empty, the behavior is treated as being internal to the Arbiter.
        :param name: The name used to refer to this elsewhere in ROS
        :param friendly_name: The name displayed, in CapitalCamelCase
        """
        if namespace and (namespace[0] != '/'):
            namespace = '/' + namespace

        if not name:
            name = namespace.strip('/')

        if not friendly_name:
            friendly_name = name

        self.name = name
        self.callback = callback
        self.namespace = namespace
        self.friendly_name = friendly_name
        self.subscribers = dict()

        self.last_msg_time = rospy.Time(0)

    def handle_message(self, topic, msg):
            self.last_msg_time = rospy.Time.now()
            self.callback(self.name, topic, msg)

    def subscribe(self, transformers):
        """
        Subscribes to the topics specified by transformers from the namespace.
        :param transformers: map{topic name : (Type, Transformer)
        :return:
        """
        if not self.namespace:
            return

        for (topic, (msg_type, _)) in transformers.iteritems():

            def callback(msg):
                self.handle_message(topic, msg)

            sub = rospy.Subscriber("{}/{}".format(self.namespace, topic), msg_type, callback)
            self.subscribers[topic] = sub

            rospy.loginfo("Subscribed to {}/{}".format(self.namespace, topic))

    def __str__(self):
        return 'Behavior["{}", id={} namespace="{}"]'.format(self.friendly_name, self.name, self.namespace)


if __name__ == '__main__':
    Arbiter().run()
