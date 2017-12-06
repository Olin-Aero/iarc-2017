#!/usr/bin/env python2
import rospy
import transformers
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Empty, String
from iarc_arbiter.msg import RegisterBehavior

from tf import TransformListener
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class Arbiter:
    """
    The Arbiter is a mutiplexer that reads cmd_* topics from several namespaces, converts them into
    standard cmd_vel form, and forwards one of them into the global namespace.

    It receives information about which behavior (namespace) should be selected from String messages
    on the /arbiter/activate_behavior topic. This information comes from the planning stack.
    In the future, a voting system could replace this mechanism.
    The special value "zero" represents an internal behavior that stops the vehicle.

    It also publishes the name of the active behavior to arbiter/active_behavior.
    """

    def __init__(self):
        self.null_behavior = Behavior(self.process_command, 'zero')

        self.behaviors = {'zero': self.null_behavior}
        self.active_behavior_name = 'zero'
        self.set_active_behavior('zero')

        self.tf = TransformListener()
        self.ddynrec = DDynamicReconfigure("example_dyn_rec")

        # Transformers are functions capable of processing incoming data in a variety of formats.
        # They are functions that take input of whatever type the topic is, and produce a transformers.Command
        # object.

        pid = transformers.PIDController(self.tf, self.ddynrec)
        print pid.cmd_pos
        self.transformers = {
            'cmd_vel': (Twist, transformers.cmd_vel),
            'cmd_takeoff': (Empty, transformers.cmd_takeoff),
            'cmd_land': (Empty, transformers.cmd_land),
            'cmd_pos': (PoseStamped, pid.cmd_pos),
        }
        """:type : dict[str, (str, (Any) -> transformers.Command)]"""

        # Subscribe to the behaviors passed as ROS parameters
        starting_behaviors = rospy.get_param('~behaviors', [])
        for b in starting_behaviors:
            behavior = Behavior(self.process_command, b)
            behavior.subscribe(self.transformers)
            self.behaviors[b] = behavior

        # Secondary behaviors are filters that are always active on the Command before it is published.
        # Examples include last-minute obstacle avoidance, speed limiters, or arena boundary constraints.
        self.secondaries = []

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=0)
        self.takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=0)
        self.land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=0)

        self.debug_pub = rospy.Publisher('/arbiter/debug', String, queue_size=10)
        self.active_pub = rospy.Publisher('/arbiter/active_behavior', String, queue_size=10)

        rospy.Subscriber('/arbiter/register', RegisterBehavior, self.handle_register)

        rospy.Subscriber('/arbiter/activate_behavior', String, self.handle_activate)

        self.start_ddynrec()

        rospy.sleep(0.5)

    def start_ddynrec(self):
        """
        Helper function to start the ddynamic reconfigure server with a callback
        function that updates the self.ddynrec attribute.
        """
        def callback(config, level):
            """
            A callback function used to as the parameter in the ddynrec.start() function.
            This custom callback function updates the state of self.ddynrec so we can
            refer to its variables whenever we have access to it. 
            """
            rospy.loginfo("Received reconf call: " + str(config))
            # Update all variables
            var_names = self.ddynrec.get_variable_names()
            for var_name in var_names:
                self.ddynrec.__dict__[var_name] = config[var_name]
            return config

        self.ddynrec.start(callback)

    def handle_activate(self, msg):
        """
        ROS subscriber for the activate_behavior topic
        :type msg: String
        """
        self.set_active_behavior(msg.data)

    def set_active_behavior(self, name):
        """
        Sets the active behavior, if the provided name is one of the known behaviors.
        :type name: str
        """
        if name not in self.behaviors:
            # TODO: consider automatically registering new behaviors
            rospy.logerr('{} does not exist as a behavior!'.format(name))
            self.set_active_behavior('zero')

        if name != self.active_behavior_name:
            self.active_behavior_name = name
            # Stop the vehicle
            self.process_command(name, 'cmd_vel', Twist())
        rospy.loginfo_throttle(1.0, '{} selected as active behavior'.format(name))

    def handle_register(self, req):
        """
        ROS subscriber for adding a new behavior to the system.
        It is recommended to publish a single latched message to this behavior when a behavior
        node starts.

        :type req: RegisterBehavior
        """
        if req.name in self.behaviors:
            rospy.logerr("Behavior {} already exists".format(req.name))

        if not req.name:
            rospy.logerr("Behavior cannot be created with empty name")

        behavior = Behavior(self.process_command, req.name)
        behavior.subscribe(self.transformers)
        self.behaviors[req.name] = behavior
        rospy.loginfo("Created behavior {}".format(behavior))

    def process_command(self, behavior, topic, raw_cmd):
        """
        process_command gets called after a message gets received from the currently active behavior.

        :param str behavior: The name of the behavior initiating the command
        :param str topic: The topic (without namespace) to which the command was sent
        :type raw_cmd: ROS message
        :return: success
        """
        if behavior != self.active_behavior_name:
            # Only messages from the active behavior are handled
            return False

        _, transformer = self.transformers[topic]

        # Convert to a transformers.Command
        cmd = transformer(raw_cmd)  # type: transformers.Command

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

    def publish_debug(self):
        """
        Publishes debug information to the ROS network

        :return: None
        """
        self.debug_pub.publish(String(str(self.behaviors)))

    def run(self):
        """
        Main method, publishes debug information and
        :return:
        """
        r = rospy.Rate(20)

        while not rospy.is_shutdown():
            self.publish_debug()
            self.null_behavior.handle_message('cmd_vel', Twist())

            r.sleep()


class Behavior:
    def __init__(self, callback, name):
        """
        :param (str, str, Any)->bool callback: The function to be called when this behavior receives a command
        :param name: The name used to refer to this elsewhere in ROS
        """

        self.name = name
        self.callback = callback
        self.subscribers = dict()

        self.last_msg_time = rospy.Time(0)

    def handle_message(self, topic, msg):
        """
        Processes an incoming message from the namespace of this behavior, ultimately calling the
        callback function provided when this behavior was created.

        :param str topic: The topic (without namespace) to which the message was sent
        :param msg: The message (various ROS message types)
        :return: None
        """
        self.last_msg_time = rospy.Time.now()
        self.callback(self.name, topic, msg)

    def subscribe(self, topics):
        """
        Subscribes to the topics specified by transformers from the namespace.

        :param topics: map{topic name : (Type, Transformer)
        :type topics: dict[str, (str, (Any) -> transformers.Command)]
        :return:
        """

        for (topic, (msg_type, _)) in topics.iteritems():
            # The selftopic=topic part makes a copy of the topic variable, so it doesn't get changed before
            # the callback is called.
            # https://stackoverflow.com/a/235764
            def callback(msg, selftopic=topic):
                self.handle_message(selftopic, msg)

            sub = rospy.Subscriber("/{}/{}".format(self.name, topic), msg_type, callback)
            self.subscribers[topic] = sub

            rospy.loginfo("Subscribed to /{}/{}".format(self.name, topic))

    def __str__(self):
        return 'Behavior[{}]'.format(self.name)

    __repr__ = __str__


if __name__ == '__main__':
    rospy.init_node('arbiter')
    Arbiter().run()
