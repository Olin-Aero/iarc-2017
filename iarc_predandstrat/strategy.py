# from transitions import Machine
from transitions.extensions import HierarchicalGraphMachine as Machine
from geometry_msgs.msg import Twist, Pose2D, Point
from std_msgs.msg import Float64

# from pygraphviz import *

import sys
import os
import rospkg
import rospy
rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))
import config as cfg


class Drone(object):
    def __init__(self):
        """
        Initialize the Drone object where:
        pos3d is a vector [x,y,z]
        vel3d is a vector [x',y',z']
        heading is an angle in radians (0 is +x and pi/2 is +y)
        """

        self.vel3d = Twist()
        rospy.Subscriber('/cmd_vel', Twist, self.recordVisible)
        rospy.Subscriber('/drone/height', Float64, self.recordHeight)
        # rospy.Subscriber('/drone/visibleRoombs', Roombas, self.recordVisible)
        # self.pos3d = [pos2d[0], pos2d[1], initial_height]
        # self.heading = heading
        # self.tag = tag
        # self.visible_roombas = []

    def recordVisible(self, msg):
    	self.visibleRoombs = msg

    def recordHeight(self, msg):
    	self.height = msg

    def getPose(self):
    	self.drone_pos, self.drone_heading = self.tf.lookupTransform(
                'map', '%s'%drone.tag, rospy.Time(0)
                )


class Target(object):
	pass

class Obstacle(object):
	pass

class StratModel(object):
	def __init__(self):
		self.tf = tf.TransformListener()


drone = Drone()


states = [
		'init',
		'search', 
		'follow', 
		'waitForValidRoomba', #If a roomba is not availible to contact
		'avoidObstacle',
		'landInFront',
		'pushButton',
		'waitForGoodRoomba']

transitions = [
{ 'trigger': 'start', 'source': 'init', 'dest': 'search' },
{ 'trigger': 'goodRoombaFound', 'source': 'search', 'dest': 'follow' },
{ 'trigger': 'timeInvalid', 'source': 'follow', 'dest': 'waitForValidRoomba' }, #timeInvalid signals that it's not the right time to follow the roomba.

#Null actions result in returning to search algorithm
{ 'trigger': 'null', 'source': 'follow', 'dest': 'search' },
{ 'trigger': 'null', 'source': 'waitForValidRoomba', 'dest': 'search' },
{ 'trigger': 'null', 'source': 'waitForGoodRoomba', 'dest': 'search' },
{ 'trigger': 'null', 'source': 'pushButton', 'dest': 'search' },
{ 'trigger': 'null', 'source': 'landInFront', 'dest': 'search' },

#Action triggers
{ 'trigger': '180Needed', 'source': 'follow', 'dest': 'landInFront' },
{ 'trigger': '45Needed', 'source': 'follow', 'dest': 'pushButton' },
{ 'trigger': 'timeElapsedNeeded', 'source': 'follow', 'dest': 'waitForGoodRoomba' },

{ 'trigger': 'actionCompleted', 'source': 'landInFront', 'dest': 'follow' },
{ 'trigger': 'actionCompleted', 'source': 'pushButton', 'dest': 'follow' },
{ 'trigger': 'actionCompleted', 'source': 'waitForGoodRoomba', 'dest': 'follow' },

#Obstacle triggers
{ 'trigger': 'obstacleInPath', 'source': 'search', 'dest': 'avoidObstacle' },
{ 'trigger': 'obstacleInPath', 'source': 'follow', 'dest': 'avoidObstacle' },
{ 'trigger': 'obstacleInPath', 'source': 'waitForValidRoomba', 'dest': 'avoidObstacle' },
]

machine = Machine(model=drone, states=states,transitions=transitions, initial='init')

drone.get_graph().draw('state_diagram.png', prog='dot')

drone.start()
print(drone.state)


print(cfg.ROOMBA_HEIGHT)
drone.goodRoombaFound()

print(drone.state)