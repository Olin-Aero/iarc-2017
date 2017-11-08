# from transitions import Machine
from transitions.extensions import HierarchicalGraphMachine as Machine
from geometry_msgs.msg import Twist, Pose2D, Point
from std_msgs.msg import Float64
from iarc_sim_2d.msg import Roomba, Roombas

# from pygraphviz import *
import time
import pdb
import sys
import os
import rospkg
import rospy
import numpy as np
rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))
import config as cfg

class Drone(object):

	def __init__(self, C1=1, C2=1, C3=1, C4=1, C5=1, MIN_OBSTACLE_DISTANCE=1.5):
		"""
		Initialize the Drone object where:
		pos3d is a vector [x,y,z]
		vel3d is a vector [x',y',z']
		heading is an angle in radians (0 is +x and pi/2 is +y)
		"""

		rospy.Subscriber('/Vis_Roombas', Roombas, self.recordVisible)
		rospy.Subscriber('/drone/height', Float64, self.recordHeight)
		self.vel3d = Twist()
		self.visibleRoombas = []
		self.visibleObstacles = []

		self.C1 = C1
		self.C2 = C2
		self.C3 = C3
		self.C4 = C4
		self.C5 = C5
		self.MIN_OBSTACLE_DISTANCE = MIN_OBSTACLE_DISTANCE

	def recordVisible(self, msg):
		self.visibleRoombas = filter(lambda x: 'target' in x.tag, msg.roombas)
		self.visibleObstacles =  filter(lambda x: 'obstacle' in x.tag, msg.roombas)


	def recordHeight(self, msg):
		self.height = msg

	def getPose(self):
		self.drone_pos, self.drone_heading = self.tf.lookupTransform(
				'map', '%s'%drone.tag, rospy.Time(0)
				)
	def goodnessScore(self):
		"""
		Determines which Roomba we pick to lead to the goal.
		Higher score is better.

		Returns: [(Roomba, Score)]
		"""

		def headingScore(roomba):
			return np.sin(roomba.heading)

		def positionScore(roomba):
			return roomba.y

		def distanceFromObstaclesScore(roomba, obstacles):
			"""
			(-infinity, 0)
			"""

			score = 0
			for obstacle in obstacles:
				x = roomba.x - obstacle.x
				y = roomba.y - obstacle.y
				dist = np.sqrt(x**2 + y**2)

				if dist < MIN_OBSTACLE_DISTANCE:
					return -math.inf

				score -= 1/dist**2

			return score



		def stateQualtityScore(roomba):
			"""
			How precisely we know the Roombas' state.
			Compare position accuracy to view radius to know if it's possible
			to see the given roomba when drone arrives.
			"""
			return 0

		def futureGoodnessScore(roomba):
			return 0

		result = []

		for i in xrange(0,len(self.visibleRoombas)):
			roomba = self.visibleRoombas[i]
			score = self.C1*headingScore(roomba) + \
				self.C2*positionScore(roomba) + \
				self.C3*distanceFromObstaclesScore(roomba, self.visibleObstacles) + \
				self.C4*stateQualtityScore(roomba) + \
				self.C5*futureGoodnessScore(roomba)
			result.append((roomba, score))

		return result

	def targetSelect(self, roombaScore):
		print(roombaScore)
		if roombaScore is not []:
			return max(roombaScore, key=lambda x: x[1])
		else:
			return []



class Target(object):
	pass

class Obstacle(object):
	pass

class StratModel(object):
	def __init__(self):
		self.tf = tf.TransformListener()


rospy.init_node('strategy')

drone = Drone()
while True:
	time.sleep(.1)
	print('-'*80)
	for roomba, score in drone.goodnessScore():
		print('score: %f roomba: %s'%(score, roomba.tag))
	bestRoomba, bestRoombaScore = drone.targetSelect(drone.goodnessScore())
	print('Best Score: %f Best Roomba: %s'%(bestRoombaScore, bestRoomba.tag))

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