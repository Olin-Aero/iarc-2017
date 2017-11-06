"""
Heatmap.py
Written by Lydia Zuehsow (Oktober13) c. Fall 2017

This code contains simulation code that attempts to predict the location of Roombas over time.
There are two main types of simulators:
longSim- calculates long term frequency of square traversals, for a holistic view of which squares are most likely to have a Roomba in them at each moment.
		This accounts for collisions, and takes a longer time to calculate. It's really intended as an aid to overall strategy.
fiveSecSim- calculates less detailed short term estimates of Roomba location. Does not handle collisions. 
		This is intended for a realtime constantly-running Roomba position prediction.
"""

import math
import random
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from scipy.misc import imread
from roomba_classes import TargetRoomba
from collections import Counter
# import config as cfg # Need to figure out relative imports

class SimRoomba(object):
	def __init__(self,targets=0,obstacles=0):
		self.t = TargetRoomba([0,0], (math.pi/2.000), 'Bob') # Creates TargetRoomba object (See robots.py)
		self.t.start()
		# t.prev_pos = []
		self.t.curr_pos = (0,0)
	def updatePos(self,time_passed, time_since=1):
		try:
			self.t.update(time_since, time_passed*1000)
			self.t.curr_pos = (int(math.floor(self.t.pos[0])),int(math.floor(self.t.pos[1])))
			print time_passed, (math.floor(self.t.pos[0]),math.floor(self.t.pos[1])), self.t.curr_pos
			# self.t.roomba_pos.append((int(math.floor(t.pos[0])),int(math.floor(t.pos[1]))))
			# self.t.curr_pos = (x,y)
			# self.t.prev_pos.append(t.curr_pos)
			return True
		except:
			return False
	def getPos(self):
		return self.t.curr_pos

# class LocationSim(object):
# 	def __init__(sekd)

def plotStuff(loc_data):
	# cluster = {{(4, -2): 8, (0, -2): 8, (-3, -1): 6} # Useful fake data for testing things
	# print loc_data.values() 

	fig = plt.figure()

	background = imread("grid.jpg")

	for (coords, numtimes) in loc_data.items():
		(x,y) = coords
		color = (float(numtimes)/len(loc_data.items()))*255.
		# print color

		# Currently trying to debug this- Color is displaying as all red, but should be a variety of shades based on that set of coordinates frequency of recurrence.

		plt.scatter(x, y, c = color, cmap = 'gist_rainbow')
	plt.imshow(background,zorder=0,extent=[-10.0, 10.0, -10.0, 10.0])

	plt.show()

def fiveSecSim(passed_time_since, passed_prev_loc, passed_prev_orient):
	"""
	This simulation is for short term rapid location estimation of a Roomba that hasn't been seen for a while.
	20 hypothetical "Imaginary Roombas" trace out paths it could have taken, and the centroid of the final scatter determines the hypothesized final position of the actual Roomba.
	times_since = Array. Time since the Roomba was last seen.
	"""
	roomba_pos = [(0,0)] * 20

	# try:
	time_since = passed_time_since
	prev_loc = passed_prev_loc
	prev_orient = passed_prev_orient

	time_to_noise = time_since % 5 # Check time remaining until Roomba experiences noise
	noise_cycles = int(math.floor(time_since / 5.0)) # Check number of times Roombas should go through "noise"

	time_to_turn = time_since % 20 # Check time remaining until Roomba turns
	turn_cycles = math.floor(time_since / 20.0) # Check number of times Roombas should go through "turn cycles"

	for roomba in range(0,20): # For one of 20 hypothetical roombas...
		if noise_cycles <= 0:
			roomba_pos[roomba] = locCalc(prev_loc,time_to_noise,prev_orient)
		else:
			for cycles in range(1,noise_cycles+1):
				# print("Hoi")
				# print roomba, cycles
				randnum = prev_orient + random.randint(math.floor(-(math.pi / 18)),math.floor((math.pi / 18)))
				prev_orient = randnum
				roomba_pos[roomba] = locCalc(roomba_pos[roomba],(time_since-time_to_noise),randnum) # Need to bugfix here- Having issue with angle again
			roomba_pos[roomba] = locCalc(roomba_pos[roomba],time_to_noise,prev_orient)
	
	# print roomba_pos
	return roomba_pos
	# except:
	# 	print("Short term simulation failed")
	# 	return roomba_pos

def locCalc(passed_prev_loc, passed_time_moving, passed_prev_orient, passed_roomba_vel = 0.33):
	"""
	current position = previous position + velocity * time
	Helper function. Calculates new position given previous location, time spent moving, orientation, and velocity. Units are in meters.
	"""
	(past_x,past_y) = passed_prev_loc
	curr_x = past_x + passed_time_moving * passed_roomba_vel * math.cos(passed_prev_orient)
	curr_y = past_y + passed_time_moving * passed_roomba_vel * math.sin(passed_prev_orient)

	return (curr_x,curr_y)

def longSim(passed_num_targets,passed_num_obstacles):
	"""
	This simulation is for long term frequency distribution type calculations, to determine the most commonly traversed squares for all Roombas.
	It's mapped to the rainbow colormap, so reds are least common, and violets are most common intersections.
	"""
	roomba_pos = []

	bob = SimRoomba(passed_num_targets,passed_num_obstacles)

	for timer in range(0,100):
		if bob.updatePos(timer):
			roomba_pos.append(bob.getPos())
		else:
			print("Long term simulation failed")
			return roomba_pos
	return roomba_pos 

def chooseSimType():
	pass

def main(): # Needs to be cleaned up a bit
	# num_targets = 1
	# num_obstacles = 0
	# roomba_pos = longSim(num_targets,num_obstacles)


	time_since = 5 # Seconds
	prev_loc = (0,0) 
	prev_orient = math.pi / 2.0000
	roomba_pos = fiveSecSim(time_since,prev_loc,prev_orient)

	# print roomba_pos
	tc = Counter(roomba_pos) # Dictionary of Roomba positions
	print(tc)

	# heatmap = [[0 for col in range(-10,10)] for row in range(-10,10)] # Not currently used- an array with frequency values for roomba traversal in every square.
	# for key in tc:
	# 	heatmap[key[0]][key[1]] = heatmap[key[0]][key[1]] + 1
	# # print heatmap

	plotStuff(tc)

if __name__ == '__main__':
	main()