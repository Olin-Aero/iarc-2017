import math
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import numpy as np
from scipy.misc import imread
from roomba_classes import TargetRoomba
from collections import Counter

class SimRoomba(object):
	def __init__(self,targets=0,obstacles=0):
		self.t = TargetRoomba([0,0], 0, 'Bob') # Creates TargetRoomba object (See robots.py)
		self.t.start()
		# t.prev_pos = []
		self.t.curr_pos = (0,0)
	def updatePos(self,time_passed, time_since=1):
		try:
			self.t.update(time_since, time_passed*1000)
			self.t.curr_pos = (int(math.floor(self.t.pos[0])),int(math.floor(self.t.pos[1])))
			# self.t.roomba_pos.append((int(math.floor(t.pos[0])),int(math.floor(t.pos[1]))))
			# self.t.curr_pos = (x,y)
			# self.t.prev_pos.append(t.curr_pos)
			return True
		except:
			return False
	def getPos(self):
		return self.t.curr_pos


def plotStuff(loc_data):
	# cluster = {{(4, -2): 8, (0, -2): 8, (-3, -1): 6} # Useful fake data for testing things
	# print loc_data.values() 

	fig = plt.figure()

	background = imread("grid.jpg")

	for (coords, numtimes) in loc_data.items():
		(x,y) = coords
		color = (float(numtimes)/len(loc_data.items()))*255.
		print color

		# Currently trying to debug this- Color is displaying as all red, but should be a variety of shades based on that set of coordinates frequency of recurrence.

		plt.scatter(x, y, c = color, cmap = 'gist_rainbow')
	plt.imshow(background,zorder=0,extent=[-10.0, 10.0, -10.0, 10.0])

	plt.show()

def main(): # Needs to be cleaned up a bit
	num_targets = 1
	num_obstacles = 0
	roomba_pos = []
	bob = SimRoomba(num_targets,num_obstacles)

	for timer in range(0,100):
		if bob.updatePos(timer):
			roomba_pos.append(bob.getPos())
		else:
			print("Herp derp")

	tc = Counter(roomba_pos)
	print(tc)

	heatmap = [[0 for col in range(-10,10)] for row in range(-10,10)] # Not currently used- an array with frequency values for roomba traversal in every square.
	for key in tc:
		heatmap[key[0]][key[1]] = heatmap[key[0]][key[1]] + 1
	# print heatmap

	plotStuff(tc)

if __name__ == '__main__':
	main()