#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import numpy as np
import math
import random
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Vector3
import sys
import matplotlib.ticker as ticker
import os
import rospkg
rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))

from config import *
class HeatMap:
    def __init__(self):
        #self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #self.cmdHeight = rospy.Publisher('/cmd_height', Float64, queue_size=10)
        self.tf = tf.TransformListener()
        self.simulationTime = 0
        rospy.init_node('example')
        rospy.Subscriber('clock',Float64,self.callback)
        #rospy.sleep(0.1)
    # def listener(self):

    #     # In ROS, nodes are uniquely named. If two nodes with the same
    #     # node are launched, the previous one is kicked off. The
    #     # anonymous=True flag means that rospy will choose a unique
    #     # name for our 'listener' node so that multiple listeners can
    #     # run simultaneously.
    #     rospy.init_node('listener', anonymous=True)
    #     rospy.Subscriber('clock',Float64,self.callback)
    def callback(self,msg):
        self.simulationTime = msg.clock.nsecs * 10**-9
    # def createHeatMap2(self,time):
    #     #ROOMBA_LINEAR_SPEED = 0.33
    #     #ROOMBA_OBSTACLE_NOISE_MAX = 23 * (np.pi / 180)
    #     xPos = 2
    #     yPos = 1
    #     multiplyingFactor = math.tan(ROOMBA_OBSTACLE_NOISE_MAX / 2) * ROOMBA_LINEAR_SPEED
    #     heading = math.pi / 4
    #     Theta = heading - math.pi / 2
    #     CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta))
    #     CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
    #     newTheta = Theta + time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
    #     xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta))
    #     yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
    #     ellipseRad1 = multiplyingFactor * time
    #     positionsX = []
    #     positionsY = []
    #     total = 0
    #     self.mappingthing = [[0.0 for _ in range(40)] for _ in range(40)]
    #     for i in range(100):
    #         adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1.0 / 2.0 * ((i-50.0) * 3 / 50)**2))
    #         #adding = 1.0
    #         total += adding
    #     for i in range(100):
    #         tempx = xPosNew + (i-50) * math.sin(newTheta) * ellipseRad1 /  50
    #         tempy = yPosNew + (i-50) * math.cos(newTheta) * ellipseRad1 / 50
    #         if(tempx > -10 and tempx < 10 and tempy > -10 and tempy < 10):
    #             adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1. / 2.0 * ((i-50.0) * 3 / 50)**2.0))
    #             #adding = 1.0
    #             #print(tempy)
    #             #print(adding / total)
    #             self.mappingthing[int((tempx+10) * 2)][int((tempy+10) * 2)] += adding / total
    #     print(self.mappingthing)
    #     #return mappingthing
    # def createHeatMap(self,time,roomba):
    #     #ROOMBA_LINEAR_SPEED = 0.33
    #     #ROOMBA_OBSTACLE_NOISE_MAX = 23 * (np.pi / 180)
    #     sim_time = self.simulationTime
    #     roombaPos,roombaHeading = self.tf.lookupTransform('map', roomba, rospy.Time(0))
    #     xPos = roombaPos[0]
    #     yPos = roombaPos[1]
    #     multiplyingFactor = math.tan(ROOMBA_OBSTACLE_NOISE_MAX / 2) * ROOMBA_LINEAR_SPEED
    #     heading = tf.transformations.euler_from_quaternion(roombaHeading)[-1]
    #     Theta = heading - math.pi / 2
    #     CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta))
    #     CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
    #     newTheta = Theta + time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
    #     xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta))
    #     yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
    #     newHeading = newTheta + math.pi / 2
    #     ellipseRad1 = multiplyingFactor * time
    #     xBox = int(xPosNew * 2) / 2.0
    #     yBox = int(yPosNew * 2) / 2.0
    #     positionsX = []
    #     positionsY = []
    #     confidence = 0.0
    #     total = 0
    #     for i in range(1000):
    #         adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1.0 / 2.0 * ((i-500.0) * 3 / 500)**2))
    #         total += adding
    #     for i in range(1000):
    #         tempx = xPosNew + (i-500) * math.sin(newTheta) * ellipseRad1 /  500
    #         tempy = yPosNew + (i-500) * math.cos(newTheta) * ellipseRad1 / 500
    #         if((tempx-xPosNew) ** 2 + (tempy-yPosNew)**2 < 0.25**2):
    #             adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1. / 2.0 * ((i-500.0) * 3 / 500)**2.0))
    #             confidence += adding / total
    #     print((xPosNew,yPosNew,confidence))
    def heatmapping(self,prediction,num_roombas,roomba):
        sim_time = self.simulationTime
        print(sim_time)
        pred_time = sim_time + prediction
        roombaPos,roombaHeading = self.tf.lookupTransform('map', roomba, rospy.Time(0)) # Get initial conditions
        initX = roombaPos[0] # Store initial X position 
        initY = roombaPos[1] # Store initial Y position
        initHeading = tf.transformations.euler_from_quaternion(roombaHeading)[-1] # Store initial heading
        xArray = [] # Initialize for course
        yArray = [] # Initialize for course
        finalPointsX = [] # Initialize for final position
        finalPointsY = [] # Initialize for final position
        finalHeading = []
        for i in range(num_roombas): # for loop that controls number of iterations of simulation
            current_time = sim_time # current_time is the time in the current run through of the simulation, sim_time is the time in the 2d sim
            xPosNew = initX # storing initial values
            yPosNew = initY
            newHeading = initHeading
            while(current_time < sim_time + pred_time): # While the time in the current run is less than the time we want to be predicted
                new_time = ROOMBA_OBSTACLE_NOISE_PERIOD/1000 - sim_time%(ROOMBA_OBSTACLE_NOISE_PERIOD/1000) # new_time is the time in which it goes forward, until sim is over or noise is introduced
                if(current_time + new_time >= sim_time + pred_time): # if the time it goes forward is greater than the time we want to predict, 
                    new_time = sim_time + pred_time - current_time # then make the time it goes forward end at the end of prediction
                xPos = xPosNew
                yPos = yPosNew
                heading = newHeading
                Theta = heading - math.pi / 2 # use the heading to determine where it is in its revolution
                CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta)) # find the position of the center of the circle based on the position in the revolution
                CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
                newTheta = Theta + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS # find where the roomba will be in its revolution after the time
                xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta)) # use this to find the new position of the roomba
                yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
                newHeading = newTheta + math.pi / 2 # and the new heading of the roomba
                heading = newHeading + 1 * (random.uniform(-ROOMBA_OBSTACLE_NOISE_MAX,ROOMBA_OBSTACLE_NOISE_MAX) / 23) # introduce noise into the roomba
                xPos = xPosNew # repeat above
                yPos = yPosNew
                current_time += new_time
                new_time = sim_time % (ROOMBA_OBSTACLE_NOISE_PERIOD/1000)
                if(current_time + new_time >= sim_time + pred_time):
                    new_time = sim_time + pred_time - current_time
                Theta = heading - math.pi / 2
                CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta))
                CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
                newTheta = Theta + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
                xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta))
                yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
                newHeading = newTheta + math.pi / 2
                current_time += new_time
                xArray.append(xPosNew) # for plotting the points along the way
                yArray.append(yPosNew)
            finalPointsX.append(xPosNew) # for plotting the final position of the roomba
            finalPointsY.append(yPosNew)
            finalHeading.append(newHeading)
        # current_time = sim_time
        # xPosNew = initX
        # yPosNew = initY
        # newHeading = initHeading
        # specialX = []
        # specialY = []
        # while(current_time < sim_time + pred_time): 
        #     # this is similar to above, just with noise removed to show perfect path
        #     new_time = ROOMBA_OBSTACLE_NOISE_PERIOD/1000 - sim_time%(ROOMBA_OBSTACLE_NOISE_PERIOD/1000)
        #     if(current_time + new_time >= sim_time + pred_time):
        #         new_time = sim_time + pred_time - current_time
        #         print(new_time)
        #     xPos = xPosNew
        #     yPos = yPosNew
        #     heading = newHeading
        #     Theta = heading - math.pi / 2
        #     CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta))
        #     CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
        #     newTheta = Theta + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
        #     xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta))
        #     yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
        #     newHeading = newTheta + math.pi / 2
        #     heading = newHeading# + (random.uniform(-ROOMBA_OBSTACLE_NOISE_MAX,ROOMBA_OBSTACLE_NOISE_MAX) / 2)
        #     xPos = xPosNew
        #     yPos = yPosNew
        #     current_time += new_time
        #     new_time = sim_time % (ROOMBA_OBSTACLE_NOISE_PERIOD/1000)
        #     if(current_time + new_time >= sim_time + pred_time):
        #         new_time = sim_time + pred_time - current_time
        #     Theta = heading - math.pi / 2
        #     CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(Theta))
        #     CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(Theta))
        #     newTheta = Theta + new_time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
        #     xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newTheta))
        #     yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newTheta))
        #     newHeading = newTheta + math.pi / 2
        #     current_time += new_time
        #     specialX.append(xPosNew)
        #     specialY.append(yPosNew)
        # a bunch of grid stuff
        #print(np.cov(finalPointsX,finalPointsY))
        #print(np.cov(finalPointsX,finalHeading))
        #print(np.cov(finalPointsY,finalHeading))
        FinalCov = np.concatenate((np.cov(finalPointsX,finalPointsY),np.cov(finalPointsX,finalHeading),np.cov(finalPointsY,finalHeading)),axis=0)#,np.cov(finalPointsY,finalHeading))
        print(FinalCov)
        plt.grid(True)
        plt.xticks(np.arange(min(xArray), max(xArray)+1, 1.0))
        plt.axis('equal')
        plt.xlim([-10,10])
        plt.ylim([-10,10])
        fig = plt.figure()                                                               
        ax = fig.add_subplot(1,1,1)                                                      
        ax.plot(xArray,yArray,'ro')
        ax.plot(finalPointsX,finalPointsY,'bo')
        #ax.plot(specialX,specialY,'go')
        # major ticks every 20, minor ticks every 5                                      
        major_ticks = np.arange(-10, 11, 1)                                                                                             

        ax.set_xticks(major_ticks)                                                                                                 
        ax.set_yticks(major_ticks)                                                                                              

        # and a corresponding grid                                                       

        ax.grid(which='both')                                                            

        # or if you want differnet settings for the grids:                                                                              
        ax.grid(which='major', alpha=0.5)                                                

#        plt.show()
if __name__ == '__main__':
    # rospy.init_node('testing')
    ex = HeatMap()
    #ex.createHeatMap(2,'/obstacle0')
    rospy.sleep(.1)
    ex.heatmapping(20,20,'/obstacle1')