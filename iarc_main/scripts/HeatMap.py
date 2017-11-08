#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
import tf
import math
from geometry_msgs.msg import Twist, Vector3
import sys
import os
import rospkg
rospack = rospkg.RosPack()
iarc_sim_path = rospack.get_path('iarc_sim_2d')
sys.path.append(os.path.join(iarc_sim_path, 'src'))

from config import *
class HeatMap:
    def __init__(self):
        self.heightPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmdHeight = rospy.Publisher('/cmd_height', Float64, queue_size=10)
        self.tf = tf.TransformListener()
        rospy.init_node('Example')        
        rospy.Subscriber("/drone/height", Float64, self.callback)
        rospy.sleep(0.1)
    def callback(self, msg):
        self.actualHeight = msg.data
    def createHeatMap2(self,time):
        #ROOMBA_LINEAR_SPEED = 0.33
        #ROOMBA_OBSTACLE_NOISE_MAX = 23 * (np.pi / 180)
        xPos = 2
        yPos = 1
        multiplyingFactor = math.tan(ROOMBA_OBSTACLE_NOISE_MAX / 2) * ROOMBA_LINEAR_SPEED
        heading = math.pi / 4
        CircleRad = heading - math.pi / 2
        CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(CircleRad))
        CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(CircleRad))
        newCircleRad = CircleRad + time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
        xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newCircleRad))
        yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newCircleRad))
        ellipseRad1 = multiplyingFactor * time
        positionsX = []
        positionsY = []
        total = 0
        self.mappingthing = [[0.0 for _ in range(40)] for _ in range(40)]
        for i in range(100):
            adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1.0 / 2.0 * ((i-50.0) * 3 / 50)**2))
            #adding = 1.0
            total += adding
        for i in range(100):
            tempx = xPosNew + (i-50) * math.sin(newCircleRad) * ellipseRad1 /  50
            tempy = yPosNew + (i-50) * math.cos(newCircleRad) * ellipseRad1 / 50
            if(tempx > -10 and tempx < 10 and tempy > -10 and tempy < 10):
                adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1. / 2.0 * ((i-50.0) * 3 / 50)**2.0))
                #adding = 1.0
                #print(tempy)
                #print(adding / total)
                self.mappingthing[int((tempx+10) * 2)][int((tempy+10) * 2)] += adding / total
        print(self.mappingthing)
        #return mappingthing
    def createHeatMap(self,time):
        #ROOMBA_LINEAR_SPEED = 0.33
        #ROOMBA_OBSTACLE_NOISE_MAX = 23 * (np.pi / 180)
        
        roombaPos,roombaHeading = self.tf.lookupTransform('map', '/obstacle0', rospy.Time(0))
        xPos = roombaPos[0]
        yPos = roombaPos[1]
        multiplyingFactor = math.tan(ROOMBA_OBSTACLE_NOISE_MAX / 2) * ROOMBA_LINEAR_SPEED
        heading = tf.transformations.euler_from_quaternion(roombaHeading)[-1]
        CircleRad = heading - math.pi / 2
        CircleCenter_x = xPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(CircleRad))
        CircleCenter_y = yPos - (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(CircleRad))
        newCircleRad = CircleRad + time * ROOMBA_LINEAR_SPEED / ROOMBA_OBSTACLE_TURN_RADIUS
        xPosNew = CircleCenter_x + (ROOMBA_OBSTACLE_TURN_RADIUS* math.cos(newCircleRad))
        yPosNew = CircleCenter_y + (ROOMBA_OBSTACLE_TURN_RADIUS* math.sin(newCircleRad))
        ellipseRad1 = multiplyingFactor * time
        xBox = int(xPosNew * 2) / 2.0
        yBox = int(yPosNew * 2) / 2.0
        print(xBox)
        positionsX = []
        positionsY = []
        confidence = 0.0
        total = 0
        for i in range(100):
            adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1.0 / 2.0 * ((i-50.0) * 3 / 50)**2))
            total += adding
        for i in range(100):
            tempx = xPosNew + (i-50) * math.sin(newCircleRad) * ellipseRad1 /  50
            tempy = yPosNew + (i-50) * math.cos(newCircleRad) * ellipseRad1 / 50
            if(tempx > xBox and tempx < xBox+0.5 and tempy > yBox and tempy < yBox+0.5):
                adding = (1.0 / math.sqrt(2*math.pi) * math.e ** (-1. / 2.0 * ((i-50.0) * 3 / 50)**2.0))
                confidence += adding / total

        print(confidence)
if __name__ == '__main__':
    # rospy.init_node('testing')
    ex = HeatMap()
    ex.createHeatMap(20)
    rospy.spin()