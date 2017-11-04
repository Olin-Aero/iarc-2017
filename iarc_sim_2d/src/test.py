import os
import numpy as np

import rospy
import tf
import actionlib
import rospkg
from geometry_msgs.msg import Twist, Pose2D, Point
from iarc_sim_2d.msg import Roomba, Roombas
rospack = rospkg.RosPack()

from iarc_sim_engine.srv import SpawnRobot, SpawnRobotRequest, SpawnRobotResponse, KillRobotRequest, KillRobotResponse
import config as cfg
from robots import TargetRoomba, ObstacleRoomba, Drone


for i in range(10):
    visRoomba = Roomba()
    # print(visRoomba)
    #visRoomba.x = robot_pos[i][0]
    #visRoomba.y = robot_pos[i][1]
    visRoomba.counter = 4503
    visRoomba.tag = "hi"

    print(visRoomba)
