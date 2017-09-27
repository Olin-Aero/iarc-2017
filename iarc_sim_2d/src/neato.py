#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
import numpy as np
import time
import config as cfg
import os

import actionlib
#import stdr_msgs.msg
from stdr_msgs.msg import FootprintMsg, SpawnRobotAction, SpawnRobotGoal, RobotMsg

'''
roomba.py

Contains two classes to represent target and obstacle roombas.

Behavior:

# Target Roombas
- Every 20 seconds, 180 deg turn clockwise
- Every 5 seconds, +-20 deg turn
- Front collision -> 180 deg turn clockwise
- Top collision -> 45 deg turn clockwise

# Obstacle Roombas
- Move in a circular motion around the origin
- Front collision -> idle until unobstructed
'''


class Roomba(object):
    '''
    Represents a generic roomba.
    (No update function)
    '''

    def __init__(self, pos, heading, tag=''):
        '''
        Initialize a roomba object with a given position and heading.
        By default, the roomba starts in STATE_IDLE.

        pos - [x,y] in meters
        heading - angle in radians (0 is +x and pi/2 is +y)
        [tag] - an optional identification element
        '''
        self.pos = pos
        self.heading = heading
        self.tag = tag
        self.x_vel = 0
        self.z_w = 0

        self.collisions = {
            'front': False,
            'top': False
        }

        self.timers = {
            'reverse': 0,
            'noise': 0
        }

        self.state = cfg.ROOMBA_STATE_IDLE

        # amount we need to turn
        self.turn_target = 0
        self.turn_clockwise = False

    def start(self):
        '''
        Sets the roomba to STATE_FORWARD
        '''
        self.state = cfg.ROOMBA_STATE_FORWARD

    def stop(self):
        '''
        Sets the roomba to STATE_IDLE

        Note: if the roomba was mid-turn during this call, it will
        not resume after a restart.
        '''
        self.state = cfg.ROOMBA_STATE_IDLE

    def update(self, delta, elapsed):
        '''
        Perform an update step (unimplemented)
        '''
        pass


class TargetRoomba(Roomba):
    '''
    Represents a target roomba.
    '''

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        # check for collisions
        if self.collisions['front']:
            self.collisions['front'] = False

            if self.state == cfg.ROOMBA_STATE_FORWARD:
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.pi
                self.turn_clockwise = True

        # update linear or angular motion
        if self.state == cfg.ROOMBA_STATE_FORWARD:
            self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
            self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta
            self.x_vel = cfg.ROOMBA_LINEAR_SPEED
            # check full turn period
            if elapsed - self.timers['reverse'] > cfg.ROOMBA_REVERSE_PERIOD:
                self.timers['reverse'] = elapsed
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.pi
                self.turn_clockwise = True
            # check random noise period
            elif elapsed - self.timers['noise'] > cfg.ROOMBA_HEADING_NOISE_PERIOD:
                self.timers['noise'] = elapsed
                self.state = cfg.ROOMBA_STATE_TURNING
                self.turn_target = np.random.rand() * cfg.ROOMBA_HEADING_NOISE_MAX
                self.turn_clockwise = (np.random.rand() > 0.5)

        elif self.state == cfg.ROOMBA_STATE_TURNING:
            amount = cfg.ROOMBA_ANGULAR_SPEED * delta
            self.x_vel = 0
            self.turn_target -= amount

            if self.turn_clockwise:
                print("Turning clockwise")
                self.z_w = -cfg.ROOMBA_ANGULAR_SPEED
                self.heading -= amount
            else:
                print("Turning counterclockwise")
                self.z_w = cfg.ROOMBA_ANGULAR_SPEED
                self.heading += amount
            
            if self.turn_target < 0:
                # we have completed the turn, reset to forward motion
                print("turn completed")
                self.z_w = 0
                self.state = cfg.ROOMBA_STATE_FORWARD


class ObstacleRoomba(Roomba):
    '''
    Represents an obstacle roomba.
    '''

    def update(self, delta, elapsed):
        '''
        Perform an update step.

        Note: in order to acheive circular motion, the heading is
        updated each step to be perpendicular to a vector from
        the center to the roomba. For small deltas, this should
        provide realistic behavior.

        delta - change in time since last update (seconds)
        elapsed - total time elapsed since start (milliseconds)
        '''
        if self.collisions['front']:
            self.collisions['front'] = False
        elif self.state == cfg.ROOMBA_STATE_FORWARD:
            self.pos[0] += cfg.ROOMBA_LINEAR_SPEED * np.cos(self.heading) * delta
            self.pos[1] += cfg.ROOMBA_LINEAR_SPEED * np.sin(self.heading) * delta

            # reorient so we tangent to a circle centered at the origin 
            ang = np.arctan2(10 - self.pos[1], 10 - self.pos[0])
            self.heading = ang + (cfg.PI / 2)

def create_neatos(targetNum=1, obstacleNum=1):

    creationString = 'rosrun stdr_robot robot_handler add $(rospack find iarc_sim_2d)/data/drone_robot.xml 10 10 0'

    os.system(creationString)

    dist = 2
    dist_2 = 4
    targetArray = []
    obstacleArray = []
    for i in xrange(1,targetNum+1):

        angle = float(i)/targetNum * (cfg.PI*2)

        creationString = 'rosrun stdr_robot robot_handler add $(rospack find iarc_sim_2d)/data/simp_robot.xml %f %f %f' %(np.cos(angle)*dist+10, np.sin(angle)*dist+10, angle)

        os.system(creationString)
        targetArray = np.append(targetArray, TargetRoomba([np.cos(angle)*dist+10,np.sin(angle)*dist+10], angle, "robot%d" %i))

    for j in xrange(targetNum+1,targetNum+obstacleNum+1):
        angle = float(j)/obstacleNum * (cfg.PI*2)
        creationString = 'rosrun stdr_robot robot_handler add $(rospack find iarc_sim_2d)/data/simp_robot.xml %f %f %f' %(np.cos(angle)*dist_2+10, np.sin(angle)*dist_2+10, angle+cfg.PI/2)
        os.system(creationString)
        obstacleArray = np.append(obstacleArray, ObstacleRoomba([np.cos(angle)*dist_2+10,np.sin(angle)*dist_2+10], angle+cfg.PI/2, "robot%d" %j))

    return [targetArray, obstacleArray]


def kill_neatos(targetNum=1, obstacleNum=1):
    for i in xrange(0,targetNum+obstacleNum):
        killString = 'rosrun stdr_robot robot_handler delete robot%d' %i
        os.system(killString)

def update_neatos(neatos, delta, elapsed):
    vel_msg = Twist()
    for target_neato in neatos[0]:
        vel_msg.linear.x = target_neato.x_vel
        vel_msg.angular.z = target_neato.z_w

        target_neato.update(delta, elapsed)
        target_neato.velocity_publisher.publish(vel_msg)

    for obstacle_neato in neatos[1]:
        obstacle_neato.update(delta, elapsed)

def reset_neatos():
    pass

def run_neatos(neatos):
    for target_neato in neatos[0]:
        target_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %target_neato.tag, Twist, queue_size=10)
        target_neato.start()

    for obstacle_neato in neatos[1]:
        obstacle_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %obstacle_neato.tag, Twist, queue_size=10)
        obstacle_neato.start()


    t0 = rospy.Time.now().to_sec()
    t1 = t0 
    t2 = t0

    while not rospy.is_shutdown():
        t2=rospy.Time.now().to_sec()
        dt = t2-t1

        print((t1-t0)*1000)

        update_neatos(neatos, dt, (t2-t0)*1000)

        rospy.sleep(.1)
        t1 = t2

def spawn_robot(
        client=None,
        pose=Pose2D(),
        footprint=FootprintMsg(radius=0.35), # standard roomba dim-ish
        robot_class='' # currently ignored argument
        ):
    if client is None:
        client = actionlib.SimpleActionClient(
                '/stdr_server/spawn_robot',
                SpawnRobotAction
                )
    client.wait_for_server()

    goal = SpawnRobotGoal(
            description=RobotMsg(
                initialPose=pose,
                footprint=footprint
                )
            )
    client.send_goal_and_wait(goal)
    res = client.get_result()
    des = res.indexedDescription
    pose = des.robot.initialPose
    name = des.name

    return robot_class(
            [pose.x, pose.y], # pos
            pose.theta, # heading
            name
            )

def spawn_robots(
        num_targets=1,
        num_obstacles=1,
        ):
    client = actionlib.SimpleActionClient(
            '/stdr_server/spawn_robot',
            SpawnRobotAction
            )

    drone = None
    targets = []
    obstacles = []

    #spawn_robot(client, robot_type='drone')
    for i in xrange(num_targets):
        theta = float(i)/num_targets * (cfg.PI*2)
        pose = Pose2D(
                10 + 4 * np.cos(theta),
                10 + 4 * np.sin(theta),
                theta
                )
        robot = spawn_robot(client, pose, robot_class=TargetRoomba)
        targets.append(robot)
    for i in xrange(num_obstacles):
        theta = float(i)/num_obstacles* (cfg.PI*2)
        pose = Pose2D(
                10 + 2 * np.cos(theta),
                10 + 2 * np.sin(theta),
                theta
                )
        robot = spawn_robot(client, pose, robot_class=ObstacleRoomba)
        obstacles.append(robot)
    return drone, targets, obstacles

def main():
    rospy.init_node('neatos')
    try:
        drone, targets, obstacles = spawn_robots()
        run_neatos([targets, obstacles])
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

if __name__ == '__main__':
    main()
    #try:
    #    #Testing our function
    #    #neatos = create_neatos()
    #    # kill_neatos()
    #    #run_neatos(neatos)
    #    spawn_robot()
    #    rospy.spin()
    #except rospy.ROSInterruptException: pass

