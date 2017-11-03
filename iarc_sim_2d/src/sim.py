#!/usr/bin/env python
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


class Simulator(object):
    def __init__(self, num_targets, num_obstacles):
        """ initialize simulation with given number of robots """
        self.tf = tf.TransformListener()
        self.drone, self.targets, self.obstacles = self.spawn_robots(num_targets, num_obstacles)
        self.Vis_Roombas = rospy.Publisher('Vis_Roombas', Roombas, queue_size=10)

    def spawn_robot(self,
            client,
            name, img,
            radius, pose,
            robot_class
            ):
        """ Spawn a single robot with given initial configurations """
        try:
            client(name=name,img=img,radius=radius,x=pose.x,y=pose.y,t=pose.theta)
        except rospy.ServiceException as e:
            print ('Service Call Failed : %s' % e)
            return

        return robot_class(
                [pose.x, pose.y], # pos
                pose.theta, # heading
                name
                )

    def spawn_robots(self, num_targets, num_obstacles):
        """
        Spawn all the robots.
        """

        try:
            client = rospy.ServiceProxy('/spawn', SpawnRobot)
        except rospy.ServiceException as e:
            print ('Client Creation Failed : %s' % e)

        img = {
                'drone' : os.path.join(rospack.get_path('iarc_sim_engine'),'data','drone.jpg'),
                'obstacle' : os.path.join(rospack.get_path('iarc_sim_engine'),'data','roomba.png'),
                'target' : os.path.join(rospack.get_path('iarc_sim_engine'),'data','roomba.png')
                }
        print 'img', img
        drone = None
        targets = []
        obstacles = []

        #Spawn Drone
        drone = self.spawn_robot(
                client,
                name='drone', img=img['drone'],
                radius=cfg.DRONE_RADIUS, pose=Pose2D(0,0,cfg.PI/2),
                robot_class=Drone,
                )

        #self.spawn_robot(client, robot_type='drone')
        for i in xrange(num_targets):
            theta = float(i)/num_targets * (cfg.PI*2)
            pose = Pose2D(
                    cfg.ROOMBA_TARGET_TURN_RADIUS * np.cos(theta),
                    cfg.ROOMBA_TARGET_TURN_RADIUS * np.sin(theta),
                    theta
                    )
            robot = self.spawn_robot(
                    client,
                    name=('target%d'%i), img=img['target'],
                    radius=cfg.ROOMBA_RADIUS, pose=pose,
                    robot_class=TargetRoomba
                    )
            targets.append(robot)
        for i in xrange(num_obstacles):
            theta = float(i)/num_obstacles* (cfg.PI*2)
            # print('%f number%d'%(theta, i))
            pose = Pose2D(
                    cfg.ROOMBA_OBSTACLE_TURN_RADIUS * np.cos(theta),
                    cfg.ROOMBA_OBSTACLE_TURN_RADIUS * np.sin(theta),
                    theta + cfg.PI/2
                    )
            robot = self.spawn_robot(
                    client,
                    name=('obstacle%d'%i), img=img['obstacle'],
                    radius=cfg.ROOMBA_RADIUS, pose=pose,
                    robot_class=ObstacleRoomba
                    )
            robot.gen_pole()
            obstacles.append(robot)

        return drone, targets, obstacles

    def delete(self, name):
        """ Delete a single robot with the given name. """
        try:
            client = rospy.ServiceProxy('/spawn', SpawnRobot)
            return client(name)
        except rospy.ServiceException as e:
            print ('Service Call Failed : %s' % e)
        return False

    def reset(self):
        """ Reset all robots to default position """
        pass

    def update(self, delta, elapsed):
        """ """
        vel_msg = Twist()
        for target_neato in self.targets:
            vel_msg.linear.x = target_neato.x_vel
            vel_msg.angular.z = target_neato.z_w

            target_neato.update(delta, elapsed)
            target_neato.velocity_publisher.publish(vel_msg)

        for obstacle_neato in self.obstacles:
            vel_msg.linear.x = obstacle_neato.x_vel
            vel_msg.angular.z = obstacle_neato.z_w

            obstacle_neato.update(delta, elapsed)
            obstacle_neato.velocity_publisher.publish(vel_msg)


        # vel_msg = self.drone.vel_msg
        self.drone.update(delta, elapsed)
        # self.drone.velocity_publisher.publish(vel_msg)

    def get_positions(self):
        """ Get positions of all robots """
        targets = self.targets # save some typing ...
        obstacles = self.obstacles
        drone = self.drone
        all_robots = np.concatenate((targets, obstacles))
        # print(all_robots)

        try:
            robot_pos, robot_headings = zip(*[self.tf.lookupTransform(
                'map', '%s'%all_robots[i].tag, rospy.Time(0)
                ) for i in xrange(len(all_robots))
                ])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Exception in tf Lookup Robots")
            return

        drone = self.drone
        drone.get_visible_roombas(all_robots, robot_pos)

        roombaArray = Roombas()

        for robot in range(len(drone.index_list)):

            Vis_Roomba = Roomba()
            Vis_Roomba.x = robot_pos[drone.index_list[robot]][0]
            Vis_Roomba.y = robot_pos[robot][1]

            Vis_Roomba.heading = tf.transformations.euler_from_quaternion(robot_headings[robot])[-1]

            Vis_Roomba.tag = all_robots[robot].tag

            Vis_Roomba.noise = all_robots[robot].timers['noise']
            Vis_Roomba.stopped = all_robots[robot].timers['stopped']

            roombaArray.roombas.append(Vis_Roomba)

        self.Vis_Roombas.publish(roombaArray)



        # Vis_Roomba.x = robot_pos[0]
        # Vis_Roomba.y = blah]

        #print(drone.visible_roombas)
        #print(all_robots)
        #print(drone.pos3d[2])#Replace with publishing custom message

        try:
            drone_pos, drone_heading = self.tf.lookupTransform(
                'map', '%s'%drone.tag, rospy.Time(0)
                )
            drone.pos3d[0], drone.pos3d[1] = [drone_pos[0], drone_pos[1]]

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Exception in tf Lookup Drone")
            return
        return robot_pos, robot_headings, drone_pos, drone_heading

    def run_collision(self, robot_pos, robot_headings, drone_pos, drone_heading):
        """ Handle collision between robots. """

        #for target robots
        targets = self.targets # save some typing ...
        obstacles = self.obstacles
        drone = self.drone
        all_robots = np.concatenate((targets, obstacles))

        #Calculate all of the collisions between robots
        n_t = len(all_robots)
        for i in xrange(0,n_t):
            for j in xrange(0, n_t):
                if i != j:
                    all_robots[i].collision(robot_pos[i], robot_headings[i], robot_pos[j], robot_headings[j])

        #Now calculate all of the collisions between the drone and the robots
        for i in xrange(0, n_t):
            if drone.pos3d[2] < 0:
                drone.pos3d[2] = 0

            if drone.pos3d[2] < cfg.ROOMBA_HEIGHT + cfg.PAD_HEIGHT and type(all_robots[i]) is TargetRoomba:
                d = np.sqrt((drone.pos3d[0] - robot_pos[i][0])**2 + (drone.pos3d[1] - robot_pos[i][1])**2)
                #Before normal collisions are checkes, see if it's a tap
                if drone.vel3d.linear.z < 0 and d < cfg.SENSOR_PAD_RADIUS:
                    drone.pos3d[2] = cfg.ROOMBA_HEIGHT + cfg.PAD_HEIGHT #Make the drone not fall through the roomba
                    all_robots[i].collisions['top'] = True
                    print('hit on top')
                    continue

            #Standard Collision
            if drone.pos3d[2] < cfg.ROOMBA_HEIGHT:
                #Do normal collisions
                all_robots[i].collision(robot_pos[i], robot_headings[i], drone_pos, drone_heading, self_radius=cfg.ROOMBA_RADIUS, other_radius=(cfg.DRONE_RADIUS+cfg.ROTOR_OFFSET))

            #Collision type for hitting obstacle roombas
            if drone.pos3d[2] > cfg.ROOMBA_HEIGHT and type(all_robots[i]) is ObstacleRoomba:

                if drone.pos3d[2] < cfg.ROOMBA_HEIGHT+all_robots[i].pole_height:
                    all_robots[i].collision(robot_pos[i], robot_headings[i], drone_pos, drone_heading, self_radius=cfg.OBSTACLE_POLE_RADIUS, other_radius=(cfg.DRONE_RADIUS+cfg.ROTOR_OFFSET))
                    drone.collision( drone_pos, drone_heading, robot_pos[i], robot_headings[i], self_radius=(cfg.DRONE_RADIUS+cfg.ROTOR_OFFSET), other_radius=cfg.OBSTACLE_POLE_RADIUS)


            # print(all_robots[i] is ObstacleRomb())

            # elif drone.pos3d[2] > a

    def run_bounds(self, robot_pos):
        all_robots = np.concatenate((self.targets, self.obstacles))
        for r,p in zip(all_robots, robot_pos):
            r.bounds(p)

    def run(self):
        """ Main loop for running simulation. """
        for target_neato in self.targets:
            target_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %target_neato.tag, Twist, queue_size=10)
            target_neato.start()

        for obstacle_neato in self.obstacles:
            obstacle_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %obstacle_neato.tag, Twist, queue_size=10)
            obstacle_neato.start()

        self.drone.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %self.drone.tag, Twist, queue_size=10)
        self.drone.velocity_subscriber = rospy.Subscriber('/%s/cmd_vel' %self.drone.tag, Twist, self.drone.record_vel)
        #self.drone.visible_roomba_publisher = rospy.Publisher('/%s/vis_room' %self.drone.tag, , queue_size=10)
        t0 = rospy.Time.now().to_sec()
        t1 = t0
        t2 = t0

        while not rospy.is_shutdown():
            try:
                robot_pos, robot_headings, drone_pos, drone_heading = self.get_positions()
            except:
                continue
            self.run_collision(robot_pos, robot_headings, drone_pos, drone_heading)
            self.run_bounds(robot_pos)

            t2=rospy.Time.now().to_sec()
            dt = t2-t1

            # print((t1-t0)*1000)
            # print(self.drone.vel3d)
            # print(self.drone.pos3d[2])
            self.update(dt, (t2-t0)*1000)
            # print dt

            rospy.sleep(.1)
            t1 = t2

def main():
    rospy.init_node('neatos')
    num_targets = rospy.get_param('~num_targets', default=1)
    num_obstacles = rospy.get_param('~num_obstacles', default=1)
    try:
        sim = Simulator(num_targets, num_obstacles)
        sim.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('Failed to run simulation : Aborting')
    return

if __name__ == '__main__':
    main()
