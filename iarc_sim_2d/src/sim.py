#!/usr/bin/env python
import numpy as np

import rospy
import tf
import actionlib

from geometry_msgs.msg import Twist, Pose2D, Point
from stdr_msgs.msg import FootprintMsg, KinematicMsg, SpawnRobotAction, SpawnRobotGoal, RobotMsg, DeleteRobotAction, DeleteRobotGoal

import config as cfg

from robots import TargetRoomba, ObstacleRoomba, Drone

class Simulator(object):
    def __init__(self, num_targets, num_obstacles):
        """ initialize simulation with given number of robots """
        self.tf = tf.TransformListener()
        self.drone, self.targets, self.obstacles = self.spawn_robots(num_targets, num_obstacles)


    def spawn_robot(self,
            client=None,
            pose=Pose2D(),
            footprint=FootprintMsg(radius=cfg.ROOMBA_RADIUS), # standard roomba dim-ish
            robot_class='', # currently ignored argument,
            kinematicModel=KinematicMsg()
            ):
        """ Spawn a single robot with given initial configurations """
        if client is None:
            client = actionlib.SimpleActionClient(
                    '/stdr_server/spawn_robot',
                    SpawnRobotAction
                    )
        client.wait_for_server()

        goal = SpawnRobotGoal(
                description=RobotMsg(
                    initialPose=pose,
                    footprint=footprint,
                    kinematicModel=kinematicModel
                    )
                )
        client.send_goal_and_wait(goal)

        print(kinematicModel)
        #TODO : handle failure?
        res = client.get_result()
        des = res.indexedDescription
        pose = des.robot.initialPose
        name = des.name


        return robot_class(
                [pose.x, pose.y], # pos
                pose.theta, # heading
                name
                )

    def spawn_robots(self, num_targets, num_obstacles):
        """
        Spawn all the robots. 

        TODO : Implement Spawning Drone
        """
        client = actionlib.SimpleActionClient(
                '/stdr_server/spawn_robot',
                SpawnRobotAction
                )

        drone_shape = []
        for j in range(4):
            theta_Rotors = cfg.PI/4 + float(j)/4 * cfg.PI*2
            for i in range(11):
                theta = float(i)/10 * cfg.PI*2
                drone_shape.append(Point(cfg.DRONE_RADIUS*np.cos(theta) + cfg.ROTOR_OFFSET*np.cos(theta_Rotors), cfg.DRONE_RADIUS*np.sin(theta)+ cfg.ROTOR_OFFSET*np.sin(theta_Rotors), 0) )
        drone_footprint = FootprintMsg(points=drone_shape)

        obstacle_shape = []
        for i in range(11):
            theta = float(i)/10 * cfg.PI*2
            obstacle_shape.append(Point(cfg.ROOMBA_RADIUS*np.cos(theta), cfg.ROOMBA_RADIUS*np.sin(theta), 0))
        for i in range(11):
            theta = -float(i)/10 * cfg.PI*2
            obstacle_shape.append(Point(0.1*np.cos(theta), 0.1*np.sin(theta), 0))

        obstacle_footprint = FootprintMsg(points=obstacle_shape)

        drone = None
        targets = []
        obstacles = []

        #Spawn Drone
        theta = cfg.PI/2
        pose = Pose2D(10, 10, theta)

        drone = self.spawn_robot(client, pose, footprint=drone_footprint, robot_class=Drone, kinematicModel=KinematicMsg(type='omni'))


        #self.spawn_robot(client, robot_type='drone')
        for i in xrange(num_targets):
            theta = float(i)/num_targets * (cfg.PI*2)
            pose = Pose2D(
                    10 + 4 * np.cos(theta),
                    10 + 4 * np.sin(theta),
                    2*cfg.PI -theta
                    )
            robot = self.spawn_robot(client, pose, robot_class=TargetRoomba)
            targets.append(robot)
        for i in xrange(num_obstacles):
            theta = float(i)/num_obstacles* (cfg.PI*2)
            pose = Pose2D(
                    10 + 2 * np.cos(theta),
                    10 + 2 * np.sin(theta),
                    theta + cfg.PI/2
                    )
            robot = self.spawn_robot(client, pose,footprint=obstacle_footprint, robot_class=ObstacleRoomba )
            obstacles.append(robot)

        return drone, targets, obstacles

    def delete(self, name):
        """ Delete a single robot with the given name. """
        client = actionlib.SimpleActionClient(
                '/stdr_server/delete_robot',
                DelteRobotAction
                )
        goal = DeleteRobotGoal(name=name)
        client.send_goal_and_wait(goal)
        return client.get_result()

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
            obstacle_neato.update(delta, elapsed)


        vel_msg = self.drone.vel_msg
        self.drone.update(delta, elapsed)
        self.drone.velocity_publisher.publish(vel_msg)

    def run_collision(self):
        """ Handle collision between robots. """
        targets = self.targets # save some typing ...
        try:
            target_pos, target_headings = zip(*[self.tf.lookupTransform(
                'map', '%s'%targets[i].tag, rospy.Time(0)
                ) for i in xrange(len(targets))
                ])
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Exception in tf Lookup")
            return

        drone = self.drone
        print(drone.tag)
        try:
            drone_pos, drone_heading = self.tf.lookupTransform(
                'map', '%s'%drone.tag, rospy.Time(0)
                )

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("Exception in tf Lookup")
            return

        print(drone_pos)

        collisions=[False for _ in xrange(len(targets))]

        n_t = len(targets)
        for i in xrange(0,n_t):
            for j in xrange(i+1, n_t):
                # for i
                h_i = tf.transformations.euler_from_quaternion(target_headings[i])[-1] # yaw
                u_i = [np.cos(h_i), np.sin(h_i)]

                dy = target_pos[j][1] - target_pos[i][1]
                dx = target_pos[j][0] - target_pos[i][0]

                d = np.sqrt(dx**2 + dy**2)
                if d < cfg.ROOMBA_RADIUS*2 and np.dot(u_i, [dx,dy]) > 0:
                    collisions[i] = True

                # for i
                h_j = tf.transformations.euler_from_quaternion(target_headings[j])[-1] # yaw
                u_j = [np.cos(h_j), np.sin(h_j)]

                dy = target_pos[i][1] - target_pos[j][1]
                dx = target_pos[i][0] - target_pos[j][0]

                d = np.sqrt(dx**2 + dy**2)
                if d < cfg.ROOMBA_RADIUS*2 and np.dot(u_j, [dx,dy]) > 0:
                    collisions[j] = True

        for i,f in enumerate(collisions):
            if f and targets[i].state != cfg.ROOMBA_STATE_TURNING:
                targets[i].collisions['front'] = True

    def run(self):
        """ Main loop for running simulation. """
        for target_neato in self.targets:
            target_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %target_neato.tag, Twist, queue_size=10)
            target_neato.start()

        for obstacle_neato in self.obstacles:
            obstacle_neato.velocity_publisher = rospy.Publisher('/%s/cmd_vel' %obstacle_neato.tag, Twist, queue_size=10)
            obstacle_neato.start()

        t0 = rospy.Time.now().to_sec()
        t1 = t0 
        t2 = t0

        while not rospy.is_shutdown():
            self.run_collision()
            t2=rospy.Time.now().to_sec()
            dt = t2-t1

            # print((t1-t0)*1000)

            self.update(dt, (t2-t0)*1000)

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

