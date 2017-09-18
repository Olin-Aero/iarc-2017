#!/usr/bin/env python

# 
#
# A modification to the Neato ROS bridge developed by Michael Ferguson
# Changes:
#   (1) Support for connecting over a socket
#   (2) Flipped the laser coordinate frame around to adhere to ROS conventions
#       (enables use of gmapping)
#   (3) Expose more sensors on the robot (including bump sensor)

# ROS node for the Neato Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its 
#       contributors may be used to endorse or promote products derived 
#       from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
ROS node for Neato XV-11 Robot Vacuum.
"""

__author__ = "Paul.Ruvolo@olin.edu (Paul Ruvolo)"
# NOTE: heavily based on Michael Ferguson's original work

import time
import roslib; roslib.load_manifest("neato_node")
import rospy
from math import sin,cos, pi

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from neato_node.msg import Bump, Accel
from tf.broadcaster import TransformBroadcaster
import numpy as np
import threading
from copy import copy

from neato_driver.neato_hybrid_driver import xv11, BASE_WIDTH, MAX_SPEED

class NeatoNode(object):

    def __init__(self):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('neato')

        host = rospy.get_param('~host')
        rospy.loginfo("Connecting to host: %s"%(host))

        self.robot = xv11(host)

        rospy.Subscriber("cmd_vel", Twist, self.cmdVelCb)
        self.scanPub = rospy.Publisher('scan', LaserScan, queue_size=10)
        self.odomPub = rospy.Publisher('odom',Odometry, queue_size=10)
        self.bumpPub = rospy.Publisher('bump',Bump, queue_size=10)
        self.accelPub = rospy.Publisher('accel',Accel, queue_size=10)

        self.odomBroadcaster = TransformBroadcaster()

        self.cmd_vel = None
        self.cmd_vel_lock = threading.Lock()

    def spin(self):
        old_ranges = None
        encoders = [0,0]

        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0

        # things that don't ever change
        scan_link = rospy.get_param('~frame_id','base_laser_link')
        scan = LaserScan(header=rospy.Header(frame_id=scan_link)) 
        scan.angle_min = -pi
        scan.angle_max = pi
        scan.angle_increment = pi/180.0
        scan.range_min = 0.020
        scan.range_max = 5.0
        scan.time_increment = 1.0/(5*360)
        scan.scan_time = 0.2
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
    
        # main loop of driver
        r = rospy.Rate(20)
        rospy.sleep(4)

        # do UDP hole punching to make sure the sensor data from the robot makes it through
        self.robot.do_udp_hole_punch()

        scan.header.stamp = rospy.Time.now()
        last_motor_time = rospy.Time.now()
        last_set_motor_time = rospy.Time.now()

        total_dth = 0.0
        while not rospy.is_shutdown():
            self.robot.requestScan()
            new_stamp = rospy.Time.now()
            delta_t = (new_stamp - scan.header.stamp).to_sec()
            scan.header.stamp = new_stamp
            (scan.ranges, scan.intensities) = self.robot.getScanRanges()

            # repeat last measurement to simulate -pi to pi (instead of -pi to pi - pi/180)
            # This is important in order to adhere to ROS conventions regarding laser scanners
            if len(scan.ranges):
                scan.ranges.append(scan.ranges[0])
                scan.intensities.append(scan.intensities[0])
                if old_ranges == scan.ranges:
                    scan.ranges, scan.intensities = [], []
                else:
                    old_ranges = copy(scan.ranges)

            if delta_t-0.2 > 0.1:
                print "Iteration took longer than expected (should be 0.2) ", delta_t

            # get motor encoder values
            curr_motor_time = rospy.Time.now()
            try:
                motors = self.robot.getMotors()
                if motors:
                    # unpack the motor values since we got them.
                    left, right = motors
                    delta_t = (rospy.Time.now() - scan.header.stamp).to_sec()
                    # now update position information
                    # might consider moving curr_motor_time down
                    dt = (curr_motor_time - last_motor_time).to_sec()
                    last_motor_time = curr_motor_time

                    d_left = (left - encoders[0])/1000.0
                    d_right = (right - encoders[1])/1000.0

                    encoders = [left, right]
                    dx = (d_left+d_right)/2
                    dth = (d_right-d_left)/(BASE_WIDTH/1000.0)
                    total_dth += dth

                    x_init = self.x
                    y_init = self.y
                    th_init = self.th

                    x = cos(dth)*dx
                    y = -sin(dth)*dx

                    self.x += cos(self.th)*x - sin(self.th)*y
                    self.y += sin(self.th)*x + cos(self.th)*y
                    self.th += dth

                    quaternion = Quaternion()
                    quaternion.z = sin(self.th/2.0)
                    quaternion.w = cos(self.th/2.0)

                    # prepare odometry
                    odom.header.stamp = curr_motor_time
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    odom.pose.pose.position.z = 0
                    odom.pose.pose.orientation = quaternion
                    odom.pose.covariance = [10**-1, 0, 0, 0, 0, 0,
                                            0, 10**-1, 0, 0, 0, 0,
                                            0, 0, 10**-1, 0, 0, 0,
                                            0, 0, 0, 10**5, 0, 0,
                                            0, 0, 0, 0, 10**5, 0,
                                            0, 0, 0, 0, 0, 10**5]
                    odom.twist.twist.linear.x = dx/dt
                    odom.twist.twist.angular.z = dth/dt
                    self.odomBroadcaster.sendTransform( (self.x, self.y, 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), curr_motor_time, "base_link", "odom" )
                    self.odomPub.publish(odom)
            except Exception as err:
                print "my error is " + str(err)
            with self.cmd_vel_lock:
                if self.cmd_vel:
                    self.robot.setMotors(self.cmd_vel[0],
                                         self.cmd_vel[1],
                                         max(abs(self.cmd_vel[0]),abs(self.cmd_vel[1])))
                    self.cmd_vel = None
                elif rospy.Time.now() - last_set_motor_time > rospy.Duration(.2):
                    self.robot.resend_last_motor_command()
                    last_set_motor_time = rospy.Time.now()

            try:
                bump_sensors = self.robot.getDigitalSensors()
                if bump_sensors:
                    self.bumpPub.publish(Bump(leftFront=bump_sensors[0],leftSide=bump_sensors[1],rightFront=bump_sensors[2],rightSide=bump_sensors[3]))
            except:
                print "failed to get bump sensors!"

            try:
                accelerometer = self.robot.getAccel()
                if accelerometer:
                    self.accelPub.publish(Accel(accelXInG=accelerometer[2],accelYInG=accelerometer[3],accelZInG=accelerometer[4]))
            except Exception as err:
                print "failed to get accelerometer!", err

            if len(scan.ranges):
                self.scanPub.publish(scan)
            # wait, then do it again
            r.sleep()

    def cmdVelCb(self,req):
        # Simple odometry model
        x = req.linear.x * 1000
        th = req.angular.z * (BASE_WIDTH/2) 
        k = max(abs(x-th),abs(x+th))
        # sending commands higher than max speed will fail
        if k > MAX_SPEED:
            x = x*MAX_SPEED/k; th = th*MAX_SPEED/k
        with self.cmd_vel_lock:
            self.cmd_vel = [ int(x-th) , int(x+th) ]
        #print self.cmd_vel, "SENDING THIS VEL"

if __name__ == "__main__":
    robot = NeatoNode()
    robot.spin()
