#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3
import tf
import math

class Example:
    def __init__(self):
        self.pub0 = rospy.Publisher('/robot0/cmd_vel', Twist, queue_size=10)
        self.pub1 = rospy.Publisher('/robot1/cmd_vel', Twist, queue_size=10)
        rospy.init_node('Example')
        rospy.Subscriber("/user_input", Bool, self.user_input_callback)
        self.spiral = False

        self.listener = tf.TransformListener()
        time.sleep(1)

    def user_input_callback(self, msg):
        print msg.data
        self.spiral = msg.data

  
    def run(self):   
        r = rospy.Rate(2)

        while not rospy.is_shutdown():
            i = 2
            while i > 0:
                trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                ang = rot
                distance = math.sqrt(trans[0]**2 + trans[1]**2)
                xDist = trans[0]
                yDist = trans[1]
                edge = 1
                xOrig = (xDist - 10)
                yOrig = (yDist - 10)
                origDist = math.sqrt(yOrig**2 + xOrig**2)
                scaleFactorAV = 1
                spiralFactor = 1
                linearSpeed = 3
                angularSpeed = 1
                angVel = 2
                if origDist != 0:
                    angVel = 1/math.sqrt(origDist)
                #ang = rot
                r.sleep()
                if self.spiral and yDist > edge and yDist < 20- edge and xDist > edge and xDist < 20 - edge:
                    output = Twist()
                    output.linear = Vector3(linearSpeed, 0, 0)
                    output.angular = Vector3(0,0,0)
                    # output.linear = Vector3(linearSpeed, 0, 0)
                    print origDist
                    self.pub0.publish(output)
                    trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                    i = i / 1.05
                    if abs(xOrig) > 6 or abs(yOrig) > 6:
                        spiralFactor = -1
                    if abs(xOrig) < 3 or abs(yOrig) < 3:
                        spiralFactor = 1
                    while xOrig < -1 * yOrig + spiralFactor and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        xOrig = trans[0] - 10
                        yOrig = trans[1] - 10
                        print xOrig
                        output = Twist()
                        output.linear = Vector3(linearSpeed, 0, 0)
                        output.angular = Vector3(0,0,0)
                        self.pub0.publish(output)
                    while(rot[2] < 0.7  and self.spiral):
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        print rot
                        output = Twist()
                        output.linear = Vector3(0, 0, 0)
                        output.angular = Vector3(0,0,angularSpeed)
                        self.pub0.publish(output)
                    while yOrig < xOrig + spiralFactor and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        xOrig = (trans[0] - 10)
                        yOrig = (trans[1] - 10)
                        output = Twist()
                        output.linear = Vector3(linearSpeed, 0, 0)
                        output.angular = Vector3(0,0,0)
                        self.pub0.publish(output)
                    while(rot[3] > 0) and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        print rot
                        output = Twist()
                        output.linear = Vector3(0, 0, 0)
                        output.angular = Vector3(0,0,angularSpeed)
                        self.pub0.publish(output)
                    while -1 * xOrig < yOrig + spiralFactor  and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        xOrig = (trans[0] - 10)
                        yOrig = (trans[1] - 10)
                        
                        output = Twist()
                        output.linear = Vector3(linearSpeed, 0, 0)
                        output.angular = Vector3(0,0,0)
                        self.pub0.publish(output)
                    while(rot[3] < 0.7)  and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        print rot
                        output = Twist()
                        output.linear = Vector3(0, 0, 0)
                        output.angular = Vector3(0,0,angularSpeed)
                        self.pub0.publish(output)
                    while -1 * yOrig < -1 * xOrig + spiralFactor  and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        xOrig = (trans[0] - 10)
                        yOrig = (trans[1] - 10)
                        output = Twist()
                        output.linear = Vector3(linearSpeed, 0, 0)
                        output.angular = Vector3(0,0,0)
                        self.pub0.publish(output)
                    while(rot[2] < 0)  and self.spiral:
                        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
                        print rot
                        output = Twist()
                        output.linear = Vector3(0, 0, 0)
                        output.angular = Vector3(0,0,angularSpeed)
                        self.pub0.publish(output)
                else:
                    i = 2

                    #if(xDist < edge):
                    #    while rot[2] < 0:
                    #        output.angular = Vector3(0,0,angularSpeed)
                    #        (trans, rot) = self.listener.lookupTransform( '/map', '/base_link',rospy.Time(0))
                    #        (trans, rot) = self.listener.lookupTransform(  '/base_link','/map',rospy.Time(0))
                    output = Twist()
                    output.linear = Vector3(0, 0, 0)
                    output.angular = Vector3(0,0,0)
                    #print xDist
                    self.pub0.publish(output)
    def edgy():
        trans, rot = self.listener.lookupTransform('/map','/base_link',rospy.Time(0))
        ang = rot
        distance = math.sqrt(trans[0]**2 + trans[1]**2)
        xDist = trans[0]
        yDist = trans[1]
        return self.spiral and yDist > edge and yDist < 20- edge and xDist > edge and xDist < 20 - edge
if __name__ == '__main__':
    ex = Example()
    ex.run()
