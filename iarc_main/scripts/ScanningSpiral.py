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
                distance = math.sqrt(trans[0]**2 + trans[1]**2)
                xDist = trans[0]
                yDist = trans[1]
                edge = 1
                xOrig = abs(10-xDist)
                yOrig = abs(10-yDist)
                origDist = math.sqrt(yOrig**2 + xOrig**2)
                scaleFactorAV = 1
                angVel = 2
                if origDist != 0:
                    angVel = 1/math.sqrt(origDist)
                #ang = rot
                r.sleep()
                if self.spiral and yDist > edge and yDist < 20- edge and xDist > edge and xDist < 20 - edge:
                    output = Twist()
                    output.linear = Vector3(1, 0, 0)
                    output.angular = Vector3(0,0,angVel)
                    # output.linear = Vector3(1, 0, 0)
                    print origDist
                    self.pub0.publish(output)
                    i = i / 1.05
                else:
                    i = 2

                    #if(xDist < edge):
                    #    while rot[2] < 0:
                    #        output.angular = Vector3(0,0,1)
                    #        (trans, rot) = self.listener.lookupTransform( '/map', '/base_link',rospy.Time(0))
                    #        (trans, rot) = self.listener.lookupTransform(  '/base_link','/map',rospy.Time(0))
                    output = Twist()
                    output.linear = Vector3(0, 0, 0)
                    output.angular = Vector3(0,0,0)
                    #print xDist
                    self.pub0.publish(output)

if __name__ == '__main__':
    ex = Example()
    ex.run()
