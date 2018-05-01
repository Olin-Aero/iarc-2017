#!/usr/bin/env python

import rospy
from iarc_main.msg import StartRound

def main():
    rospy.init_node('start')
    pub = rospy.Publisher('/start_round', StartRound, queue_size=1, latch=True)
    msg = StartRound()
    msg.time = rospy.Time.now()
    print msg.time
    msg.start = True
    pub.publish(msg)
    rospy.spin()

if __name__ == "__main__":
    main()
