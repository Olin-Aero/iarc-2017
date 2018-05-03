#!/usr/bin/env python

import rospy
from iarc_main.msg import StartRound
from std_srvs.srv import Empty

def main():
    rospy.init_node('start')
    _sim = rospy.get_param('~sim', default=False)
    print('_sim:{}'.format(_sim))

    pub = rospy.Publisher('/start_round', StartRound, queue_size=1, latch=True)
    if _sim:
        srv = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        srv()

    msg = StartRound()
    msg.time = rospy.Time.now()
    print msg.time
    msg.start = True
    pub.publish(msg)
    rospy.spin()

if __name__ == "__main__":
    main()
