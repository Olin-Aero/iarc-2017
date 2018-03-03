#!/usr/bin/env python
from std_msgs.msg import Float64

from ChangeHeight import ChangingHeight
from follow_roomba import *


class PushRoomba:
    def __init__(self):
        self.ButtonHit = False
        rospy.Subscriber("/drone/height", Float64, self.callback)
        self.random = ChangingHeight()
        self.follower = RoombaFollower()

    def callback(self, msg):
        self.actualHeight = msg.data

    def run(self):
        # follow the robot code
        self.follower.follow_roomba()

    # if(not self.ButtonHit && distancefromroomba < 0.17):
    #	self.random.changeHeight(0.05)
    # else:
    #	self.random.changeHeight(3.0)
    # print(self.actualHeight)
    # if(self.actualHeight < 0.1):
    #	self.ButtonHit = True
    def runLoop(self):
        while (True):
            roomba = Roomba("target1")
            self.run(roomba)
            rospy.sleep(.1)


if __name__ == '__main__':
    ex = PushRoomba()
    ex.runLoop()
    rospy.spin()
