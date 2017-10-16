#!/usr/bin/env python

import roslaunch
import rospy
import rospkg
import os

if __name__ == "__main__":
    rospack = rospkg.RosPack()
    rospy.init_node('iarc_spawn', anonymous=True)

    pkg_root = rospack.get_path('iarc_sim_3d')
    launch_file = os.path.join(pkg_root, 'launch', 'spawn_roomba.launch')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid,[launch_file])

    launch.start()
    launch.shutdown()
