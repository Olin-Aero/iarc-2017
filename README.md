This is the primary repository for the Olin College Aero club's attempt at competing in IARC 2018.

# Installation instructions

This repository includes a dependency on gstreamer, a program for low-latency network video streaming. You must install it for the program to build.

    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-* gstreamer1.0-libav* gstreamer1.0-plugins*
  
All other dependencies can be installed using `rosdep` and `pip`:

    rosdep install --ignore-src --from-paths $(rospack find iarc_main)/.. -y
    pip install -r $(rospack find iarc_main)/../requirements.txt

Finally, `catkin_make` can be used to build the code

    cd ~/catkin_ws && catkin_make

Copy camera calibration to your ROS install

    mkdir -p ~/.ros/camera_info && cp $(rospack find iarc_vision)/calibration/* ~/.ros/camera_info/

# Useful Commands
As of 12/18/2017, the following commands are useful to run the code on an AR Drone

	roscore
	roslaunch iarc_main backbone.launch
	roslaunch iarc_main ardrone_interface.launch
	
	roslaunch iarc_vision apriltags.launch
	rosrun iarc_main ardrone_keyboard.py cmd_vel:=/teleop/cmd_vel

	# One of the following three...
	rostopic pub /arbiter/activate_behavior std_msgs/String "teleop" -1
	rostopic pub /arbiter/activate_behavior std_msgs/String "goto_click" -1
	rostopic pub /arbiter/activate_behavior std_msgs/String "follow" -1
	
	rqt
	rviz
	rostopic echo /ardrone/navdata/batteryPercent

	rosbag record -a -x ".*image_raw\$|.*tag_detections.*"

