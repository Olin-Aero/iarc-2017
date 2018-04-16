This is the primary repository for the Olin College Aero club's attempt at competing in IARC 2018.

# Running with Docker
## Installing GPU acceleration
<!-- http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration -->
```
# https://nvidia.github.io/nvidia-docker/
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | \
  sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/ubuntu16.04/amd64/nvidia-docker.list | \
  sudo tee /etc/apt/sources.list.d/nvidia-docker.list
sudo apt-get update

# https://github.com/NVIDIA/nvidia-docker/wiki/Installation-(version-1.0)
sudo apt-get install nvidia-docker

```

## Starting the container and getting a terminal
```
nvidia-docker run -it --rm \
	--net foo \
	--name devel \
	--env="DISPLAY" \
	--env=UIUSER=(whoami) \
	--volume="/etc/group:/etc/group:ro" \
	--volume="/etc/passwd:/etc/passwd:ro" \
	--volume="/etc/shadow:/etc/shadow:ro" \
	--volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	ros:iarc-2017-devel   \
	bash
```

## Getting more terminals
```
docker exec -it devel bash
```

## Running GUI tools
```
docker exec devel ui rviz
# or
docker exec devel ui rqt
# .... etc
```

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

