# IARC\_Vision

## Run Perception Pipeline with Gazebo

### Setup

```bash
roslaunch iarc_sim_3d sim.launch
rosrun tf static_transform_publisher 0 0 0 0 0 0 drone/base_link base_link 1
roslaunch iarc_roombafilter filter.launch gz_obs:=false
rosrun iarc_vision colortracking.py _gui:=False
```

### Start

```bash
rosrun iarc_forebrain start.py _sim:=true
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" --once
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```

## Grid Tracking

Steps to reproduce the results in simulation:

```bash
# make sure you're in the right place
git fetch
git checkout gridfinding_v3 && git pull

# initialize simulation
roslaunch iarc_sim_3d sim.launch 
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" 

# visualization
rosrun iarc_roombafilter f_rviz.py _map_frame:=odom

# run grid finder
rosrun iarc_vision grid_finder.py _ann_out:='ann_img' _odom:='odom'

# move around
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

# gui to figure out what's happening
rosrun rviz rviz -d $(rospack find iarc_vision)/rviz/grid_tracking_sim.rviz

# view frames
# rosrun rqt_tf_tree rqt_tf_tree
```

## Testing Grid Tracking with Noisy Odometry

```bash
roscore
roslaunch iarc_sim_3d sim.launch
roslaunch iarc_sim_3d noisy_odom.launch
rosrun tf static_transform_publisher 0 0 0 0 0 0 drone/base_link base_link 1
rosrun iarc_forebrain start.py _sim:=true
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" --once
rosrun iarc_vision grid_finder.py _ann_out:=ann_out _map:=map
roslaunch iarc_main backbone.launch 
rostopic pub /arbiter/activate_behavior std_msgs/String "data: 'goto_click'" 

# visualization
rosrun iarc_roombafilter f_rviz.py _map_frame:=odom
rviz
```


```bash
roscore
roslaunch iarc_sim_3d sim.launch
roslaunch iarc_sim_3d noisy_odom.launch 
rosrun tf static_transform_publisher 0 0 0 0 0 0 drone/base_link base_link 1
rosrun iarc_forebrain start.py _sim:=True
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" --once
rosrun iarc_vision grid_finder.py _ann_out:=ann_out _map:=map
roslaunch iarc_main backbone.launch 
rosrun iarc_roombafilter f_rviz.py _map_frame:=map
rosrun iarc_strategy explorer.py 
rosrun iarc_vision color_tracker.py 
rosrun iarc_forebrain forebrain.py 
```
