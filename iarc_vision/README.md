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
```
rosrun iarc_forebrain start.py _sim:=true
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" --once
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 
```
