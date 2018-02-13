# IARC Roomba Filter
---

This package primarily deals with multi-target model-based UKF tracking for IARC roombas.

Demo video available [here](https://www.youtube.com/watch?v=vEf5ihOr4Sg) (02/12/2018);

To reproduce the results,

Run the Gazebo simulation:

```bash
roscore
roslaunch iarc_sim_3d sim.launch
```

Launch the drone and start the controller:

```
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" --once
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

Run the filter with visualization and gazebo interface:

```
roslaunch iarc_roombafilter filter.launch sim:=true viz:=true
```
