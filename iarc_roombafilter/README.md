# IARC Roomba Filter
---

This package primarily deals with multi-target model-based UKF tracking for IARC roombas.

Demo video available [here](https://www.youtube.com/watch?v=vEf5ihOr4Sg) (02/12/2018);
and [here](https://www.youtube.com/watch?v=WNuSyY4nzlk) with added observability (02/27/2018).

## Overview

See [here](writeup.md) for a quick write-up of how the filter works, or a short diagram below:

![pipeline.png](./figures/pipeline.png)

## Running

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
