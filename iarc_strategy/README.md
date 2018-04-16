# IARC Prediction and Strategy Development

---

Current Development Architecture:

![arch\_1](./figures/arch_1.jpg)

## Exploration

To test exploration with 2D Simulation:

```bash
roscore
roslaunch iarc_sim_2d basic.launch
roslaunch iarc_main backbone.launch
rostopic pub /arbiter/activate_behavior std_msgs/String "data: 'forebrain'" --once
rosrun iarc_strategy explorer.py #_viz:=True
rosrun iarc_forebrain forebrain.py
#rosrun iarc_roombafilter f_ros.py _sim2d:=True
#rosrun iarc_roombafilter f_rviz.py
rostopic pub /start_round std_msgs/Bool "data: true" --once
#rosrun rqt_reconfigure rqt_reconfigure
```
