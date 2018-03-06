# IARC\_Vision

## Grid Tracking

Steps to reproduce the results in simulation:

```
# make sure you're in the right place
git fetch
git checkout gridfinding_v3 && git pull

# initialize simulation
roslaunch iarc_sim_3d sim.launch 
rostopic pub /ardrone/takeoff std_msgs/Empty "{}" 

# visualization
rosrun iarc_roombafilter f_rviz.py _map_frame:=odom

# run grid finder
rosrun iarc_main grid_finder.py _ann_out:='ann_img' _odom:='odom'

# move around
rosrun teleop_twist_keyboard teleop_twist_keyboard.py 

# gui to figure out what's happening
rosrun rviz rviz -d $(rospack find iarc_vision)/rviz/grid_tracking_sim.rviz

# view frames
# rosrun rqt_tf_tree rqt_tf_tree
```
