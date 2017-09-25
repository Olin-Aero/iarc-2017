IARC 2D Simulation Environment

---

### Install Dependencies

```bash
rosdep install iarc_sim_2d
```

### Running

```bash
roslaunch iarc_sim_2d basic.launch
```

### TroubleShooting

If you get an error message like this:

```bash
[ERROR] [1506298878.129997008]: Skipping XML Document "/opt/ros/kinetic/share/hector_pose_estimation/hector_pose_estimation_nodelets.xml" which had no Root Element.  This likely means the XML is malformed or missing.
```

(*Hotfix Warning*) Then simply get the file and put it there:

```bash
sudo wget -P $(rospack find hector_pose_estimation) https://raw.githubusercontent.com/tu-darmstadt-ros-pkg/hector_localization/catkin/hector_pose_estimation/hector_pose_estimation_nodelets.xml
```
