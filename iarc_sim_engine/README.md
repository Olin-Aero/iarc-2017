##IARC\_Sim\_Engine
---

C++/Qt4 Engine for 2D IARC Simulation.

## Install ..

```bash
rosdep install iarc_sim_engine --ignore-src
catkin_make -C ~/catkin_ws
```

## Running

```bash
rosrun iarc_sim_engine iarc_sim_node
```

### Spawn a New Robot

```bash
rosservice call /iarc_sim_engine/spawn "{name: 'robot_name', img: "$(rospack find iarc_sim_engine)/data/roomba.png", radius: 0.35, x: 0.0, y: 0.4, t: 0.2}" 
```

here, t is the heading.

### Kill existing Robot

```bash
rosservice call /iarc_sim_engine/kill "name: 'robot_name'" 
```
