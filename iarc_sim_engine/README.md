##IARC\_Sim\_Engine
---

C++/Qt4 Engine for 2D IARC Simulation.

## Running

```bash
rosrun iarc_sim_engine iarc_sim_node
```

### Spawn a New Robot

```bash
rosservice call /iarc_sim_engine/spawn "{name: 'robot_name', img: '/home/jamie/Downloads/roomba.png', radius: 0.5, x: 0.0, y: 0.4, t: 0.2}" 
```

here, t is the heading.

### Kill existing Robot

```bash
rosservice call /iarc_sim_engine/kill "name: 'robot_name'" 
```
