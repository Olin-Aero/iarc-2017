## IARC-Sim-3D

Gazebo World Environment for IARC Mission 7.

IARC Team @ Olin College of Engineering, FA2017

### Dependencies

Currently, the package depends on (listing external packages):

- [aptima-ros-pkg](https://code.google.com/archive/p/aptima-ros-pkg/source/default/source)
    * For convenience, **irobot-create-description** is currently embedded under libraries/
    * The package has been modified enough, so that the original link os provided here mostly for credit; the original package **Will not work**.
- [tum-simulator](https://github.com/angelsantamaria/tum_simulator.git)
    * **NOT** The standard version; needs to be from this repository.
    * Be sure to also install its dependencies.

### Building

```
catkin_make -C ~/catkin_ws
```

Note that multiple `catkin_make` may be necessary.

### Running

```
roslaunch iarc_sim_3d sim.launch
rosrun iarc_forebrain start.py _sim:=true
```

**Wait 5 seconds** in simulation time for all the robots to start running.


**WARNING :** The simulation will run extremely slow on a VM, and may appear broken as a result.
