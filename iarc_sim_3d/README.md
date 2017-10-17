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

### Running

Run `roscore` first, and in a new terminal:

```
roslaunch iarc_sim_3d sim.launch
```