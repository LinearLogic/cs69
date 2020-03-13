# MR-SLAM with Rendezvous and Frontier Growing

```
This project carries out mr-slam in simulation
```

## Getting Started

```
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workpace
```

### Prerequisites

What software you need to install

```
numpy
ROS
ROS - nav2d
```
### Installing

copy the 'simple_sensing' folder into src.
then build and source:

```
cd ..
catkin_make
```

### Run

to run the nav2d exploration

```
roslaunch mr_slam nav2d_simulation.launch

rosrun mr_slam explorer_team.py
```
