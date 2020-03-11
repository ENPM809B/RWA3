# RWA3

## Authors

* **Raj Prakash Shinde** [GitHub](https://github.com/RajPShinde)
* **Shubham Sonawane** [GitHub](https://github.com/shubham1925)
* **Rachith Prakash** [GitHub](https://github.com/RachithP)
* **Chinmay JosHi** [GitHub](https://github.com/Chinj17)
* **Shesh Mali** [GitHub](https://github.com/smali08)


## Overview
A Package to grasp a part of an order from the conveyor belt

## Dependencies
1. Ubuntu 18.04
2. ROS Melodic

## Build
Steps to build
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
cd src/
git clone https://github.com/ENPM809B/RWA3.git
cd ~/catkin_ws/
catkin_make
```
## Run
To run all nodes through the launch file
```
source devel/setup.bash
roslaunch group2_rwa3 ariac_manager.launch
```
In second terminal:
```
source devel/setup.bash
roslaunch group2_rwa3 group2_rwa3
```

In third terminal:
```
source devel/setup.bash
rosservice call /ariac/arm1/gripper/control "enable: true"
```

