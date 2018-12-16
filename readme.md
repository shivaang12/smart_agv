# Smart AGV
[![Build Status](https://travis-ci.org/shivaang12/smart_agv.svg?branch=master)](https://travis-ci.org/shivaang12/smart_agv)
[![Coverage Status](https://coveralls.io/repos/github/shivaang12/smart_agv/badge.svg?branch=master)](https://coveralls.io/github/shivaang12/smart_agv?branch=master)

## Overview
This is an smart AGV (Automated Guided Vehicle) project which can take commands from the end user, and navigate autonomously to perform a required task. This AGV is implemented in ROS Gazebo simulation environment using the TurtleBot as the base platform. A custom global planner was written as a plugin, and uses the A* algorithm for planning. SLAM was done prior on the environment to get a map for navigation.

## Algorithm
The algorithm used in this package, Astar,  is optimal given the known environement. The AGV can traverse avoiding static obstacles. It recieves goal position and uses the current information on the environement to find the shortest path possible while avoiding the obstacles.

## Personnel
This module was created by Shivang Patel as a final project for the University of Maryland course ENPM808X - Software Development for Robotics during the Fall 2018 semester.

## License
MIT open-source license, see [license](https://opensource.org/licenses/MIT)

## Dependencies (for now)
- Ubuntu 16.04
- ROS Kinetic Kame
- turtlebot_gazebo
- turtlebot_navigation
- Package Dependencies
    - roscpp
    - std_msgs
    - message_generation
    - geometry_msgs
    - actionlib
    - nav_msgs
    - move_base_msgs
    - tf
    - nav_core
    - base_local_planner
    - acitonlib_msgs
    - genmsg

## SIP
SIP process chart can be access [here](https://docs.google.com/spreadsheets/d/15FUiV2CVoGEMxKz0mKP8EMVmY0W706Poa8y_qtE8bmY/edit?usp=sharing)

## Sprint Notes

Click [here](https://docs.google.com/document/d/1l9LacTmeBo5nzlTT7CSJo50k2eist533l3HIxqJ_3jI/edit?usp=sharing) to access sprint notes.

## Build

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone --recursive https://github.com/shivaang12/smart_agv.git
$ cd ~/catkin_ws/
$ catkin_make
```

## Run/Demo
To run demo, please follow the below steps after building the repository from above mentioned steps.
```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ roslaunch smartagv agv_demo.launch
```
This will launch gazebo simulator and in it one can see turtlebot spawned in a small warehouse.

In this picture, there are there predefined areas where AGV can be traversed through commands.
- storage
- shop
- rest pos

Now to control AGV and give direction to move to these predefined areas follow below steps in a new terminal.

> NOTE: If you have not yet launched "agv_demo.launch" in other terminal, please go ahead and do that first. Below steps won't work untill you do that first. Steps are mention above.

```bash
$ cd ~/catkin_ws/
$ source devel/setup.bash
$ rosservice call /agvService "move to" "storage"
```
After running above commands you can see in gazebo that turtlebot is moving to storage area. In order to move AGV to machine shop, replace "storage" with "shop" and if return to initial position replace "storage" with "rest pos". 

## Testing with rostest
Test of level 1/2 has been implemented. Please follow the below steps to build and run tests:

```bash
cd ~/catkin_ws
source ./devel/setup.bash
catkin_make run_tests
```


## Recording with rosbag
agv_demo.launch file supports recording topics in Gazebo simulation (except /camera/*).  This can be
done by specifying "rec" argument. By default, recording is disabled.

To enable rosbag recording, follow the below steps:

```bash
cd ~/catkin_ws
source ./devel/setup.bash
roslaunch smartagv agv_demo.launch rec:=true
```

To inspect rosbag recording result:

```bash
cd ~/catkin_ws/src/smart_agv/results/
rosbag info session.bag
```

> NOTE: The file can be large even without recording the camera. This is the sole reason one cannot find it in the repository.

## Known issues
1. Limited locations

Currently this AGV can only support traversing to predefined locations mentioned above. One has to manually inspect the new location and and has to determine new locations coordinates and orientation variables.

2. Obstacles

Currently, this robot can be deployed in known obstacle environement. However, it does not support dynamic obstacle avoidance yet. The global planner A* which is used by this AGV for traversing, can only avoid static obstacles. In addition to that, DWA local planner has also issues regarding dynamic obstacle avoidance.

## Documentation

Doxygen style documentation can be found [here](https://github.com/shivaang12/smart_agv/blob/master/documentation/html/index.html)
