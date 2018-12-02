# Smart AGV
[![Build Status](https://travis-ci.org/shivaang12/Smart-AGV.svg?branch=master)](https://travis-ci.org/shivaang12/Smart-AGV)[![Coverage Status](https://coveralls.io/repos/github/shivaang12/Smart-AGV/badge.svg?branch=master)](https://coveralls.io/github/shivaang12/Smart-AGV?branch=master)

## Overview
This is an smart AGV (Automated Guided Vehicle) project which can take commands from the end user, and navigate autonomously to perform a required task. This AGV is implemented in ROS Gazebo simulation environment using the TurtleBot as the base platform. A custom global planner was written as a plugin, and uses the A* algorithm for planning. SLAM was done prior on the environment to get a map for navigation. 

## Author
- Shivang Patel

## License
MIT open-source license, see [license](https://opensource.org/licenses/MIT)

## Dependencies (for now)
- Ubuntu 16.04
- ROS Kinetic Kame
- TurtleBot_Gazebo
- Package Dependencies
    - roscpp
    - std_msgs
    - message_generation
    - geometry_msgs

## SIP

SIP process chart can be access [here](https://docs.google.com/spreadsheets/d/15FUiV2CVoGEMxKz0mKP8EMVmY0W706Poa8y_qtE8bmY/edit?usp=sharing)

## Sprint Notes

Click [here](https://docs.google.com/document/d/1l9LacTmeBo5nzlTT7CSJo50k2eist533l3HIxqJ_3jI/edit?usp=sharing) to access sprint notes.

## Build

## Run/Demo

## Testing with rostest

## Recording with rosbag

## Known issues

## Documentation