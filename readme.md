# Smart AGV

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
