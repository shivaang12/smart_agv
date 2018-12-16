/********************************************************************
 *   MIT License
 *  
 *   Copyright (c) 2018 Shivang Patel
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 ********************************************************************/

/** @file action.cpp
 *  @brief Implementation of class Action
 *
 *  This file contains implementation of class Action which performs actions
 *  requested from console input.
 * 
 *  @author Shivang Patel
*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <smartAGV/navigation.hpp>
#include <smartAGV/action.hpp>
#include <smartAGV/location.hpp>

void Action::initialize(ros::NodeHandle &n) {
  nodeHandle = n;
  nav.initialize(nodeHandle);
}
void Action::execute(int act, const std::string &arg) {
  action = act;

  ROS_INFO_STREAM("Action::" << action << " arg::" << arg);

  navigate(action, arg);
  return;
}
void Action::navigate(int act, const std::string &arg) {
  geometry_msgs::Pose goal;

  struct Location locationA(std::string("rest pos"), 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.950, 0.312);
  struct Location locationB(std::string("storage"), -1.38, 4.43, 0.0, 0.0, 0.0,
                            0.73, 0.68);
  struct Location locationC(std::string("shop"), 3.60, 4.07, 0.0, 0.0, 0.0,
                            0.80, 0.59);

  std::vector<Location> locations;
  locations.push_back(locationA);
  locations.push_back(locationB);
  locations.push_back(locationC);

  if (action == ACT_MOVETO) {
    for (int i = 0; i < locations.size(); ++i) {
      if (arg.find(locations[i].loc) != std::string::npos) {
        ROS_INFO_STREAM("Action:: navigate move to " << locations[i].loc);
        goal.position.x = locations[i].pointX;
        goal.position.y = locations[i].pointY;
        goal.position.z = locations[i].pointZ;
        goal.orientation.x = locations[i].orientationX;
        goal.orientation.y = locations[i].orientationY;
        goal.orientation.z = locations[i].orientationZ;
        goal.orientation.w = locations[i].orientationW;
        nav.goTO(goal);
        break;
      }
    }
  } else {
    nav.abortMove();
  }

  return;
}
