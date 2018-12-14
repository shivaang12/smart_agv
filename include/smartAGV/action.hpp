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

/** @file action.hpp
 *  @brief Definition of class Action
 *
 *  This file contains definitions of class Action which performs actions
 *  requested from console input.
 * 
 *  @author Shivang Patel
*/

#pragma once
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <string>
#include <smartAGV/navigation.hpp>
#include <smartAGV/location.hpp>

class Action {
 public:
  enum act { ACT_MOVETO, ACT_STOPMOVETO};
  Action() : action(0) {}
  ~Action() {}
  void intialize(ros::NodeHandle &);
  void execute(int, const std::string &arg);
 
 private:
  int action;
  ros::NodeHandle nodeHandle;
  Navigation naviCtrl;
  void navigate(int, const std::string &arg);
};
