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

/** @file smartBOT.hpp
 *  @brief Definition of class SmartBOT
 *
 *  This file contains definitions of class SmartBOT which receives  
 *  commands from console and executes the commands accordingly.
 *
 *  @author Shivang Patel
*/

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <smartAGV/agvservice.h>
#include <smartAGV/action.hpp>

class SmartBOT {
 public:
  SmartBOT(){}
  ~SmartBOT(){}
  void initialize(ros::NodeHandle &);

 private:
  ros::Subscribe agvSub;
  ros::Publisher agvPub;
  ros::ServiceServer agvServer;
  Action action;
  ros::NodeHandle nodeHandle;
  void agvCallback(const std_msgs::String::ConstPtr &);
  bool agvService(
    smartAGV::agvservice::Request &,
    smartAGV::agvservice::Response &
  );
};
