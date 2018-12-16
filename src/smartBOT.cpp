/********************************************************************
 *   MIT License
 *
 *   Copyright (c) 2018 Shivang Patel
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 *deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 *THE
 *  SOFTWARE.
 ********************************************************************/

/** @file smartBOT.cpp
 *  @brief Implementation of class SmartBOT
 *
 *  This file contains implemetation of class SmartBOT which receives
 *  commands from console and executes the commands accordingly.
 *
 *  @author Shivang Patel
*/

#include <ros/ros.h>
#include <smartagv/agvservice.h>
#include <std_msgs/String.h>
#include <smartAGV/action.hpp>
#include <smartAGV/smartBOT.hpp>

void SmartBOT::initialize(ros::NodeHandle &n) {
  ROS_INFO_STREAM("smartBot::initialized");
  nodeHandle = n;
  agvSub = nodeHandle.subscribe("/smartbot/command", 1000,
                                &SmartBOT::agvCallback, this);

  agvPub = nodeHandle.advertise<std_msgs::String>("/smartbot/command", 1000);

  agvServer =
      nodeHandle.advertiseService("agvService", &SmartBOT::agvService, this);

  action.initialize(nodeHandle);

  return;
}

void SmartBOT::agvCallback(const std_msgs::String::ConstPtr &msg) {
  std::string cmd;
  std::string args;
  std::istringstream lineStream(msg->data.c_str());

  // parse string message into command and arguments
  getline(lineStream, cmd, ',');
  getline(lineStream, args, ',');

  // convert command and args to lower case
  std::transform(cmd.begin(), cmd.end(), cmd.begin(), ::tolower);
  std::transform(args.begin(), args.end(), args.begin(), ::tolower);

  ROS_INFO_STREAM("ServiceBot::commandCallback cmd=" << cmd
                                                     << " args=" << args);

  if ((cmd.find("move to") != std::string::npos) ||
      (cmd.find("go to") != std::string::npos)) {
    if (args.empty()) {
      args = cmd;
    }
    action.execute(Action::ACT_MOVETO, args);
  } else if ((cmd.find("stop moving") != std::string::npos) ||
             (cmd.find("abort") != std::string::npos) ||
             (cmd.find("cancel") != std::string::npos)) {
    action.execute(Action::ACT_STOPMOVETO);
  } else {
    ROS_INFO_STREAM("ServiceBot:: unknown action");
  }

  return;
}

bool SmartBOT::agvService(smartagv::agvservice::Request &req,
                          smartagv::agvservice::Response &resp) {
  std_msgs::String msg;

  ROS_INFO_STREAM("ServiceBot::commandService: receive req = " << req.cmd << " "
                                                               << req.arg);

  std::stringstream ss;
  ss << req.cmd << "," << req.arg;
  msg.data = ss.str();

  // send messages
  agvPub.publish(msg);

  resp.resp = "OK";

  return true;
}
