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

/** @file navigation.cpp
 *  @brief Implementation of class Navigation
 *
 *  This file contains implementation of class Navigation which navigates robot/AGV to 
 *  pre-defined locations and controls robot's movements to go forward, backward,
 *  turn left or right, or stop.
 *
 *  @author Shivang Patel
*/

#include <ros/ros.h>
#include <ros/timer.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <iostream>
#include <smartAGV/navigation.hpp>

void Navigation::initialize(ros::NodeHandle &n) {
  ROS_INFO_STREAM("Navigation::initialize");
  movebaseCmdVelPub =
      n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);
}

void Navigation::goTO(geometry_msgs::Pose &goal) {
  move_base_msgs::MoveBaseGoal mbGoal;

  mbGoal.target_pose.header.frame_id = "map";
  mbGoal.target_pose.pose = goal;
  mbClient.sendGoal(mbGoal,
                boost::bind(&Navigation::movebaseCallback, this, _1, _2),
                MoveBaseClient::SimpleActiveCallback());
  return;
}

void Navigation::abortMove(void) {
  ROS_INFO_STREAM("cancel moving to goal");
  mbClient.cancelGoal();
  return;
}

void Navigation::movebaseCallback(
    const actionlib::SimpleClientGoalState &state,
    const move_base_msgs::MoveBaseResult::ConstPtr &result) {
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO_STREAM("NAVIGATION:: BASE SUCCEEDED TO MOVE TO GOAL");
  }
}
