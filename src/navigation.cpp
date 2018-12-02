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
#include <smartAGV/navigation.hpp>

void Navigation::intialize(ros::NodeHandle &Nh) {}

void Navigation::goTO(geometry_msgs::Pose) {}

void Navigation::abortMove(void) {}

void Navigation::forward(void) {}

void Navigation::back(void) {}

void Navigation::left(void) {}

void Navigation::right(void) {}

void Navigation::stop(void) {}

void Navigation::movebaseCallback(
    const actionlib::SimpleClientGoalState &goal,
    const move_base_msgs::MoveBaseResult::ConstPtr &mbPtr
  ) {}

void Navigation::odomCallBack(const nav_msgs::Odometry::ConstPtr &odomPtr) {}

void Navigation::timerCallback(const ros::TimerEvent &timeEvent) {}

double Navigation::convert2degree(double) {}
