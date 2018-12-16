/**
 * Copyright 2018 Shivang Patel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**@file test_class.cpp
 * @brief Source file for TestClass class
 *
 * This file contains implemetation of TestClass class which provides certain
 * predefined testing functionlities for this project's classes.
 *
 * @author Shivang Patel
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <algorithm>
#include "test_class.hpp"

void TestClass::TestAgvCallback(const std_msgs::String::ConstPtr &msg) {
    std::istringstream commandLine(msg->data.c_str());
    getline(commandLine, cmd, ',');
    getline(commandLine, arg, ',');

    return;
}

void TestClass::TestMoveBaseGoal(
    const move_base_msgs::MoveBaseActionGoal::ConstPtr &msg) {
    goalID = msg->goal_id.id;
    BotPos = msg->goal.target_pose.pose;

    ROS_DEBUG_STREAM("TEST AGV's position");
    ROS_DEBUG_STREAM("x " << BotPos.position.x);
    ROS_DEBUG_STREAM("y " << BotPos.position.y);
    ROS_DEBUG_STREAM("z " << BotPos.position.z);
    ROS_DEBUG_STREAM("TEST AGV's orientation");
    ROS_DEBUG_STREAM("x " << BotPos.orientation.x);
    ROS_DEBUG_STREAM("y " << BotPos.orientation.y);
    ROS_DEBUG_STREAM("z " << BotPos.orientation.z);
    ROS_DEBUG_STREAM("w " << BotPos.orientation.w);
    return;
}

void TestClass::TestMoveBaseCancelGoal(
    const actionlib_msgs::GoalID::ConstPtr &msg) {
    cancelID = msg->id;

    ROS_DEBUG_STREAM("TEST cancelID = " << cancelID);
    return;
}

void TestClass::TestMoveBaseVelocity(
  const geometry_msgs::Twist::ConstPtr &msg) {
    twist = *msg;

    ROS_DEBUG_STREAM("TEST twist linear x = " << twist.linear.x);
    ROS_DEBUG_STREAM("TEST twist angular z = " << twist.angular.z);
    return;
}
