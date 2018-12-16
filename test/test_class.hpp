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

/**@file test_class.hpp
 * @brief Header file TestClass class
 *
 * This file contains declaration of TestClass class which provides certain
 * predefined testing functionlities for this project's classes.
 *
 * @author Shivang Patel
 */

#pragma once
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <string>

/**
 * @brief Class definition for TestClass class.
 */

class TestClass {
 public:
  void TestAgvCallback(const std_msgs::String::ConstPtr &);
  void TestMoveBaseGoal(const move_base_msgs::MoveBaseActionGoal::ConstPtr &);
  void TestMoveBaseCancelGoal(const actionlib_msgs::GoalID::ConstPtr &);
  void TestMoveBaseVelocity(const geometry_msgs::Twist::ConstPtr &);
  std::string cmd;
  std::string arg;
  std::string goalID;
  std::string cancelID;
  geometry_msgs::Pose BotPos;
  geometry_msgs::Twist twist;
};
