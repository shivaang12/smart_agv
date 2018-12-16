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

/** @file test_navigation.cpp
 *  @brief Implementation of unit test for SmartBOT class
 *
 *  This file contains implemetation of unit test of class SmartBOT.
 *
 *  @author Shivang Patel
*/

#include <actionlib_msgs/GoalID.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdlib.h>
#include <iostream>
#include <boost/thread.hpp>
#include <smartAGV/navigation.hpp>
#include "test_class.hpp"

void threadSpinning(void) {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

TEST(TestNavigation, testDefault) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Navigation nav;

  ros::Rate loop_rate(2);
  nav.initialize(n);

  auto sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                         &TestClass::TestMoveBaseVelocity, &testVar);
  loop_rate.sleep();
  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST(TestNavigation, testGoTOFunction) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Navigation nav;

  ros::Rate loop_rate(2);
  nav.initialize(n);
  std::cout << "before sub" << '\n';
  ros::Subscriber MBsubGoal = n.subscribe(
      "/move_base/goal", 1000, &TestClass::TestMoveBaseGoal, &testVar);
  ros::Subscriber MBsubCancel = n.subscribe(
      "/move_base/cancel", 1000, &TestClass::TestMoveBaseCancelGoal, &testVar);

  ROS_DEBUG_STREAM(
      "TEST MBGoal number of publisher = " << MBsubGoal.getNumPublishers());
  ROS_DEBUG_STREAM(
      "TEST MBCancel number of publisher = " << MBsubCancel.getNumPublishers());

  geometry_msgs::Pose varGoal;
  varGoal.position.x = 5;
  varGoal.position.y = 5;
  varGoal.position.z = 0.0;
  varGoal.orientation.x = 0.0;
  varGoal.orientation.y = 0.0;
  varGoal.orientation.z = 0.950;
  varGoal.orientation.w = 0.312;

  nav.goTO(varGoal);
  loop_rate.sleep();
  geometry_msgs::Pose goalPos = testVar.BotPos;

  EXPECT_EQ(0, std::memcmp(&varGoal, &goalPos, sizeof(varGoal)));
}

TEST(TestNavigation, testAbortMoveFunction) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Navigation nav;

  ros::Rate loop_rate(2);
  ros::Subscriber MBsubGoal = n.subscribe(
      "/move_base/goal", 1000, &TestClass::TestMoveBaseGoal, &testVar);
  ros::Subscriber MBsubCancel = n.subscribe(
      "/move_base/cancel", 1000, &TestClass::TestMoveBaseCancelGoal, &testVar);

  ROS_DEBUG_STREAM(
      "TEST MBGoal number of publisher = " << MBsubGoal.getNumPublishers());
  ROS_DEBUG_STREAM(
      "TEST MBCancel number of publisher = " << MBsubCancel.getNumPublishers());

  geometry_msgs::Pose varGoal;
  varGoal.position.x = -5;
  varGoal.position.y = -5;
  varGoal.position.z = 0.0;
  varGoal.orientation.x = 0.0;
  varGoal.orientation.y = 0.0;
  varGoal.orientation.z = 0.95;
  varGoal.orientation.w = 0.1;

  nav.goTO(varGoal);
  loop_rate.sleep();
  nav.abortMove();
  loop_rate.sleep();
  EXPECT_STREQ(testVar.goalID.c_str(), testVar.cancelID.c_str());
}

TEST(TestNavigation, testCallback) {
  ros::NodeHandle n;
  Navigation nav;
  actionlib::SimpleClientGoalState state =
      actionlib::SimpleClientGoalState::SUCCEEDED;
  move_base_msgs::MoveBaseResult::ConstPtr result = NULL;
  nav.initialize(n);
  nav.movebaseCallback(state, result);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_agv_navigation");
  ::testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle nodeHand;
  boost::thread th(threadSpinning);

  int rturn = RUN_ALL_TESTS();

  ros::shutdown();

  // wait the second thread to finish
  th.join();

  return rturn;
}
