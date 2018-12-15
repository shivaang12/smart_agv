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

/** @file test_smartBOT.cpp
 *  @brief Implementation of unit test for SmartBOT class
 *
 *  This file contains implemetation of unit test to class SmartBOT.
 *
 *  @author Shivang Patel
*/

#include <stdlib.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <gtest/gtest.h>
#include <sstream>
#include <boost/thread.hpp>
#include "test_class.hpp"
#include <smartAGV/smartBOT.hpp>

void threadSpinning(void) {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

TEST(TestServiceBot, testDefault) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;

  ros::Rate loop_rate(1);

  // register to check number of publishers to /servicebot/command topic
  ros::Subscriber sub = n.subscribe("/smartbot/command", 1000,
                                    &TestClass::TestAgvCallback, &testVar);

  // register to check number of subscribers to /servicebot/command
  ros::Publisher pub =
      n.advertise<std_msgs::String>("/smartbot/command", 1000);

  loop_rate.sleep();

  EXPECT_NE(0, sub.getNumPublishers());
  EXPECT_NE(0, pub.getNumSubscribers());

  ros::ServiceClient client =
      n.serviceClient<smartagv::agvservice>("agvService");

  // expect service to be ready in 1 second
  EXPECT_EQ(true, client.waitForExistence(ros::Duration(1)));
}

TEST(TestServiceBot, testAgvServiceCmd) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;

  ros::Rate loop_rate(1);

  // register to check number of publishers to /servicebot/command topic
  ros::Subscriber sub = n.subscribe("/smartbot/command", 1000,
                                    &TestClass::TestAgvCallback, &testVar);
  loop_rate.sleep();

  ros::ServiceClient client =
      n.serviceClient<smartagv::agvservice>("agvService");

  // expect service to be ready in 1 second
  EXPECT_EQ(true, client.waitForExistence(ros::Duration(1)));

  smartagv::agvservice::Request req;
  smartagv::agvservice::Response res;

  req.cmd = "test";
  req.arg = "";

  EXPECT_TRUE(client.call(req, res));

  EXPECT_STREQ("OK", res.resp.c_str());

  loop_rate.sleep();

  EXPECT_STREQ("test", testVar.cmd.c_str());
}

TEST(TestServiceBot, testMoveToCommand) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;

  ros::Rate loop_rate(2);

  // Register to publish topic
  ros::Publisher pub =
      n.advertise<std_msgs::String>("/smartbot/command", 1000);

  // Subscribe to check goal and cancel commands received by movebase
  ros::Subscriber MBsubGoal =
      n.subscribe("/move_base/goal", 1000,
                  &TestClass::TestMoveBaseGoal, &testVar);

  ros::Subscriber MBsubCancel =
      n.subscribe("/move_base/cancel", 1000,
                  &TestClass::TestMoveBaseCancelGoal, &testVar);

  loop_rate.sleep();

  ROS_DEBUG_STREAM(
      "Test subMBGoal number of publisher = " << MBsubGoal.getNumPublishers());
  ROS_DEBUG_STREAM(
      "Test subMBCancel number of publisher = " << MBsubCancel.getNumPublishers());

  // test move to room a
  msg.data = "move to,storage";
  pub.publish(msg);

  loop_rate.sleep();

  geometry_msgs::Pose varGoal;
  varGoal.position.x = -1.38;
  varGoal.position.y = 4.43;
  varGoal.position.z = 0.0;
  varGoal.orientation.x = 0.0;
  varGoal.orientation.y = 0.0;
  varGoal.orientation.z = 0.73;
  varGoal.orientation.w = 0.68;

  geometry_msgs::Pose actGoal = testVar.BotPos;

  // confirm goal pose is expected
  EXPECT_EQ(0, std::memcmp(&varGoal, &actGoal, sizeof(varGoal)));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_agv_smartbot");
  ::testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle Nh;
  boost::thread th(threadSpinning);

  int rturn = RUN_ALL_TESTS();

  ros::shutdown();

  // wait the second thread to finish
  th.join();

  return rturn;
}
