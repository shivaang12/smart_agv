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

/** @file test_action.cpp
 *  @brief Implementation of unit test for Action class
 *
 *  This file contains implemetation of unit test to class Action.
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
#include <smartAGV/action.hpp>

void threadSpinning(void) {
  ros::Rate loop_rate(10);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

TEST(TestAction, testDefault) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Action act;

  ros::Rate loop_rate(2);

  act.initialize(n);

  // register to check number of publishers to
  // /mobile_base/commands/velocity topic
  ros::Subscriber sub = n.subscribe("/mobile_base/commands/velocity", 1000,
                                    &TestClass::TestMoveBaseVelocity, &testVar);


  loop_rate.sleep();

  EXPECT_EQ(1, sub.getNumPublishers());
}

TEST(TestAction, testGoTOAction) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Action act;

  ros::Rate loop_rate(2);

  auto MBsubGoal = n.subscribe("/move_base/goal", 1000,
                               &TestClass::TestMoveBaseGoal, &testVar);

  auto MBsubCancel = n.subscribe("/move_base/cancel", 1000,
                               &TestClass::TestMoveBaseCancelGoal, &testVar);

  act.execute(Action::ACT_MOVETO, "storage");

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

    // confirm goal pose received by move_base is expected
    EXPECT_EQ(0, std::memcmp(&varGoal, &actGoal, sizeof(varGoal)));
}

TEST(TestAction, testAbortMove) {
  ros::NodeHandle n;
  TestClass testVar;
  std_msgs::String msg;
  Action act;

  ros::Rate loop_rate(2);

  auto MBsubGoal = n.subscribe("/move_base/goal", 1000,
                               &TestClass::TestMoveBaseGoal, &testVar);

  auto MBsubCancel = n.subscribe("/move_base/cancel", 1000,
                               &TestClass::TestMoveBaseCancelGoal, &testVar);

  act.execute(Action::ACT_MOVETO, "storage");

  loop_rate.sleep();

  act.execute(Action::ACT_STOPMOVETO);

  loop_rate.sleep();

  EXPECT_STREQ(testVar.goalID.c_str(), testVar.cancelID.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_agv_action");
  ::testing::InitGoogleTest(&argc, argv);
  ros::NodeHandle Nh;
  boost::thread th(threadSpinning);

  int rturn = RUN_ALL_TESTS();

  ros::shutdown();

  // wait the second thread to finish
  th.join();

  return rturn;
}
