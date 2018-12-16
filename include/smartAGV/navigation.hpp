/********************************************************************
 *   MIT License
 *
 *   Copyright (c) 2018 Shivang Patel
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 *deal in the Software without restriction, including without limitation the
 *rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 *sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 *IN THE SOFTWARE.
 ********************************************************************/

/** @file navigation.hpp
 *  @brief Definition of class Navigation
 *
 *  This file contains definitions of class Navigation which navigates robot/AGV
 * to pre-defined locations and controls robot's movements to go forward,
 * backward, turn left or right, or stop.
 *
 *  @author Shivang Patel
 */
#pragma once
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/timer.h>

/**
 *  @typedef MoveBaseClient
 *
 *  @brief Actionlib client of Mobilebase
 */
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

/**
 *  @brief Class definition of Navigation class
 */
class Navigation {
 public:
  /**
   * @brief  Constructor of Navigation class
   *
   * @param[in]  none
   * 
   * @return none
   */
  Navigation() : mbClient("move_base", true) {}
  /**
   * @brief  This method intializes the default parametres for the classes.
   *
   * @param[in]  none
   * 
   * @return none
   */
  void initialize(ros::NodeHandle &);
  /**
   * @brief  Send goal to movebase to navigate robot to goal location.
   * 
   * param[in]  geometry_msgs::Pose : Goal position.
   * 
   * @return none
   */
  void goTO(geometry_msgs::Pose &);
  /**
   * @brief  This methods helps to abort the current goal task navigation.
   * 
   * param[in]  none
   * 
   * @return none
   */
  void abortMove(void);
  /**
   * @brief  This is as Callback function to receive navigation result from
   * movebase.           
   * 
   * param[in]  actionlib::SimpleClientGoalState : Goal state.
   * param[in]  move_base_msgs::MoveBaseResult::ConstPtr : result in move base.
   * @return none
   */
  void movebaseCallback(const actionlib::SimpleClientGoalState &,
                        const move_base_msgs::MoveBaseResult::ConstPtr &);

 private:
  ros::Publisher movebaseCmdVelPub;   //> Ros publisher
  MoveBaseClient mbClient;            //> Move base client
};
