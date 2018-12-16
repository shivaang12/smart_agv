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

/** @file action.hpp
 *  @brief Definition of class Action
 *
 *  This file contains definitions of class Action which performs actions
 *  requested from console input.
 *
 *  @author Shivang Patel
 */

#pragma once
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <string>
#include <smartAGV/location.hpp>
#include <smartAGV/navigation.hpp>

/**
 * @brief Class definition of Action class
 */
class Action {
 public:
  /**
   * @enum Action
   *
   * @brief Enums for actions which are defined in below enums.
   */
  enum act { ACT_MOVETO, ACT_STOPMOVETO };
  /**
   * @brief  A constructor of Action class
   *
   * @param[in]  none
   *
   * @return none
   */
  Action() : action(0) {}
  /**
   * @brief  A Destructor of Action class
   *
   * @param[in]  none
   *
   * @return none
   */
  ~Action() {}
  /**
   * @brief  Initialization of Action class with default params
   *
   * @param[in]  ros::NodeHandle : Ros node handler.
   *
   * @return none
   */
  void initialize(ros::NodeHandle &);
  /**
   * @brief This method executes commands
   *
   * @param[in] int : Action number.
   * 
   * @param[in] arg : string containing command.
   *
   * @return none
   */ 
  void execute(int, const std::string &arg = "");

 private:
  int action;   //> Action container
  ros::NodeHandle nodeHandle;   //> Ros node handler
  Navigation nav;   //> Navigation class object
  /**
   * @brief This method provides navigation functionalities and uses navigation
   * class' object to do that.
   *
   * @param[in] int : Action number.
   * 
   * @param[in] arg : string containing command.
   *
   * @return none
   */ 
  void navigate(int, const std::string &arg);
};
