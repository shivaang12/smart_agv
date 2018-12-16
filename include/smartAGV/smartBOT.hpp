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

/** @file smartBOT.hpp
 *  @brief Definition of class SmartBOT
 *
 *  This file contains definitions of class SmartBOT which receives
 *  commands from console and executes the commands accordingly.
 *
 *  @author Shivang Patel
*/

#pragma once
#include <ros/ros.h>
#include <smartagv/agvservice.h>
#include <std_msgs/String.h>
#include <smartAGV/action.hpp>

/**
 *  @brief Class definition of SmartBOT class
*/
class SmartBOT {
 public:
  /**
   * @brief  A constructor of SmartBOT class
   *
   * @param  none
   *
   * @return none
   */
  SmartBOT() {}
  /**
 * @brief  A Destructor of SmartBOT class
 *
 * @param  none
 *
 * @return none
 */
  ~SmartBOT() {}
  /**
   * @brief  Initialization of SmartBOT class with default params
   *
   * @param[in]  ros::NodeHandle : Ros node handler.
   *
   * @return none
   */
  void initialize(ros::NodeHandle &);

 private:
  ros::Subscriber agvSub;
  ros::Publisher agvPub;
  ros::ServiceServer agvServer;
  Action action;
  ros::NodeHandle nodeHandle;
  /**
   * @brief   Command callback to process commands received.
   * 
   * @param[in]   std_msgs::String::ConstPtr : String containing command.
   * 
   * return  none
   */
  void agvCallback(const std_msgs::String::ConstPtr &);
  /**
   * @brief   agvservice service callback to receive commands from console
   *
   * 
   * @param[in]   smartagv::agvservice::Request : Service request.
   * 
   * @param[in]   smartagv::agvservice::Response : Service response.
   * 
   * return  none
   */
  bool agvService(smartagv::agvservice::Request &,
                  smartagv::agvservice::Response &);
};
