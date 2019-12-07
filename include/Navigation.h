/**
 *  MIT License
 *
 *  Copyright (c) 2019 Arpit Aggarwal Shantam Bajpai
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 */

/**
 *@file       ObstacleDetector.cpp
 *@author     Shantam Bajpai
 *@copyright  MIT License
 *@brief      Describes the Navigation class.
 */
#ifndef INCLUDE_NAVIGATION_H_
#define INCLUDE_NAVIGATION_H_

#include <geometry_msgs/Twist.h>
#include <ObstacleDetector.h>

class Navigation {
 public:
  /*
   *@brief: Constructor for the Navigation class
   */
  Navigation();

  /*
   * @brief: method to rotate or move the turtlebot linearly
   * @param: detect- Is a boolean variable which is 1 when the obstacle is detected and 0 when
   *         the obstacle is not in range
   */
  void move(bool detect);

  /*
   * @brief: Destructor for the Navigation class
   */
  ~Navigation();

 private:
  // Variable of type ros::NodeHandle
  ros::NodeHandle nh;

  // Variable of type ros::Publisher
  ros::Publisher pubNav;

  // Object to extract linear and angular velocities
  geometry_msgs::Twist msg;

  // Object of ObstacleDetector class
  ObstacleDetector obstacle;

};

#endif // _INCLUDE_NAVIGATION_H_
