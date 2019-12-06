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
#include <Navigation.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>


/*
 * @brief: Constructor for the Navigation Class
 */

Navigation::Navigation() {
  // Initialize the publisher to advertise the velocities to the turtlebot
  pubNav = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",
                                              1000);

  // Initializing the linear and angular velocities to be zero
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;

  pubNav.publish(msg);
}

/*
 * @brief: Move method declaration to rotate or make the turtlebot move in a straight line
 * @param: detect- Is a boolean variable which is 1 when the obstacle is detected and 0 when
 *         the obstacle is not in range
 */

void Navigation::move(bool detect) {
  ros::Rate loopRate;
  while (ros::ok()) {
    detect = obstacle.getIsCollision();
    if (detect == true) {
      ROS_WARN_STREAM("The object is in range, Turn !!!");
      msg.linear.x = 0;
      msg.angular.z = 1;
    } else {
      ROS_INFO_STREAM("Keep investigating the area");
      msg.angular.z = 0;
      msg.linear.z = 1;
    }
    pubNav.publish(msg);
    ros::spinOnce();
    loopRate.sleep();
  }
}

/*
 * @brief: Destructor declaration for the Navigation class
 */

Navigation::~Navigation() {
  ROS_INFO("Destructor being invoked: linear and  angular velocities are zero");

  // Set the linear and angular velocities to be zero
  msg.linear.x = 0;
  msg.linear.y = 0;
  msg.linear.z = 0;
  msg.angular.x = 0;
  msg.angular.y = 0;
  msg.angular.z = 0;

  // Publish the message
  pubNav.publish(msg);
}
