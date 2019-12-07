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
 *@copyright  MIT License
 *@brief      Implements the methods of the ObstacleDetector class.
 */
#include <ros/ros.h>
#include <ObstacleDetector.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

ObstacleDetector::ObstacleDetector() {
    ROS_INFO_STREAM("ObstacleDetector Constructor called.");
    // Set isCollision to false
    isCollision = false;
    // Publish to dist topic
    distPub = node.advertise<std_msgs::Float64>("/dist", 500);
    // Subscribe to dist topic
  distSub = node.subscribe<std_msgs::Float64>("/dist", 500,
                                              &ObstacleDetector::distCallback,
                                              this);
    // Subscribe to scan topic
  sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 500,
                                               &ObstacleDetector::laserCallback,
                                               this);
}

void ObstacleDetector::laserCallback(
    const sensor_msgs::LaserScan::ConstPtr& data) {
    ROS_INFO_STREAM("Callback for /scan topic called.");
    // Loop through the laser scan data
    float minDist = 2000;
  for (const auto& range : data->ranges) {
    if (range < minDist) {
      minDist = range;
    }
    }
    // Create object of type std_msgs::Float64 and publish data to dist topic
    std_msgs::Float64 val;
    val.data = minDist;
    distPub.publish(val);
}

void ObstacleDetector::distCallback(const std_msgs::Float64::ConstPtr& data) {
  ROS_INFO_STREAM("Callback for /dist topic called.");
  // If distance is than 2 then change isCollision flag to true,
  // otherwise make it false
  if (data->data < 2.00) {
    isCollision = true;
  } else {
    isCollision = false;
  }
}

bool ObstacleDetector::getIsCollision() {
    // Return isCollision value
    return isCollision;
}

void ObstacleDetector::setIsCollision(bool collisionVal) {
    // Set isCollision value
    isCollision = collisionVal;
}

ObstacleDetector::~ObstacleDetector() {
}
