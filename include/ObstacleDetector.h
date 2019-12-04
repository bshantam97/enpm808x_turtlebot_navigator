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
 *@file       ObstacleDetector.h
 *@copyright  MIT License
 *@brief      Defines the ObstacleDetector class.
 */

#ifndef TURTLEBOT_NAVIGATOR_INCLUDE_OBSTACLEDETECTOR_H_
#define TURTLEBOT_NAVIGATOR_INCLUDE_OBSTACLEDETECTOR_H_

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>

class ObstacleDetector {
 private:
    // Variable of type ros::NodeHandle
    ros::NodeHandle node;

    // Variable of type ros::Subscriber, subscribes to /scan topic
    ros::Subscriber sub;

    // Variable of type bool
    bool isCollision;

    // Variable of type ros::Subscriber, subscribes to /dist topic
    ros::Subscriber distSub;

    // Variable of type ros::Publisher, published to /dist topic
    ros::Publisher distPub; 

 public:
    /**
      * @brief Constructs the ObstacleDetector object.
    */
    explicit ObstacleDetector();

    /**
      * @brief Destructor of the ObstacleDetector class.
    */
    ~ObstacleDetector();

    /**
     * @brief Callback for ros subscriber sub.
     * @param data sensor_msgs::LaserScan::ConstPtr.
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& data);

    /**
     * @brief Callback for ros subscriber distSub.
     * @param data const std_msgs::Float64::ConstPtr.
    */
    void distCallback(const std_msgs::Float64::ConstPtr& data);
 
    /**
     * @brief Returns the value of isCollision.
    */
    bool getIsCollision();

    /**
     * @brief Returns the value of isCollision.
     * @param collisionVal bool.
    */
    void setIsCollision(bool collisionVal);
};
#endif  // TURTLEBOT_NAVIGATOR_INCLUDE_OBSTACLEDETECTOR_H_
