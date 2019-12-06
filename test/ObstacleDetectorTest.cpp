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
 *@file       ObstacleDetectorTest.cpp
 *@copyright  MIT License
 *@brief      Test cases for ObstacleDetector.cpp file.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include <ObstacleDetector.h>
#include <iostream>

/**
 * @brief      Tests the object creation of the class ObstacleDetector
 * @param      ObstacleDetectorTest     gtest framework
 * @param      ObstacleDetectorTest     Name of the test
 * @return     none
 */
TEST(ObstacleDetectorTest, ObstacleDetectorTest) {
    // Object of type ObstacleDetector created
    EXPECT_NO_FATAL_FAILURE(ObstacleDetector obstacleDetector);
}

/**
 * @brief      Tests the getIsCollision method of the class ObstacleDetector
 * @param      ObstacleDetectorTest     gtest framework
 * @param      CollisionMethodTest      Name of the test
 * @return     none
 */
TEST(ObstacleDetectorTest, CollisionMethodTest) {
    // Object of type ObstacleDetector created
    ObstacleDetector obstacleDetector;
    obstacleDetector.setIsCollision(false);
    EXPECT_FALSE(obstacleDetector.getIsCollision());
}

/**
 * @brief      Tests the laserCallback method of the class ObstacleDetector
 * @param      ObstacleDetectorTest      gtest framework
 * @param      LaserCallbackMethodTest   Name of the test
 * @return     none
 */
TEST(ObstacleDetectorTest, LaserCallbackMethodTest) {
    // Object of type ObstacleDetector created
    ObstacleDetector obstacleDetector;
    // Create ros node
    ros::NodeHandle node;
    // Create a ros publisher
    ros::Publisher pub = node.advertise<sensor_msgs::LaserScan>("/scan", 50);
    // Create a ros subscriber
    ros::Subscriber sub = node.subscribe<sensor_msgs::LaserScan>("/scan", 50, &ObstacleDetector::laserCallback, &obstacleDetector);

    // Check collision flag
    while(ros::ok()) {
        if(obstacleDetector.getIsCollision() == false) {
            break;
        }
        ros::spinOnce();
    }

    // Expect the collision flag to be false
    EXPECT_FALSE(obstacleDetector.getIsCollision());
}

/**
 * @brief      Tests the distCallback method of the class ObstacleDetector
 * @param      ObstacleDetectorTest      gtest framework
 * @param      DistCallbackMethodTest    Name of the test
 * @return     none
 */
TEST(ObstacleDetectorTest, DistCallbackMethodTest) {
    // Object of type ObstacleDetector created
    ObstacleDetector obstacleDetector;
    // Create ros node
    ros::NodeHandle node;
    // Create a ros publisher
    ros::Publisher pub = node.advertise<std_msgs::Float64>("/dist", 50);
    // Create a ros subscriber
    ros::Subscriber sub = node.subscribe<std_msgs::Float64>("/dist", 50, &ObstacleDetector::distCallback, &obstacleDetector);

    // Check collision flag
    while(ros::ok()) {
        if(obstacleDetector.getIsCollision() == false) {
            break;
        }
        ros::spinOnce();
    }

    // Expect the collision flag to be false
    EXPECT_FALSE(obstacleDetector.getIsCollision());
}
