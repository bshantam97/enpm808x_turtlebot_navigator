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

#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <../include/Navigation.h>
#include <../include/ObstacleDetector.h>

/**
 * @brief      Tests the object creation of the class Navigation
 * @param      NavigationTest     gtest framework
 * @param      NavigationTest     Name of the test
 * @return     none
 */
TEST(NavigationTest, NavigationTest) {
  // Object of type Navigation created
  EXPECT_NO_FATAL_FAILURE(Navigation nav);
}

/*
 *@brief: Test Case for the move() method
 *@param: NavigationTest  gtest framework
 *@param: moveMethodTest  Name of the test
 */
TEST(NavigationTest , moveMethodTest) {
  Navigation nav;
  EXPECT_NO_FATAL_FAILURE(nav.move(true));
  nav.obstacle.setIsCollision(false);
  EXPECT_NO_FATAL_FAILURE(nav.move(nav.obstacle.getIsCollision()));
}
