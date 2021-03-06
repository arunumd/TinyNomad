/**************************************************************************************************
* @file        NavigationTest.cpp
* @author      Arun Kumar Devarajulu
* @date        December 15, 2018
* @copyright   2018, Arun Kumar Devarajulu
* @license     MIT License
*
*              Permission is hereby granted, free of charge, to any person obtaining a copy
*              of this software and associated documentation files (the "Software"), to deal
*              in the Software without restriction, including without limitation the rights
*              to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*              copies of the Software, and to permit persons to whom the Software is
*              furnished to do so, subject to the following conditions:
*
*              The above copyright notice and this permission notice shall be included in all
*              copies or substantial portions of the Software.
*
*              THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*              IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*              FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*              AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*              LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*              OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
*              SOFTWARE.
*
* @brief       The NavigationTest.cpp file contains all the test cases for the Navigation class
****************************************************************************************************/
#include <gtest/gtest.h>
#include "../include/Navigation.h"
#include "ros/ros.h"

/**
*@brief   Unit test to check if the linear velocity is constant
*/
TEST(NavigationTest, LinearTest) {
    Navigation nav = Navigation();
    EXPECT_EQ(1.0, nav.moveCommand().linear.x);
    EXPECT_EQ(0.0, nav.moveCommand().linear.y);
    EXPECT_EQ(0.0, nav.moveCommand().linear.z);
    EXPECT_EQ(0.0, nav.moveCommand().angular.x);
    EXPECT_EQ(0.0, nav.moveCommand().angular.y);
    EXPECT_EQ(0.0, nav.moveCommand().angular.z);
}

/**
*@brief   Unit test to check if the turn velocity is positive
*/
TEST(NavigationTest, TurnTest) {
    Navigation nav = Navigation();
    EXPECT_EQ(0.0, nav.turnCommand().linear.x);
    EXPECT_EQ(0.0, nav.turnCommand().linear.y);
    EXPECT_EQ(0.0, nav.turnCommand().linear.z);
    EXPECT_EQ(0.0, nav.turnCommand().angular.x);
    EXPECT_EQ(0.0, nav.turnCommand().angular.y);
    EXPECT_LE(1.0, nav.turnCommand().angular.z);
}

/**
*@brief   Unit test to check callback function
*/
TEST(NavigationTest, LaserTest) {
    Navigation nav = Navigation();
    ros::NodeHandle n;
    ros::Subscriber laserSub =
        n.subscribe("/scan", 1000,
                    &Navigation::laserCallback, &nav);
    EXPECT_EQ(0.0, nav.getObstacleRange());
}
