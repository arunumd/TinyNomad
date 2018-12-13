/**************************************************************************************************
* @file      : Navigation.h
* @author    : Arun Kumar Devarajulu
* @date      : December 9, 2018
* @copyright : 2018, Arun Kumar Devarajulu
* @license   : MIT License
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
* @brief     : The Navigation.h file has class declarations for instructing turtlebot to move
*              straight when there are no obstacles while performing a random turn every 60 seconds;
*              and also make an obstacle evasion manuevre when there is an obstacle. The class uses
*              ros::Timer class and several callback methods for accomplishing its goals.
****************************************************************************************************/
#pragma once
#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <random>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

class Navigation {
 public:
    /**
    *@brief  : Default constructor
    */
    Navigation() {}

    /**
    *@brief  : Default destructor
    */
    ~Navigation() {}

    /**
    *@brief  : Command for normal forward movement of turtlebot
    *@return : drivePower to turtlebot
    */
    geometry_msgs::Twist moveCommand();

    /**
    *@brief  : Command for random turn manuevre of turtlebot
    *@return : drivePower to turtlebot
    */
    geometry_msgs::Twist turnCommand();

    /**
    *@brief  : Command for stopping the movement of turtlebot
    *@return : drivePower to turtlebot
    */
    geometry_msgs::Twist stopCommand();

    /**
    *@brief  : Callback method for identifying the approach distance
    */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& data);

 private:
    /**
    *@brief : Node handle at the beginning
    */
    ros::NodeHandle n;

    /**
    *@brief : Variable for storing obstacle range
    */
    float obstacleRange;

    /**
    *@brief : Timer object for obstacle evasion of turtlebot
    */
    ros::Timer normalTurnTimer;

    /**
    *@brief : Timer object for normal driving of turtlebot
    */
    ros::Timer driveTimer;

    /**
    *@brief : Timer object for periodic turning of turtlebot
    */
    ros::Timer periodicTurnTimer;

    /**
    *@brief : Message object for sending command velocity to turtlebot
    */
    geometry_msgs::Twist drivePower;
};

#endif // NAVIGATION_H_
