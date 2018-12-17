/**************************************************************************************************
* @file      : Turtlebot.h
* @author    : Arun Kumar Devarajulu
* @date      : December 11, 2018
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
* @brief     : The Turtlebot.h class is used for publishing velocity messages based on laser scan
*              feedbacks from the Navigation class.
****************************************************************************************************/
#pragma once
#ifndef TURTLEBOT_H_
#define TURTLEBOT_H_

#include "Navigation.h"
#include "Vision.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"

class Turtlebot {
 public:
    /**
    *@brief : Default constructor
    */
    Turtlebot();

    /**
    *@brief : Default destructor
    */
    ~Turtlebot();

    /**
    *@brief : Method to publish velocity messages based on laser range
    */
    int drive();

 private:
    /**
    *@brief : Create an instance of the navigation class
    */
    Navigation nomad = Navigation();
    /**
    *@brief : Create a nodehandle
    */
    ros::NodeHandle n;
    /**
    *@brief : Publisher object for publishing command velocity to turtlebot
    */
    ros::Publisher velPub;
    /**
    *@brief : Subscriber object for subscribing to laser range from turtlebot
    */
    ros::Subscriber laserData;
    /**
    *@brief : Create a vision class object
    */
    Vision camera = Vision();
    /**
    *@brief : Create a subscriber object for turtlebot camera
    */
    ros::Subscriber cameraData;
};

#endif
