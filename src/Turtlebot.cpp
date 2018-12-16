/**************************************************************************************************
* @file      : Turtlebot.cpp
* @author    : Arun Kumar Devarajulu
* @date      : December 12, 2018
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
* @brief     : The Turtlebot.cpp class has the implementation for the Turtlebot class
****************************************************************************************************/
#include "Navigation.h"
#include "Turtlebot.h"
#include "Vision.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include <iostream>
#include <ostream>

Turtlebot::Turtlebot() {
	laserData = n.subscribe("/scan", 1000, &Navigation::laserCallback, &nomad);
	velPub = n.advertise<geometry_msgs::Twist> ("/mobile_base/commands/velocity", 100);
	cameraData = n.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 100, &Vision::cameraCallback, &camera);
}

Turtlebot::~Turtlebot() {}

int Turtlebot::drive() {
	auto dist = nomad.getObstacleRange();
	ROS_INFO_STREAM("The distance to obstacle is : " << dist);
	if (dist > 1.50) {
		velPub.publish(nomad.moveCommand());
		ROS_INFO_STREAM("Move command working !");
		return 1;
	} else {
		velPub.publish(nomad.turnCommand());
		ROS_INFO_STREAM("Turn command working !");
		return 2;
	}
}
