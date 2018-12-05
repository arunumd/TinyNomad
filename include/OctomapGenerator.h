/************************************************************************************************
* @file      : Header file for OctomapGenerator class
* @author    : Arun Kumar Devarajulu
* @date      : December 4, 2018
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
* @brief     : The OctomapGenerator.h file subscribes to the laser scan data in turtlebot and
*              dynamically grows an octomap of the environment
**************************************************************************************************/
#ifndef OCTOMAPGENERATOR_H_
#def OCTOMAPGENERATOR_H_
#pragma once

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <string>

class OctomapGenerator {
 public:


 private:
 	/*
 	*@brief  : Nodehandle declaration
 	*/
 	ros::Nodehandle nh;
 	/*
 	*@brief  : Subscriber for laser scan data
 	*/
 	ros::Subscriber laserSub;
 	/*
 	*@brief  : Create a tf listener object for listening to tf
 	*/
 	tf_listener = new tf::TransformListener();
 	/*
 	*@brief  : Create a tf transform object
 	*/
 	tf::StampedTransform transform;
 	/*
 	*@brief  : Create an OcTree object of octomap
 	*/
 	octomap::OcTree* probmap;
}

#endif OCTOMAPGENERATOR_H_
