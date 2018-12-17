/**************************************************************************************************
* @file        Vision.h
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
* @brief       The Vision.h class is used for accessing images taken by turtlebot's RGB camera
****************************************************************************************************/
#pragma once
#ifndef VISION_H_
#define VISION_H_

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include "ros/ros.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Vision {
 public:
    /**
    *@brief   Default constructor
    */
    Vision();

    /**
    *@brief   Default destructor
    */
    ~Vision();

    /**
    *@brief   Callback function for turtlebot camera subscriber
    *@param   picture is the picture coming from the robot 
    */
    void cameraCallback(const sensor_msgs::ImageConstPtr& picture);
};

#endif
