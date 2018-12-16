/**************************************************************************************************
* @file      : Vision.cpp
* @author    : Arun Kumar Devarajulu
* @date      : December 15, 2018
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
* @brief     : The Vision.cpp file has implementation details for the Vision class. The vision
*              class is used for subscribing to the turtle bot camera and save the image to a
*              folder location using opencv and ros bridging
****************************************************************************************************/
#include "Turtlebot.h"
#include "Vision.h"
#include "ros/ros.h"
#include <iostream>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

Vision::Vision() {}

Vision::~Vision() {}

void Vision::cameraCallback(const sensor_msgs::ImageConstPtr& picture) {
    cv_bridge::CvImagePtr picPtr;
    try {
        picPtr = cv_bridge::toCvCopy(picture, "bgr8");
    } catch (cv_bridge::Exception& exp) {
        ROS_ERROR_STREAM("There was a cv_bridge connection problem : " << exp.what());
        return;
    }

    // Let's now proceed to sequentially save all the pictures in a drive location
    std::ostringstream fileName;
    fileName << "Sanpshot taken at " << ros::WallTime::now() << ".jpg";
    cv::imwrite(fileName.str(), picPtr->image);
}