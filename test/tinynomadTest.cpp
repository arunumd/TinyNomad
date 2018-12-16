/**************************************************************************************************
* @file      : tinynomadTest.cpp
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
* @brief     : The tinynomadTest.cpp file is the master test file for running all Google tests for
*              all classes in the project
****************************************************************************************************/
#include <ros/ros.h>
#include <gtest/gtest.h>

/**
 * @brief  : main function to run all Google tests
 * @param  : argc is the no. of input arguments
 * @param  : argv is the array of input arguments
 * @return : 0 if all tests pass else 1
 */
int main(int argc, char **argv) {
  ros::init(argc, argv, "tinynomadTest");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}