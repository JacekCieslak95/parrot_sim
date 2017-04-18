/*
 * parrot_sim_header.h
 *
 *  Created on: 18.04.2017
 *      Author: jacek
 */

#ifndef SRC_PARROT_SIM_HEADER_H_
#define SRC_PARROT_SIM_HEADER_H_

#include "ros/ros.h"
//includy do symulacji
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <sstream>
#include <math.h>
#include <iostream>
//includy do obrazu
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cxcore.h>
#include <cv.h>
#include <highgui.h>
#include <math.h>

void imageCb(const sensor_msgs::ImageConstPtr& msg);
void createTrackbars();
void imageFilters(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded, char color);
void drawGrid(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded);
std::vector<cv::Vec3f> circleFinding(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded);
cv::Vec3f findBiggestCircle(std::vector<cv::Vec3f>  circles);
void findControl(cv_bridge::CvImagePtr &cv_ptr, cv::Vec3f biggest);

#endif /* SRC_PARROT_SIM_HEADER_H_ */
