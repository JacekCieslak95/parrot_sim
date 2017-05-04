/*
 * parrot_sim_header.h
 *
 *  Created on: 18.04.2017
 *      Author: Jacek Cieślak
 */

#ifndef SRC_PARROT_SIM_HEADER_H_
#define SRC_PARROT_SIM_HEADER_H_

#include "ros/ros.h"
#include <string>
//includy do symulacji
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <sstream>
#include <math.h>
#include <iostream>
#include "sensor_msgs/Range.h"
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
#include <string>     // std::string, std::to_string

//definicje
#define PI 3.14159
#define min_alt 0.60
//definicje danych dotyczących planszy
#define pattern_colls 4
#define pattern_rows 7
#define bok_kwad 8.0
#define l_punktow (pattern_colls*pattern_rows)


using namespace cv;
using namespace std;

//Funkcje CallBack
void imageCb(const sensor_msgs::ImageConstPtr& msg);
void altCb(const sensor_msgs::Range& msg);

//Funkcje rysujące + wpisywanie parametrów szachownicy
void drawGrid(Mat &imgThresholded);
void writeMsg(Mat &imgThresholded);
void chessboardParam();

//funkcje konwertujące deg-rad
double deg2rad(double angle_in_degrees);
double rad2deg(double angle_in_radians);

//funkcje odpowiedzialne za kontrole prędkości
void findControl(cv_bridge::CvImagePtr &cv_ptr);
geometry_msgs::Twist setVelocity(bool objectFound);

//klasa regulatora PD
class PD
{
private:
	double Kp, Kd, dt, max, min, preset;
	double derivative,  pre_error, error;
	double calculate();
public:
	PD(double new_preset);
	PD( double new_Kp, double new_Kd, double new_dt, double new_max, double new_min, double new_preset);
	double getCurrentControl(double current_value);
    void setPreset(double new_preset);
    ~PD();
};



#endif /* SRC_PARROT_SIM_HEADER_H_ */
