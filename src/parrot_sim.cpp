#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include <sstream>
#include <math.h>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "parrot_sim");
	ros::NodeHandle n;

	cout << "start" <<endl;

	return 0;
}
