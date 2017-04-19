#include "parrot_sim_header.h"



int fontFace = cv::FONT_HERSHEY_DUPLEX;
//początkowe wartości do filtra. Po dobraniu można to wywalić i w ich miejsce wsadziś stałe
int iLowH = 68;				//dobrze działa na 68
int iHighH = 96;			//dobrze działa na 96

int iLowS = 90;				//dobrze działa na 90
int iHighS = 255;			//dobrze działa na 255

int iLowV = 28;				//dobrze działa na 28
int iHighV = 190;			//dobrze działa na 190

int circle_filter=70;		//dobrze działa na 100
int height,width;			//zmienne do przechowywania wymiarów obrazu
int dist_control; //zmienne globalne, do komunikacji obraz - symulator
//( double new_Kp, double new_Kd, double new_dt, double new_max, double new_min, double new_preset)

PD PD_vert(0.01, 0.01, 1, 1, -1, 240);
PD PD_hor(0.01, 0.01, 1, 1, -1, 320);
PD PD_dist(0.05, 0.05, 1, 1, -1, 60);

double vel_lin_x, vel_lin_z, vel_ang_z;

bool found=false;

image_transport::Subscriber image_sub_; //do subskrypcji obrazu
image_transport::Publisher image_pub_;	//do publikacji przerobionego obrazu
ros::Publisher control_pub;


int main(int argc, char** argv)
{
	std::cout<<"Parrot_sim started!"<<std::endl;
	dist_control=0; //przypisanie sterowania
	ros::init(argc, argv, "image_converter");
	ros::NodeHandle nh;
	ros::Publisher image_pub_;
	ros::Publisher control_pub;
	geometry_msgs::Twist vel_msg;
	image_pub_ = nh.advertise<sensor_msgs::Image>("/image_converter/output_video", 1);
	control_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	std::cout << "start" <<std::endl;
	ros::Subscriber image_sub_;
	image_sub_ = nh.subscribe("/camera/image", 1,	imageCb);

	//tworzenie okna i suwaków do kontroli
	createTrackbars();
	cv::namedWindow("Window with detection");
	while(nh.ok())
	{
		vel_msg=setVelocity(found);
		control_pub.publish(vel_msg);
		ros::spinOnce();
	};

	cv::destroyWindow("Window with detection");
	std::cout<<std::endl<<"Parrot_sim closed!"<<std::endl;
	return 0;
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	//przekopiowanie wiadomości do wskaźnika (?)
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
	{

		height = cv_ptr->image.rows;
		width = cv_ptr->image.cols;
		cv::Mat imgThresholded;
		imageFilters(cv_ptr, imgThresholded, 'a'); //nałożenie filtrów i zapisywanie obrazu do "imgThresholded"
		std::vector<cv::Vec3f> circles = circleFinding(cv_ptr, imgThresholded); //przeszukanie obrazu w poszukwaniu kółek i zapisanie do wektora
		drawGrid(cv_ptr, imgThresholded);
		if(!circles.empty())
		{
			found=true;
			cv::Vec3f biggest_circle = findBiggestCircle(circles);
			findControl(cv_ptr,biggest_circle);
		}
		else
		{
			found=false;
			cv::Point info_text_base(0, height-40);
			cv::putText(cv_ptr->image, "no object found!", info_text_base, fontFace, 1, CV_RGB(200,0,0), 1, 8);

		}
		//cv::imshow("Green detection", imgThresholded); 			//Pokaż przefiltrowany obraz
		cv::imshow("Window with detection", cv_ptr->image);		//Pokaż obraz z nałożonymi kołami
		cv::waitKey(3);
		// opublikowanie przerobionego obrazu
		image_pub_.publish(cv_ptr->toImageMsg());
	}
}
void createTrackbars()
{
	cv::namedWindow("Control",  CV_WINDOW_AUTOSIZE);

	cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	cvCreateTrackbar("HighH", "Control", &iHighH, 179);

	cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	cvCreateTrackbar("HighS", "Control", &iHighS, 255);

	cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
	cvCreateTrackbar("HighV", "Control", &iHighV, 255);

	cvCreateTrackbar("Circle filter", "Control", &circle_filter, 255);
}
void imageFilters(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded, char color)
{
	//w późniejszym czasue można ustalić wzorce dla konkretnych kolorów (przekazywanych w arkumencie "char color"
	//wtedy w zależności od przekazywanego znaku ustala się wartości iLowH, iLowS, iLowV, iHighH, iHighS, iHighV
	cv::Mat imgHSV;
	cv::cvtColor(cv_ptr->image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV


	//Nałożenie filtrów ( z suwaków)
	cv::inRange(imgHSV, cv::Scalar(iLowH, iLowS, iLowV), cv::Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	//morphological closing (fill small holes in the foreground)
	cv::dilate( imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
	cv::erode(imgThresholded, imgThresholded, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	//detekcja
	cv::GaussianBlur( imgThresholded, imgThresholded, cv::Size(9, 9), 2, 2 );
}
void drawGrid(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded)
{
	cv::Point left(0, height/2);
	cv::Point right(width, height/2);
	cv::Point top(width/2, 0);
	cv::Point bottom(width/2, height);
	//cv::line(Mat& img, Point pt1, Point pt2, const Scalar& color, int thickness=1, int lineType=8, int shift=0)
	cv::line(cv_ptr->image, left, right, cv::Scalar(160, 160, 160), 0.5);
	cv::line(cv_ptr->image, top, bottom, cv::Scalar(160, 160, 160), 0.5);

	cv::line(imgThresholded, left, right, cv::Scalar(160, 160, 160), 0.5);
	cv::line(imgThresholded, top, bottom, cv::Scalar(160, 160, 160), 0.5);
}
std::vector<cv::Vec3f> circleFinding(cv_bridge::CvImagePtr &cv_ptr, cv::Mat &imgThresholded)
				{
	std::vector<cv::Vec3f> circles;												// wektor przechowujący parametry kółek
	cv::HoughCircles(imgThresholded, circles, CV_HOUGH_GRADIENT,
			2, imgThresholded.rows  /4, 200, circle_filter );					// Wykrywanie kółek
	for( size_t i = 0; i < circles.size(); i++ )								// pętla rysująca kołka, centra, i opisy
	{
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
		int radius = cvRound(circles[i][2]);
		cv::Point text_base(cvRound(circles[i][0]) + radius, cvRound(circles[i][1]) +radius) ;
		// draw the circle center
		cv::circle( cv_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
		// draw the circle outline
		cv::circle( cv_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
	}
	return circles;
				}
cv::Vec3f findBiggestCircle(std::vector<cv::Vec3f> circles)
{
	cv::Vec3f biggest;
	int radius = 0;
	int biggest_index=0;
	for( size_t i = 0; i < circles.size(); i++ )								// pętla rysująca kołka, centra, i opisy
	{
		if (cvRound(circles[i][2])>=radius) biggest_index=i;
	}
	biggest[0]=circles[biggest_index][0];
	biggest[1]=circles[biggest_index][1];
	biggest[2]=circles[biggest_index][2];
	return biggest;
}
void findControl(cv_bridge::CvImagePtr &cv_ptr, cv::Vec3f biggest)
{
	cv::Point hor_text_base(0, height-10);
	cv::Point vert_text_base(0, height-40);
	cv::Point dist_text_base(0, height-70);
	if (biggest[0]< (width/2-(biggest[2]/3)))
	{
		cv::putText(cv_ptr->image, "go right!", hor_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_ang_z=PD_hor.getCurrentControl(biggest[0]);
	}
	else if (biggest[0]> (width/2+(biggest[2]/3)))
	{
		cv::putText(cv_ptr->image, "go left!", hor_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_ang_z=PD_hor.getCurrentControl(biggest[0]);
	}
	else
	{
		cv::putText(cv_ptr->image, "Horizonal - OK", hor_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
		vel_ang_z=0;//PD_hor.getCurrentControl(biggest[0]);
	}

	if (biggest[1]< (height/2-(biggest[2]/3)))
	{
		cv::putText(cv_ptr->image, "go up!", vert_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_lin_z=PD_vert.getCurrentControl(biggest[1]);

	}
	else if (biggest[1]> (height/2+(biggest[2]/3)))
	{
		cv::putText(cv_ptr->image, "go down!", vert_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_lin_z=PD_vert.getCurrentControl(biggest[1]);
	}
	else
	{
		cv::putText(cv_ptr->image, "Vertical - OK", vert_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
		vel_lin_z=0;//PD_vert.getCurrentControl(biggest[1]);
	}

	if (cvRound(biggest[2])<50)
	{
		cv::putText(cv_ptr->image, "too far!", dist_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_lin_x=PD_dist.getCurrentControl(biggest[2]);

	}
	else if (cvRound(biggest[2])>70)
	{
		cv::putText(cv_ptr->image, "too close!", dist_text_base, fontFace, 1, CV_RGB(204,204,0), 1, 8);
		vel_lin_x=PD_dist.getCurrentControl(biggest[2]);
	}
	else
	{
		cv::putText(cv_ptr->image, "Distance - OK", dist_text_base, fontFace, 1, CV_RGB(0,204,0), 1, 8);
		vel_lin_x=0;//PD_dist.getCurrentControl(biggest[2]);
	}

}
geometry_msgs::Twist setVelocity(bool objectFound)
{
	geometry_msgs::Twist set_vel;
	if(objectFound)
	{
		set_vel.linear.x=0;
		set_vel.linear.y=0;
		set_vel.linear.z=0;
		set_vel.angular.x=0;
		set_vel.angular.y=0;
		set_vel.angular.z=0;

		//kontrola obrotu
		set_vel.angular.z=vel_ang_z;
		//kontrola wysokości
		set_vel.linear.z=vel_lin_z;
		//kontrola odległości
		set_vel.linear.x=vel_lin_x;
	}
	else
	{
		set_vel.linear.x=0;
		set_vel.linear.y=0;
		set_vel.linear.z=0;
		set_vel.angular.x=0;
		set_vel.angular.y=0;
		set_vel.angular.z=0;
	}
	return set_vel;
}


//klasa PID
PD::PD(double new_preset)
{
	Kp=0.05;
	Kd=0.05;
	preset=new_preset;
	max=1;
	min=-1;
	derivative=0;
	pre_error=0;
	error=0;
	dt=1;
}
PD::PD( double new_Kp, double new_Kd, double new_dt, double new_max, double new_min, double new_preset)
{
	Kp=new_Kp;
	Kd=new_Kd;
	preset=new_preset;
	max=new_max;
	min=new_min;
	derivative=0;
	pre_error=0;
	error=0;
	dt=new_dt;
}
void PD::setPreset(double new_preset)
{
	preset=new_preset;
}
double PD::getCurrentControl(double current_value)
{
	error=preset-current_value;
	double Pout = Kp * error;


	double derivative = (error - pre_error) / dt;
	double Dout = Kd * derivative;

	double output = Pout + Dout;
	//std::cout<< "pierwotny " << output;
	// Restrict to max/min
	if( output > max )
		output = max;
	else if( output < min )
		output = min;
	//std::cout<< " ograniczony " << output << std::endl;

	// Save error to previous error
	pre_error = error;

	return output;
}
PD::~PD()
{

}
