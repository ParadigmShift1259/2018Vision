//#define USE_NETWORK_TABLES
//#define FISHEYE_CORR_FLAG_TEST 	// define for distortion correction  

#define PI 3.1415926535897932384626433832795

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam.h>
#include <raspicam_cv.h>

#ifdef USE_NETWORK_TABLES
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#endif


#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <cstdint>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>
#include <cmath>

using namespace std;
using namespace cv;
using namespace raspicam;
#ifdef USE_NETWORK_TABLES
using namespace nt;
#endif

RaspiCam_Cv Camera;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;
cv::Mat image;
cv::Mat imageHSV;
cv::Mat inrange;
cv::Mat drawing;

#ifdef USE_NETWORK_TABLES
NetworkTableInstance nt_Inst;
shared_ptr<NetworkTable> netTable; 	// X,Y,Z is the order for the coordinates
#endif

int counter = 0;
int cube_contour_max_index = 0;
int cube_contour_min_index = 0;
double cube_contour_max_y, cube_contour_min_y, im_actual_dist;
double im_center_x, im_center_y;

double Total_Distance_Inch = 0;
double Horizontal_Angle_Degree = 0;
double Vertical_Angle_Degree = 0;
double Horizontal_Distance_Pixel = 0;
double Vertical_Distance_Pixel = 0;
double Horizontal_Distance_Inch = 0;
double Vertical_Distance_Inch = 0;
double Forward_Distance_Inch = 0;

// Following settings is for camera calibrated value
const double DEFAULT_FOV_ROW_NUM = 960;		// default = 960
const double DEFAULT_HEIGHT_PIXEL = 510;	// default = 400
const double DEFAULT_PIXEL_PER_INCH = 41.5;   		// default = 42
const double CAL_DISTANCE_INCH = 12;  		// default = 12

const double ANGLE_THRESHOLD = 60;		// in degree
const double FORWARD_DIST_THRESHOLD = 240;	// in inch = 20 feet
const double CUBE_CONTOUR_THRESHOLD = 100; 	// in pixel

double standard_height_p = 0;
double pixel_per_in = 0;
double pixel_per_degree = 0;


double cube_center_x = 0;
double cube_center_y = 0;

int main()
{
	// Initialize Network Tables
#ifdef USE_NETWORK_TABLES
	nt_Inst = NetworkTableInstance::GetDefault();
	nt_Inst.StartClientTeam(1259);
#endif
	// Configure the camera
	Camera.set(CV_CAP_PROP_GAIN, 10);
	Camera.set(CV_CAP_PROP_EXPOSURE, 200);
	cout<<"Vision has Started..."<<endl;
	Camera.open();
	cout<<"Camera is opened...\n";
	
	// Warm up camera for 60 frames to stabilize image brightness
	for (int loop = 0; loop<60 ; loop++)
	{
		Camera.grab();
	}
	Camera.retrieve(image);

	// Waiting for Network Table serve being the roboRIO
#ifdef USE_NETWORK_TABLES
	while(!nt_Inst.IsConnected())
	{
		sleep(0.1);
	}
	
	// Setting up a table called OenCV
	netTable = nt_Inst.GetTable("OpenCV");
	
	// VisionCounter and making sure it's established
	double visioncounter = 0;
	auto Keys = netTable->GetKeys();
	for (auto key: Keys)
	{
		cout<<"These are the keys: "<<key<<endl;
	}
	auto ntval = netTable->GetValue("visioncounter");

	// Getting the counter value if previously established
	if (ntval)
	{
		visioncounter = ntval->GetDouble();
		cout<<"Retrieving counter value: "<<visioncounter<<endl;

	}
	cout<<"The network table value for vision counter is: "<<visioncounter<<endl;

	// Setting up visioncounter to make sure we are connected
	if(visioncounter > 0)
	{
		counter = visioncounter;
	}
	else
	{
		counter = 0;
	}
#endif
	//Thresholds for HSV: better to identify yellow color
	//const Scalar lower = Scalar(20, 190, 20);
	//const Scalar upper = Scalar(30, 255, 255);

        // HSV threshold to find all yellow
	const Scalar lower = Scalar(20, 200, 50);
	const Scalar upper = Scalar(40, 255, 255);

	// Original threshold settings for BGR 
	//const Scalar lower = Scalar(15, 90, 90);
	//const Scalar upper = Scalar(70, 199, 199);

	//const Scalar lower = Scalar(50, 150, 150);
	//const Scalar upper = Scalar(140, 255, 255);

        //const Scalar lower = Scalar(0, 65, 65);
        //const Scalar upper = Scalar(5, 115, 115);

	// Contour variables to give contour location. Countour is outline points of the object you are trying to identify
	int biggestContour = 0;
	int biggestContourLocation = 0;
	int currentContourSize = 0;

	
	double 	cube_height = 0;    	// cube height on image 
	double dx, dy, dz;		// distance in X, Y, Z between camera and cube

	cout<<"Starting main loop"<<endl;
	while (true)
	{

		// Trigger the camera to get an image
		Camera.grab();

		// Increment by 3 just in case we miss by 3 
		counter +=3;
	
		// Store the image in image variable
		Camera.retrieve(image);
		cout<<"Got Image"<<endl;


		// Making sure we are connected
		counter++;

#ifdef GRAB_DIAG_IMAGE
		imwrite("CameraGrab.bmp", image);
#endif
		
		cvtColor(image, imageHSV, COLOR_BGR2HSV);	// Convert BGR to HSV
#ifdef GRAB_DIAG_IMAGE
		imwrite("CamerGrabHsv.bmp", imageHSV);
#endif
		// Searching for color in the image that has a high of upper scaler and a low of lower scaler. Stores result in inrange
		inRange(imageHSV, lower, upper, inrange);	// Identify color per HSV image
	
		// Finding Contour
		findContours(inrange, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
		// creates a drawing matrix with inrange size and fills it with zeroes
		drawing = Mat::zeros(inrange.size(), CV_8UC3);
	
		// Reset the contour values to zero
		biggestContour = 0;
		biggestContourLocation = 0;
		currentContourSize = 0;

		// Walk through each contour that we found looking for the biggest contour
		for (int i = 0; i < contours.size(); i++)
		{
			currentContourSize = moments(contours[i],true).m00;
			if (biggestContour < currentContourSize)
			{
				biggestContour = currentContourSize;
				biggestContourLocation = i;
			}
		}
         cout<<"After looping through contour vector"<<endl;

		// Actually draw the contour for the biggest contour we find 
        drawContours(drawing, contours,biggestContourLocation , Scalar(124, 252, 0), 2, 8,hierarchy, 0);

		// Finding the moment of the biggest contour
		Moments m = moments(contours[biggestContourLocation], true);

		// Find cube center coordinate and draw a circle at that point
		cube_center_x = m.m10/m.m00;
		cube_center_y = m.m01/m.m00;
		circle(drawing, Point(cube_center_x,cube_center_y), 16, Scalar(255,105,180), 2);

		// Find image center coordinate
		im_center_x = drawing.size().width/2;
		im_center_y = drawing.size().height/2;
		circle(drawing, Point(im_center_x,im_center_y), 1, Scalar(255,105,180));
//		drawMarker(drawing, Point(im_center_x, im_center_y), Scalar(255,255,255));

		// Find cube height per center coordinate of the cube and contour info
		cube_contour_max_y = 10000;   // assign a big value
		cube_contour_min_y = 10000;   // assign a big value

		for (int i = 0; i < contours[biggestContourLocation].size(); i++)
		{
			im_actual_dist = abs(cube_center_x - contours[biggestContourLocation][i].x);
			if (cube_contour_max_y > im_actual_dist)
			{
				if (contours[biggestContourLocation][i].y > cube_center_y) // Find coordinate > object center
				{
					cube_contour_max_index = i;
					cube_contour_max_y = im_actual_dist;
				}
			}

			if (cube_contour_min_y > im_actual_dist)
			{
				if (contours[biggestContourLocation][i].y <= cube_center_y) // Find coordinate <= object center
				{
					cube_contour_min_index = i;
					cube_contour_min_y = im_actual_dist;
				}
			}
		}

		// Apply distortion correction for fisheye camera to three sets of coordinates only to speed up calculation
#ifdef FISHEYE_CORR_FLAG_TEST
		if (FISHEYE_CORR_FLAG == 1)
		{
			double x_original[3];
			double y_original[3];
			double x_corrected[3];
			double y_corrected[3];
			double xt[3];
			double yt[3];
			double ut[3];
			double vt[3];
			double r;
			double R;
			double theta;
			double s;
			double s2;
			double border_corr;
			const double k = -0.46;     		// a constant value for fisheye lens used by Raspberry pi
			
			x_original[0] = cube_center_x;
			x_original[1] = cube_center_x;
			x_original[2] = cube_center_x;
		
			y_original[0] = contours[biggestContourLocation][cube_contour_min_index].y;
			y_original[1] = cube_center_y;
			y_original[2] = contours[biggestContourLocation][cube_contour_max_index].y;

			R = sqrt(im_center_x*im_center_x + im_center_y*im_center_y);
			border_corr = 1/(1 + k*pow( min(im_center_x, im_center_y)/R, 2.0) ); // Scaling factor per border
	
			for (int i = 0; i < 3; i++)
			{
				xt[i] = x_original[i] - im_center_x;
				yt[i] = y_original[i] - im_center_y; 
		
				r = sqrt(xt[i]*xt[i] + yt[i]*yt[i]);	// Find radius

				theta = atan(yt[i]/xt[i])*180/PI; 	// Find theta for the angle

				if 	((yt[i] > 0) && (xt[i] >= 0))
				{	theta = theta;     	}
				else if ((yt[i] < 0) && (xt[i] >= 0))
				{  	theta = 360 + theta;	}
				else if ((yt[i] >= 0) && (xt[i] <= 0))
				{	theta = 180 + theta;	}
				else if ((yt[i] < 0) && (xt[i] <= 0))
				{	theta = 180 + theta;	}
			
				r = r/R;				// Normalize the polar coordinate r
		
				s = r*(1+k*r);				// Apply r-based transform with k

				s2 = s*R;				// Un-normalize s

				s2 = s2*border_corr;			// Scale radius 

				ut[i] = s2*cos(theta*PI/180);		// Converted back to cartesian coordinates
				vt[i] = s2*sin(theta*PI/180);

				x_corrected[i] = ut[i] + im_center_x;	// Add image center back 
				y_corrected[i] = vt[i] + im_center_y;
			}

        	cube_height = abs(y_corrected[2] -  y_corrected[0]);
			cube_center_x = x_corrected[1];
			cube_center_y = y_corrected[1];
		}
		else
#endif
		{
			cube_height = 	abs(contours[biggestContourLocation][cube_contour_max_index].y -  
								contours[biggestContourLocation][cube_contour_min_index].y);
		}

		standard_height_p = DEFAULT_HEIGHT_PIXEL/(DEFAULT_FOV_ROW_NUM/drawing.size().height);
		pixel_per_in = DEFAULT_PIXEL_PER_INCH/(DEFAULT_FOV_ROW_NUM/drawing.size().height);

		Total_Distance_Inch = ((standard_height_p/cube_height)*CAL_DISTANCE_INCH);
		Horizontal_Distance_Pixel = cube_center_x - im_center_x;
		Vertical_Distance_Pixel = im_center_y - cube_center_y;	// Switch due to (0,0) at the top left to convert to bottom left
		Horizontal_Angle_Degree = atan(Horizontal_Distance_Pixel/(pixel_per_in*CAL_DISTANCE_INCH))*180/PI;
		Vertical_Angle_Degree = atan(Vertical_Distance_Pixel/(pixel_per_in*CAL_DISTANCE_INCH))*180/PI;

		// Output values given to Network Table
		Horizontal_Distance_Inch = Total_Distance_Inch * sin(Horizontal_Angle_Degree*PI/180);
		Vertical_Distance_Inch = Total_Distance_Inch * sin(Vertical_Angle_Degree*PI/180);
		Forward_Distance_Inch = Total_Distance_Inch*cos(Vertical_Angle_Degree*PI/180)*cos(Horizontal_Angle_Degree*PI/180);

		// Add constraints for cube distance and angle
		if ((abs(Horizontal_Angle_Degree ) > ANGLE_THRESHOLD ) ||
			(abs(Vertical_Angle_Degree) > ANGLE_THRESHOLD ) ||
			(Forward_Distance_Inch < 0) || (Forward_Distance_Inch > FORWARD_DIST_THRESHOLD ) ||
			(contours[biggestContourLocation].size() < CUBE_CONTOUR_THRESHOLD ))
	   	{
			Horizontal_Distance_Inch = 0;
			Vertical_Distance_Inch=0;
			Forward_Distance_Inch =0;
	   	}

#ifdef USE_NETWORK_TABLES
		netTable->PutNumber("visioncounter", counter);
		netTable->PutNumber("XOffAngle", Horizontal_Angle_Degree);
		//netTable->GetEntry("visioncounter").ForceSetDouble(counter);
#endif

		// For manually calibrating the camera
		// TODO Wrap this in a runtime flag
		imwrite("inrange.bmp",inrange);
		imwrite("drawing.bmp", drawing);
		imwrite("image.bmp", image);

		//cout<<"x: "<<m.m10<<" y: "<<m.m01<<" divided by: "<<m.m00<<endl;
		//cout<<"center of cube x is at: "<<m.m10/m.m00<<" center of cube y is at: "<<m.m01/m.m00<<endl;
		//cout<<"cube_contour_max_index is: "<<cube_contour_max_index<<" cube_contour_min_index is: "<<cube_contour_min_index<<endl;
		//cout<<"Height: "<<cube_height<<endl;
		//cout<<"Horizontal_Angle_Degree: "<<Horizontal_Angle_Degree<<" Vertical_Angle_Degree "<<Vertical_Angle_Degree<<" Total_Distance_Inch : "<<Total_Distance_Inch<<endl;

		cout 	<< "Counter: " << counter 
				<< " Horizontal_Distance_Inch: " << Horizontal_Distance_Inch
				<< " Vertical_Distance_Inch " << Vertical_Distance_Inch
				<< " Forward_Distance_Inch: " << Forward_Distance_Inch 
				<< " Horizontal_Angle " << Horizontal_Angle_Degree 
				<< endl;

	}  //end of while  

	Camera.release();
	return 0;

}
