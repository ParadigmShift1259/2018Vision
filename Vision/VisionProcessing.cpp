#define PI 3.1415926535897932384626433832795

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <raspicam.h>
#include <raspicam_cv.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
//#include <thread>
//#include <errno.h>
//#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <stdint.h>
//#include <atomic>
#include <math.h>
#include <cmath>

using namespace std;
using namespace cv;
using namespace raspicam;
using namespace nt;

RaspiCam_Cv Camera;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;
Mat image;
Mat imageHSV;
Mat inrange;
Mat drawing;

NetworkTableInstance nt_Inst;
shared_ptr<NetworkTable> netTable; 	// X,Y,Z is the order for the coordinates
int counter = 0;
int index1 = 0;
int index2 = 0;
double im_min_dist1, im_min_dist2, im_actual_dist;
double im_center_x, im_center_y;

int    FISHEYE_CORR_FLAG = 1;	// 0 for no distorsion correction, 1 for correction 
double x_original[3];
double y_original[3];
double x_corrected[3];
double y_corrected[3];
double xt[3], yt[3], ut[3], vt[3], r, R, theta, s, s2, border_corr;
double k = -0.46;     		// a constant value for fisheye lens used by Raspberry pi

double Total_Distance_Inch = 0;
double Horizontal_Angle_Degree = 0;
double Vertical_Angle_Degree = 0;
double Horizontal_Distance_Pixel = 0;
double Vertical_Distance_Pixel = 0;
double Horizontal_Distance_Inch = 0;
double Vertical_Distance_Inch = 0;
double Forward_Distance_Inch = 0;


//VideoCapture cap(0);
//Mat frame;

//Mat imageBrightness;

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

	nt_Inst = NetworkTableInstance::GetDefault();
	Camera.set(CV_CAP_PROP_GAIN, 10);
	Camera.set(CV_CAP_PROP_EXPOSURE, 200);
	//Camera.set(
	cout<<"Vision has Started..."<<endl;
	Camera.open();
	cout<<"Camera is opened...\n";
	//Camera.grab();
	
	// Warm up camera for 60 frames to stabilize image brightness
	for (int loop = 0; loop<60 ; loop++)
	{
		Camera.grab();
		//counter++;
	}
	Camera.retrieve(image);

	//nt_Inst.StartClient();
    	//nt_Inst.SetServer("roboRIO-1259-frc.local");

	nt_Inst.StartClientTeam(1259);
	//nt_Inst.SetServer("10.12.59.2");
	//nt_Inst.SetServer("roboRIO-1259-frc.local");
	netTable = nt_Inst.GetTable("OpenCV");
	//netTable->GetEntry("visioncounter").Delete();
	//cout<<netTable->GetEntry("visioncounter").Exists()<<endl;
	//netTable->GetEntry("visioncounter").ClearPersistent();
	//cap>>frame; //takes a frame
	//cvtColor(frame, image, COLOR_GRAY2RGB;
	//image.convertTo(imageBrightness, -1, 1, 100);
	
	
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

	int biggestContour = 0;
	int biggestContourLocation = 0;
	int currentContourSize = 0;

        double 	cube_height = 0;    	// cube height on image 
	double dx, dy, dz;		// distance in X, Y, Z between camera and cube
	//	  namedWindow("inrange.bmp",WINDOW_AUTOSIZE);
	//	  namedWindow("drawing.bmp", WINDOW_AUTOSIZE);
	//	  namedWindow("image.bmp", WINDOW_AUTOSIZE);



	while (true)
	{

		Camera.grab();

		// Mat image = imread("./test_images/IMG_4415.JPG", CV_LOAD_IMAGE_COLOR);  // For debug

		//if(counter%2==0)
			counter +=3;
		//else
		//	counter -=5;
		Camera.retrieve(image);


		counter++;
		
		cvtColor(image, imageHSV, COLOR_BGR2HSV);	// Convert BGR to HSV
		inRange(imageHSV, lower, upper, inrange);	// Identify color per HSV image
	
		findContours(inrange, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);

		drawing = Mat::zeros(inrange.size(), CV_8UC3);
	
		biggestContour = 0;
		biggestContourLocation = 0;
		currentContourSize = 0;
		for (int i = 0; i < contours.size(); i++)
		{
			currentContourSize = moments(contours[i],true).m00;
			if (biggestContour < currentContourSize)
			{
				biggestContour = currentContourSize;
				biggestContourLocation = i;
			}
		}
                             

        	drawContours(drawing, contours,biggestContourLocation , Scalar(124, 252, 0), 2, 8,hierarchy, 0);

        	Moments m = moments(contours[biggestContourLocation], true);

	        // find cube center coordinate
        	cube_center_x = m.m10/m.m00;
        	cube_center_y = m.m01/m.m00;
        	circle(drawing, Point(cube_center_x,cube_center_y), 16, Scalar(255,105,180), 2, LINE_8, 0);

	        // Find image center coordinate
        	im_center_x = drawing.size().width/2;
        	im_center_y = drawing.size().height/2;
		drawMarker(drawing, Point(im_center_x, im_center_y), Scalar(255,255,255), MARKER_STAR, 30, 2, LINE_8);

	        // Find cube hegith per center coordinate of the cube and contour info
        	im_min_dist1 = 10000;   // assign a big value
       	 	im_min_dist2 = 10000;   // assign a big value

        	for (int i = 0; i < contours[biggestContourLocation].size(); i++)
        	{
                	im_actual_dist = abs(cube_center_x - contours[biggestContourLocation][i].x);
                	if (im_min_dist1 > im_actual_dist) 
                	{
                        	if (contours[biggestContourLocation][i].y > cube_center_y)    // Find coordinate > object center
                        	{
                                	index1 = i;
                                	im_min_dist1 = im_actual_dist;
                        	}
                	}


	                if (im_min_dist2 > im_actual_dist)
        	        {
                	        if (contours[biggestContourLocation][i].y <= cube_center_y)    // Find coordinate <= object center
                        	{
                                	index2 = i;
                                	im_min_dist2 = im_actual_dist;
                        	}
                	}
        	}

	        if (FISHEYE_CORR_FLAG == 1)
		{
        		// Apply distortion correction for fisheye camera to three sets of coordinates only to speed up calculation
			x_original[0] = cube_center_x;
			x_original[1] = cube_center_x;
			x_original[2] = cube_center_x;
		
			y_original[0] = contours[biggestContourLocation][index2].y;
			y_original[1] = cube_center_y;
			y_original[2] = contours[biggestContourLocation][index1].y;

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
		{
			cube_height = abs(contours[biggestContourLocation][index1].y - 
					  contours[biggestContourLocation][index2].y);
		}

		standard_height_p = DEFAULT_HEIGHT_PIXEL/(DEFAULT_FOV_ROW_NUM/drawing.size().height);
		pixel_per_in = DEFAULT_PIXEL_PER_INCH/(DEFAULT_FOV_ROW_NUM/drawing.size().height);

    	    	Total_Distance_Inch = ((standard_height_p/cube_height)*CAL_DISTANCE_INCH);
    	    	Horizontal_Distance_Pixel = cube_center_x - im_center_x;
    	    	Vertical_Distance_Pixel = im_center_y - cube_center_y;	// Switch due to (0,0) at the top left to convert to bottom left
    	   	Horizontal_Angle_Degree = atan(Horizontal_Distance_Pixel/(pixel_per_in*CAL_DISTANCE_INCH))*180/PI;
    		Vertical_Angle_Degree = atan(Vertical_Distance_Pixel/(pixel_per_in*CAL_DISTANCE_INCH))*180/PI;


        	Horizontal_Distance_Inch = Total_Distance_Inch * sin(Horizontal_Angle_Degree*PI/180);
        	Vertical_Distance_Inch = Total_Distance_Inch * sin(Vertical_Angle_Degree*PI/180);
        	Forward_Distance_Inch = Total_Distance_Inch*cos(Vertical_Angle_Degree*PI/180)*cos(Horizontal_Angle_Degree*PI/180);


      		// Add constraints for cube distance
        	if ((abs(Horizontal_Angle_Degree ) > ANGLE_THRESHOLD ) ||
 	    	   (abs(Vertical_Angle_Degree) > ANGLE_THRESHOLD ) ||
	    	   (Forward_Distance_Inch < 0) || (Forward_Distance_Inch > FORWARD_DIST_THRESHOLD ) ||
	    	   (contours[biggestContourLocation].size() < CUBE_CONTOUR_THRESHOLD ))
	   	{
       			Horizontal_Distance_Inch = 0;
     			Vertical_Distance_Inch=0;
       			Forward_Distance_Inch =0;
	   	}
		netTable->GetEntry("visioncounter").ForceSetDouble(counter);
//		double roboCounter = netTable->GetEntry("RoboCounter").GetDouble(0);
		netTable->PutNumber("visioncounter", counter);
//		netTable->PutNumber("Horizontal_Distance_Inch", Horizontal_Distance_Inch);
//		netTable->PutNumber("Vertical_Distance_Inch", Vertical_Distance_Inch);
//		netTable->PutNumber("Forward_Distance_Inch", Forward_Distance_Inch);
		netTable->PutNumber("XOffAngle", Horizontal_Angle_Degree);
		//netTable->PutNumber("NetworkTableFirstValue", counter+42);

		//cout<<"Counter is at: "<<counter<<endl;
		//cout<<"RoboRIO is at: "<<roboCounter<<endl;
		  imwrite("inrange.bmp",inrange);
//		  imshow("inrange.bmp",inrange);
		  imwrite("drawing.bmp", drawing);
//		  imshow("drawing.bmp", drawing);
		  imwrite("image.bmp", image);
//		  imshow("image.bmp", image);

		//cout<<"x: "<<m.m10<<" y: "<<m.m01<<" divided by: "<<m.m00<<endl;
		//cout<<"center of cube x is at: "<<m.m10/m.m00<<" center of cube y is at: "<<m.m01/m.m00<<endl;
		//cout<<"Index1 is: "<<index1<<" Index2 is: "<<index2<<endl;
		//cout<<"Height: "<<cube_height<<endl;
		//cout<<"Horizontal_Angle_Degree: "<<Horizontal_Angle_Degree<<" Vertical_Angle_Degree "<<Vertical_Angle_Degree<<" Total_Distance_Inch : "<<Total_Distance_Inch<<endl;

		cout<<"Counter: "<<counter<<" Horizontal_Distance_Inch: "<<Horizontal_Distance_Inch<<
 		     " Vertical_Distance_Inch "<<Vertical_Distance_Inch<<" Forward_Distance_Inch: "<<
		      Forward_Distance_Inch<<endl<<"Horizontal_Angle "<<Horizontal_Angle_Degree<<endl;

	}  //end of while
	Camera.release();
	return 0;

}
