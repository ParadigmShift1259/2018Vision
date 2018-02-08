#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "../raspicam-0.1.6/src/raspicam.h"
#include "../raspicam-0.1.6/src/raspicam_cv.h"
//#include <networktables/NetworkTable.h>
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

using namespace std;
using namespace cv;
using namespace raspicam;

RaspiCam_Cv Camera;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;
Mat image;
Mat inrange;
Mat drawing;
//VideoCapture cap(0);
Mat frame;
//Mat imageBrightness;

int main()
{
	Camera.set(CV_CAP_PROP_GAIN, 1);
	Camera.set(CV_CAP_PROP_EXPOSURE, 124);
	//Camera.set(
	cout<<"Vision has Started..."<<endl;
	Camera.open();
	cout<<"Camera is opened...\n";
	for (int loop = 0; loop<30 ; loop++)
	{
		Camera.grab();
	}
	cout<<"Grabbing image...\n";
	Camera.retrieve(image);
	cout<<"Retrieveing image...\n";
	Camera.release();
	cout<<"Closing Camera...\n";
	//cap>>frame; //takes a frame
	//cvtColor(frame, image, COLOR_GRAY2RGB;
	//image.convertTo(imageBrightness, -1, 1, 100);

		//Fix later
		const Scalar lower = Scalar(0, 65, 65);
		const Scalar upper = Scalar(5, 115, 115);

		inRange(image, lower, upper, inrange);
		findContours(inrange, contours, hierarchy, CV_RETR_LIST,CV_CHAIN_APPROX_NONE);
		drawing = Mat::zeros(inrange.size(), CV_8UC3);
		int biggestContour = 0;
		int biggestContourLocation = 0;
		int currentContourSize = 0;
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
		circle(drawing, Point(m.m10/m.m00,m.m01/m.m00), 8, Scalar(255,105,180), 1, LINE_8, 0);
	imwrite("inrange.bmp",inrange);
	imwrite("drawing.bmp", drawing);
	imwrite("image.bmp", image);
	imwrite("frame.jpg", frame);
	return 0;

}
