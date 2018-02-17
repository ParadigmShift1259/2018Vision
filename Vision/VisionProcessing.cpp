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

using namespace std;
using namespace cv;
using namespace raspicam;
using namespace nt;

RaspiCam_Cv Camera;
vector<Vec4i> hierarchy;
vector<vector<Point> > contours;
Mat image;
Mat inrange;
Mat drawing;
Mat PowerCube = imread("PowerCube.jpg", CV_LOAD_IMAGE_COLOR);
NetworkTableInstance nt_Inst;
shared_ptr<NetworkTable> netTable; 	// X,Y,Z is the order for the coordinates
int counter = 0;
int index1, index2;
double im_min_dist1, im_min_dist2, im_actual_dist;



//VideoCapture cap(0);
//Mat frame;

//Mat imageBrightness;

// Following settings is for camera calibrated value
const double default_FOV_row_num = 960;
const double standard_height_p = 400/default_FOV_row_num;
const double pixel_per_in = 42/default_FOV_row_num;
const double pixel_per_degree = 10/default_FOV_row_num;
const double cal_distance_in_inch = 12;

int main()
{

	nt_Inst = NetworkTableInstance::GetDefault();
	Camera.set(CV_CAP_PROP_GAIN, 1);
	Camera.set(CV_CAP_PROP_EXPOSURE, 124);
	//Camera.set(
	cout<<"Vision has Started..."<<endl;
	Camera.open();
	cout<<"Camera is opened...\n";
	/*for (int loop = 0; loop<100 ; loop++)
	{
		Camera.grab();
		counter++;
	}*/
	//Camera.retrieve(image);

	//nt_Inst.StartClient();
	nt_Inst.StartClientTeam(1259);
	//nt_Inst.SetServer("10.12.59.2");
	//nt_Inst.SetServer("roboRIO-1259-frc.local");
	netTable = nt_Inst.GetTable("OpenCV");//netTable = nt_Inst.GetTable("OpenCV");
	netTable->GetEntry("visioncounter").Delete();
	cout<<netTable->GetEntry("visioncounter").Exists()<<endl;
	netTable->GetEntry("visioncounter").ClearPersistent();
	//cap>>frame; //takes a frame
	//cvtColor(frame, image, COLOR_GRAY2RGB;
	//image.convertTo(imageBrightness, -1, 1, 100);

		//Fix later
		const Scalar lower = Scalar(15, 90, 90);
		const Scalar upper = Scalar(70, 250, 250);

		int biggestContour = 0;
		int biggestContourLocation = 0;
		int currentContourSize = 0;

                double cube_height;    // cube height on image 
		double dx, dy, dz;     // distance in X, Y, Z between camera and cube

	//while (true)
	//{

		Camera.grab();
//		if(counter%2==0)
			counter +=3;
		//else
		//	counter -=5;
		Camera.retrieve(image);

		//inRange(image, lower, upper, inrange);
		inRange(PowerCube, lower, upper, inrange);
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
        circle(drawing, Point(m.m10/m.m00,m.m01/m.m00), 8, Scalar(255,105,180), 1, LINE_8, 0);
        double cube_center_x = m.m10/m.m00;
        double cube_center_y = m.m01/m.m00;

        im_min_dist1 = 10000;   // assign a big value
        im_min_dist2 = 10000;   // assign a big value


        int contour_id = biggestContourLocation;
        /*for (int i = 0; i < contours[contour_id].size(); i++)
        {
        	//Point coordinate_of_contour = StartingContour.size();
        	//cout<<"Contour Position at 1x: "<<coordinate_of_contour.x<<" Contour y is: "<<coordinate_of_contour.y<<endl;
        	cout<<"Second Contour is at: " <<contours[contour_id][i] <<endl;
        }*/
           // cout<<"Size of Contours = " <<biggestContourLocation <<endl;

        // Add calculation for cube_hegith, distance (dx,, dy, dz) from camera to the cube
        for (int i = 0; i < contours.size(); i++)
        {
                im_actual_dist = abs(cube_center_x - contours[biggestContourLocation][i].x);
                if (im_min_dist1 > im_actual_dist)
                {
                        if (cube_center_y < contours[biggestContourLocation][i].y)
                        {
                                index1 = i;
                                im_min_dist1 = im_actual_dist;
                        }
                }
                if (im_min_dist2 > im_actual_dist)
                {
                        if (cube_center_y > contours[biggestContourLocation][i].y)
                        {
                                index2 = i;
                                im_min_dist2 = im_actual_dist;
                        }
                }
        }
        cube_height = contours[biggestContourLocation][index1].y - contours[biggestContourLocation][index2].y;



		netTable->GetEntry("visioncounter").ForceSetDouble(counter);
		double roboCounter = netTable->GetEntry("RoboCounter").GetDouble(0);
		netTable->PutNumber("visioncounter", counter);
		//netTable->PutNumber("NetworkTableFirstValue", counter+42);

		//cout<<"Counter is at: "<<counter<<endl;
		//cout<<"RoboRIO is at: "<<roboCounter<<endl;
		imwrite("inrange.bmp",inrange);
		imwrite("drawing.bmp", drawing);
		imwrite("PowerCube.bmp", PowerCube);
		cout<<"x: "<<m.m10<<" y: "<<m.m01<<" divided by: "<<m.m00<<endl;
		cout<<"center of x is at: "<<m.m10/m.m00<<" center of y is at: "<<m.m01/m.m00<<endl;
		cout<<"Index1 is: "<<index1<<" Index2 is: "<<index2<<endl;
		cout<<"Height: "<<cube_height<<endl;
	//}
	Camera.release();
	return 0;





}
