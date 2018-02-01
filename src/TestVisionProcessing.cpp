#include "highgui.h"

int main(int argc, char** argv)
{
	IplImage* img = cvLoadImage(argv[1]);
	cvNamedWindow("Example 1", CV_WINDOW_NORMAL);
	cvShowImage("Example 1", img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow("Example 1");
}
