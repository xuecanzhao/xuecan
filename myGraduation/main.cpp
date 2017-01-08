#include<iostream>
#include"LaneDetection.h"
#include"Otsu.h"
#include <stdio.h>  
#include <time.h>    

using namespace cv;
using namespace std;

int main()
{	
	//test
	VideoCapture capture("D:\\AndroidStdio\\lanedetection\\GraduationProject\\≤‚ ‘ ˝æ›\\road.avi");
	if (!capture.isOpened())
		cout << "can not open" << endl;
	Mat frame,img;
	LaneDetection dec;
	while (1)
	{
		capture >> frame;
		if (frame.empty())
			break;
		clock_t a = clock();
		clock_t b = clock();
		cvtColor(frame, img, CV_RGB2GRAY);
		dec.getFrame(img);
		b = clock();
		cout << (b - a) <<" ";
		dec.EdgeDetect();
		b = clock();
		cout << (b - a) << "  ";
		dec.houghTransform();
		b = clock();
		cout << (b - a) << "  ";
		dec.drawLines(frame);
		b = clock();
		cout << (b - a)  << endl;
	}

}