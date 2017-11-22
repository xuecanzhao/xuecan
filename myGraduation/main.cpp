#include<iostream>
#include"LaneDetection.h"
#include"Otsu.h"
#include <stdio.h> 
#include <fstream>
#include<string.h>
#include <time.h>    

using namespace cv;
using namespace std;


int main()
{	
	//test git
	VideoCapture capture("D:\\AndroidStdio\\lanedetection\\GraduationProject\\≤‚ ‘ ˝æ›\\road.avi");
	//VideoCapture capture("C:\\Users\\meizalansi\\Desktop\\april21.avi");
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
		//b = clock();
		//cout << (b - a) <<" ";
		dec.EdgeDetect();
		//b = clock();
		//cout << (b - a) << "  ";
		dec.houghTransform();
		//b = clock();
		//cout << (b - a) << "  ";
		dec.judge(frame);
		dec.TCLJudge(frame);
		dec.drawLines(frame);
		b = clock();
	//	cout << (b - a)  << endl;
	}

}
/*int main()
{
	LaneDetection detect; // object of laneDetection class

	string ippath = "./images/"; // Relative path of the images
	string oppath = "./output/"; // Relative path of the output folder
	string imname;
	ifstream imageNames("list.txt"); // Name of the text file which containes name of all the images.
	//getline(imageNames, imname); //getting the name of the image

	//ippath += imname;
	//Mat img1 = imread(ippath, 0); // Read the image
	//resize(img1, img1, Size(detect._width, detect._height)); // Resizing the image (only for display purposes)

	//detect.LMFiltering(img1); // Filtering to detect Lane Markings
	//vector<Vec2f> lines = detect.houghTransform(); // Hough Transform
	//Mat imgFinal = detect.drawLines(img1, lines, imname); // draw final Lane Markings on the original image
	//oppath += imname;
	//imwrite(oppath, imgFinal); // writing the final result to a folder
	LaneDetection dec;
	// Till we consider all the images
	while (getline(imageNames, imname)){
		ippath = "./images/"; // Relative path of the images
		oppath = "./output/"; // Relative path of the output folder
		ippath += imname;
		Mat frame,img2;
		frame = imread(ippath); // Read the image
	//	resize(img2, img2, Size(detect._width, detect._height)); // Resizing the image (only for display purposes)
		cvtColor(frame, img2, CV_RGB2GRAY);
	//	detect.LMFiltering(img2); // Filtering to detect Lane Markings
	//	vector<Vec2f> lines2 = detect.houghTransform(); // Hough Transform
		dec.getFrame(img2);
		dec.EdgeDetect();
		//b = clock();
		//cout << (b - a) << "  ";
		dec.houghTransform();
		//b = clock();
		//cout << (b - a) << "  ";
		dec.judge(img2);
		dec.drawLines(frame);

		// if lanes are not detected, then use the Kalman Filter prediction
		if (lines2.size() < 2) {
			imgFinal = detect.drawLines(img2, lines, imname); // draw final Lane Markings on the original image for display
			oppath += imname;
			imwrite(oppath, imgFinal);
			continue;
		}

		///// Kalman Filter to predict the next state
		CKalmanFilter KF2(lines); // Initialization 
		vector<Vec2f> pp = KF2.predict(); // Prediction

		vector<Vec2f> lines2Final = KF2.update(lines2); // Correction
		lines = lines2Final; // updating the model
		imgFinal = detect.drawLines(img2, lines2, imname); // draw final Lane Markings on the original image
		/////
		
		oppath += imname;
		imwrite(oppath,frame); // writing the final result to a folder

	}

}*/