#ifndef _HOUGH_H_
#define _HOUGH_H_

#include <opencv2/opencv.hpp>

using namespace cv;

#define TYPE_INNER_HOUGH -1
#define TYPE_OUTER_HOUGH 1

class Hough
{
public:
	Mat leftHough;
	Mat rightHough;
	float *leftRhoAcc;
	float *rightRhoAcc;
	int accLength;
private:
	int type;
public:
	Hough();
	Hough(Size size);
	void transform(Mat &gray, Mat &edge, int thresh);
	void transform(Mat &gray, Mat &edge, int thresh, Mat &leftHough, Mat &rightHough,
		float *leftRhoAcc, float *rightRhoAcc, int accLength);
	void release();
};

#endif //_HOUGH_H_
