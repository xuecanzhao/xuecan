#include "Hough.h"
#include <memory.h>
#include <math.h>

using namespace cv;

#define DIR_1 0.32175
#define DIR_2 1.249
#define DIR_3 1.85159
#define DIR_4 2.81984

Hough::Hough()
{
	type = TYPE_OUTER_HOUGH;
}

Hough::Hough(Size size)
{
	type = TYPE_INNER_HOUGH;
	leftHough = Mat::zeros(size, CV_32SC1);
	rightHough = Mat::zeros(size, CV_32SC1);
	leftRhoAcc = new float[size.width];
	rightRhoAcc = new float[size.width];
	accLength = size.width;
}

void Hough::transform(Mat &gray, Mat &edge, int thresh)
{
	if (type == TYPE_INNER_HOUGH) {
		transform(gray, edge, thresh, leftHough, rightHough, leftRhoAcc, rightRhoAcc, accLength);
	}
}

void Hough::transform(Mat &gray, Mat &edge, int thresh, Mat &leftHough, Mat &rightHough,
	float *leftRhoAcc, float *rightRhoAcc, int accLength)
{
	/* reset the acc of rho */
	memset(leftRhoAcc, 0, sizeof(float) * accLength);
	memset(rightRhoAcc, 0, sizeof(float) * accLength);

	/* reset the mat of hough */
	int *houghLeftPtr = (int *)leftHough.ptr<int>(0);
	int *houghRightPtr = (int *)rightHough.ptr<int>(0);
	memset(houghLeftPtr, 0, sizeof(int) * leftHough.rows * leftHough.cols);
	memset(houghRightPtr, 0, sizeof(int) * rightHough.rows * rightHough.cols);

	int colStart = 1, colEnd = edge.cols - 1;
	int rowStart = 1, rowEnd = edge.rows - 1;

	int i, j;
	float px, py, angle;
	float deltaOne = (float)(CV_PI / 180);
	float deltaThree = (float)(CV_PI / 180);

	for (i = rowStart; i < rowEnd; i++) {
		uchar *grayPtrU = (uchar *)gray.ptr<uchar>(i + 1);     // up
		uchar *grayPtrC = (uchar *)gray.ptr<uchar>(i);       // current
		uchar *grayPtrD = (uchar *)gray.ptr<uchar>(i - 1);     // down

		uchar *edgePtrU = (uchar *)edge.ptr<uchar>(i + 1);
		uchar *edgePtrC = (uchar *)edge.ptr<uchar>(i);
		uchar *edgePtrD = (uchar *)edge.ptr<uchar>(i - 1);
		for (j = colStart; j < colEnd; j++) {
			if (edgePtrC[j] <= thresh)
				continue;
			// scharr operator
			px = grayPtrD[j + 1] * 3 + grayPtrC[j + 1] * 10 + grayPtrU[j + 1] * 3
				- grayPtrD[j - 1] * 3 - grayPtrC[j - 1] * 10 - grayPtrU[j - 1] * 3;
			py = grayPtrU[j - 1] * 3 + grayPtrU[j] * 10 + grayPtrU[j + 1] * 3
				- grayPtrD[j - 1] * 3 - grayPtrD[j] * 10 - grayPtrD[j + 1] * 3;

			angle = atan2(py, px);

			if (angle < 0)
				angle += CV_PI;
			if (angle > CV_PI)
				angle -= CV_PI;

			if (angle >= DIR_1 && angle < DIR_2) {
				if (edgePtrC[j] < edgePtrU[j + 1] || edgePtrC[j] < edgePtrD[j - 1])
					continue;
			}
			else if (angle >= DIR_2 && angle < DIR_3) {
				continue;
			}
			else if (angle >= DIR_3 && angle < DIR_4) {
				if (edgePtrC[j] < edgePtrU[j - 1] || edgePtrC[j] < edgePtrD[j + 1])
					continue;
			}
			else {
				if (edgePtrC[j] < edgePtrC[j - 1] || edgePtrC[j] < edgePtrC[j + 1])
					continue;
			}

			// 与x轴的夹角小于90度的则为左车道线
			if (angle < CV_PI / 2) {
				int r = (int)(j * cos(angle) + i * sin(angle) + 0.5);
				int n = (int)(angle / deltaOne + 0.5);
				if (n >= leftHough.rows || r >= leftHough.cols)
					continue;
				houghLeftPtr[n * leftHough.cols + r]++;
				leftRhoAcc[r]++;
			}
			else {
				angle = CV_PI - angle;
				int r = (int)((edge.cols - j) * cos(angle) + i * sin(angle) + 0.5);
				int n = (int)(angle / deltaThree + 0.5);
				if (n >= rightHough.rows || r >= rightHough.cols)
					continue;
				houghRightPtr[n * rightHough.cols + r]++;
				rightRhoAcc[r]++;
			}
		}
	}
}

void Hough::release()
{
	if (type == TYPE_INNER_HOUGH) {
		leftHough.release();
		rightHough.release();
		delete[] leftRhoAcc;
		delete[] rightRhoAcc;
	}
}