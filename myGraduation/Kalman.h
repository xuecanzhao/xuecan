#ifndef KALMAN_H
#define KALMAN_H

#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class Kalman
{
public: Kalman(vector<Vec2f>);
		Kalman();
	~Kalman();
	vector<Vec2f> predict();
	vector<Vec2f> update(vector<Vec2f>);

	KalmanFilter* kalman;
	vector<Vec2f> prevResult;
};
#endif