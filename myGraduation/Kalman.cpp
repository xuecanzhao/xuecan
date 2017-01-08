#include"Kalman.h"

using namespace cv;
using namespace std;
Kalman::Kalman()
{

}
Kalman::Kalman(vector<Vec2f> res)
{
	kalman = new KalmanFilter(2, 2, 0);
	kalman->transitionMatrix = (Mat_<float>(2, 2) << 1,0,0,1);

	prevResult = res;
	kalman->statePre.at<float>(0) = res[0][0]; // r1
	kalman->statePre.at<float>(1) = res[0][1]; // theta1
	//kalman->statePre.at<float>(2) = res[1][0]; // r2
	//kalman->statePre.at<float>(3) = res[1][1]; // theta2

	kalman->statePost.at<float>(0) = res[0][0];
	kalman->statePost.at<float>(1) = res[0][1];
	//kalman->statePost.at<float>(2) = res[1][0];
	//kalman->statePost.at<float>(3) = res[1][1];
	setIdentity(kalman->measurementMatrix, Scalar::all(1));
	setIdentity(kalman->processNoiseCov, Scalar::all(1e-4));
	setIdentity(kalman->measurementNoiseCov, Scalar::all(1e-1));
	setIdentity(kalman->errorCovPost, Scalar::all(1));
}
Kalman::~Kalman()
{
	delete kalman;
}
vector<Vec2f> Kalman::predict()
{
	Mat prediction = kalman->predict(); // predict the state of the next frame
	prevResult[0][0] = prediction.at<float>(0); 
	prevResult[0][1] = prediction.at<float>(1);
	//prevResult[1][0] = prediction.at<float>(2); 
	//prevResult[1][1] = prediction.at<float>(3);
	return prevResult;
}

vector<Vec2f> Kalman::update(vector<Vec2f> measure)
{
	Mat_<float> measurement(2, 1);
	measurement.setTo(Scalar(0));

	measurement.at<float>(0) = measure[0][0]; 
	measurement.at<float>(1) = measure[0][1];
	//measurement.at<float>(2) = measure[1][0];
	//measurement.at<float>(3) = measure[1][1];

	Mat estimated = kalman->correct(measurement); // Correct the state of the next frame after obtaining the measurements

	// Update the measurement	

		measure[0][0] = estimated.at<float>(0); 
		measure[0][1] = estimated.at<float>(1);
		//measure[1][0] = estimated.at<float>(2); 
		//measure[1][1] = estimated.at<float>(3);
	return measure;
}