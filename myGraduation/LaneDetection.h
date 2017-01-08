#ifndef LANEDETECTION_H
#define LANEDETECTION_H
#include<opencv2\opencv.hpp>
#include"Kalman.h"
using namespace cv;
using namespace std;
class LaneDetection
{
public: LaneDetection();
		~LaneDetection();
		void getFrame(Mat& src);
		void EdgeDetect();
		void houghTransform();
		void drawLines(Mat &src);
		void track();
		vector<Point2f> Ransac(vector<Point2f> data);
private:	Mat gray;
			Mat roi;
			Mat edge;
			int left, right, top,bottom,width,height;
			vector<Vec2f> lines;
			vector<Vec2f> leftLines;
			vector<Vec2f> rightLines;
			vector<Vec2f> preLeftResult;
			vector<Vec2f> preRightResult;
			bool leftFlag;
			bool rightFlag;
			int leftErrorCount;
			int rightErrorCount;
			Kalman* leftKalman;
			Kalman* rightKalman;
};
#endif  