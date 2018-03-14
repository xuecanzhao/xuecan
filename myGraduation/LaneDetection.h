#ifndef LANEDETECTION_H
#define LANEDETECTION_H
#include<opencv2\opencv.hpp>
#include"KalmanTrack.h"
using namespace cv;
using namespace std;
class LaneDetection
{
public: LaneDetection();
		~LaneDetection();
		void getFrame(Mat& src);
		void init(int left, int top, int right, int bottom, int frameWidth, int frameHeight);
		void EdgeDetect();
		void houghTransform();
		int drawLines(Mat &src);
		void track();
		void TCLJudge(Mat &src);
		void judge(Mat &src);
		void judgeTest();
		void judgeThres();
		vector<Point2f> Ransac(vector<Point2f> data);
private:	Mat gray;
			Mat roi;
			Mat edge;
			int left, right, top,bottom,width,height;
			int roiwidth, roiheight;
			vector<Vec2f> lines;
			vector<Vec2f> leftLines;
			vector<Vec2f> rightLines;
			vector<Vec2f> preLeftResult;
			vector<Vec2f> preRightResult;
			bool leftFlag, rightFlag,tclFlag;
			float rthe, lthe,rp,lp,mid;
			int leftErrorCount;
			int rightErrorCount;
			Kalman* leftKalman;
			Kalman* rightKalman;
			int currentStatue;//0为正常，-1为左移，1为右移
			int TCLStatue;
			bool pyrdown;
			int departureCount;
};
#endif  