#include"LaneDetection.h"
#include"EdgeDetect.h"
#include <time.h>
#define DIR_1 0.32175 // 17.836 бу 
#define DIR_2 1.249   // 51.318 бу      
#define DIR_3 1.85159  // 61.628 бу    
#define DIR_4 2.81984   // 70.474 бу
#define ERROR_FRAME 5

LaneDetection::LaneDetection()
{
	leftFlag = false;
	rightFlag = false;
	leftErrorCount = 0;
	rightErrorCount = 0;
	leftKalman = NULL;
	rightKalman = NULL;

}
void LaneDetection::getFrame(Mat& src)
{
	gray = src.clone();
	width = src.cols;
	height = src.rows;
	left = 0;
	right = width;
	top = height / 2;
	bottom = height - 2;
	Rect r1(left, top, right - left, bottom - top);
	gray(r1).copyTo(roi);
}

LaneDetection::~LaneDetection()
{

}
void LaneDetection::EdgeDetect()
{
	edgeDetect(roi,edge);
}


void LaneDetection::houghTransform()
{
	HoughLines(edge, lines, 1, CV_PI / 180, 80,0,0);
	int count = lines.size();
	if (lines.size() >0)
	{
		vector<Point2f> left;
		vector<Point2f> right;
		for (int i = 0; i < count; i++)
		{
		//	cout << lines[i][1] / (CV_PI / 180) << endl;
			//if ((lines[i][1] > CV_PI / 9) && (lines[i][1]<CV_PI*7/18))
			if ((lines[i][1] >0) && (lines[i][1]<CV_PI * 7 / 18))
				left.push_back(lines[i]);
			//else if ((lines[i][1] > CV_PI * 11 / 18) && (lines[i][1]<CV_PI*8/9))
			else if ((lines[i][1] > CV_PI * 11 / 18) && (lines[i][1]<CV_PI))
				right.push_back(lines[i]);
		//	else continue;
		}
		//cout << left.size()<<" "<<right.size() << endl;
		lines.clear();
		leftLines.clear();
		rightLines.clear();
		left = Ransac(left);
		right = Ransac(right);
		//if (left.size() < 1 || right.size() < 1 ||(float)(cos((left[0].y + left[1].y) / 2) * cos((right[0].y + right[1].y) / 2)) >= 0) 
		//	return ;
		if (left.size() >= 2)
		{
			float lthe = (left[0].y + left[1].y) / 2;
		//	cout << "lthe:" << lthe / (CV_PI / 180) << endl;
			if (((lthe > CV_PI / 9) && (lthe<CV_PI * 7/ 18)))
			leftLines.push_back(Vec2f((left[0].x + left[1].x) / 2, lthe));
			if (leftLines.size() >= 1)
			{
				if (leftKalman == NULL)
				{
					preLeftResult = leftLines;
					leftKalman = new Kalman(preLeftResult);
				}
				else
				{
					(*leftKalman).predict();
					preLeftResult = (*leftKalman).update(leftLines);
				}
				if (leftErrorCount > 0)
					leftErrorCount = 0;
			}
			else
			{

				leftErrorCount++;
				if (leftErrorCount >= ERROR_FRAME)
				{
					preLeftResult.clear();
					leftErrorCount = ERROR_FRAME;
					if (leftKalman != NULL)
						delete leftKalman;
					leftKalman = NULL;
				}
			}

		}
		else
		{
			leftErrorCount++;
		//	cout << "lefterror"<<leftErrorCount << endl;
			if (leftErrorCount >= ERROR_FRAME)
			{
				preLeftResult.clear();
				leftErrorCount = ERROR_FRAME;
				if (leftKalman!=NULL)
				delete leftKalman;
				leftKalman = NULL;
			}
		}

		if (right.size() >= 2)
		{
			float rthe = (right[0].y + right[1].y) / 2;
			//cout << "right:"<<rthe / (CV_PI / 180) <<endl;
			if ((rthe > CV_PI * 11/ 18) && (rthe<CV_PI * 16 / 18))
			rightLines.push_back(Vec2f((right[0].x + right[1].x) / 2, rthe));
			if (rightLines.size() >= 1)
			{
				if (rightKalman == NULL)
				{
					preRightResult = rightLines;
					rightKalman = new Kalman(preRightResult);

				}
				else{
					rightKalman->predict();
					preRightResult = rightKalman->update(rightLines);
				}
				if (rightErrorCount > 0)
					rightErrorCount = 0;
			}
			else
			{
				rightErrorCount++;
				if (rightErrorCount >= ERROR_FRAME)
				{
					preRightResult.clear();
					rightErrorCount = ERROR_FRAME;
					if (rightKalman != NULL)
						delete rightKalman;
					rightKalman = NULL;
				}
			}
		}
		else
		{
			rightErrorCount++;
		//	cout << "righterror:" << rightErrorCount << endl;
			if (rightErrorCount >= ERROR_FRAME)
			{
				preRightResult.clear();
				rightErrorCount = ERROR_FRAME;
				if (rightKalman!=NULL)
				delete rightKalman;
				rightKalman = NULL;
			}			
		}
	}

}

void LaneDetection::track()
{
	
		
}

void LaneDetection::drawLines(Mat &src)
{
	//if (leftLines.size() <= 0)
	//	leftLines = preLeftResult;
	int leftcount = preLeftResult.size();
	for (int i = 0; i < leftcount; i++)
	{
		float rho = preLeftResult[i][0], theta = preLeftResult[i][1];
		//float mm = theta / (CV_PI / 180);
		//cout << theta/(CV_PI/180)<<" "<<rho << endl;
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			pt1.x = cvRound((rho - (bottom- bottom)*b)/ a) + left;
			pt1.y = top;
			pt2.x = cvRound((rho - (bottom - top)*b) / a) + left;
			pt2.y = bottom;
			if (pt1.x>width *2/ 3)
				break;
			line(src, pt1, pt2, Scalar(0, 0, 255), 1, CV_AA);
	}
	//if (rightLines.size() <= 0)
	//	rightLines = preRightResult;
	int rightcount = preRightResult.size();
	for (int i = 0; i < rightcount; i++)
	{
		float rho = preRightResult[i][0], theta = preRightResult[i][1];
		//float mm = theta / (CV_PI / 180);
		//cout << theta / (CV_PI / 180) << " " << rho << endl;
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound((rho - (bottom-bottom)*b) / a) + left;
		pt1.y = top;
		pt2.x = cvRound((rho - (bottom - top)*b) / a) + left;
		pt2.y = bottom;
		if (pt1.x < width / 3)
			break;
		line(src, pt1, pt2, Scalar(0, 0, 255), 1, CV_AA);
	}
	namedWindow("src");
	imshow("src", src);
	namedWindow("edge");
	imshow("edge", edge);
	clock_t a = clock();
	if (waitKey(30) == 27)
	{
		if (waitKey(0) == 27)
			exit(0);
	}
	clock_t b = clock();
	cout << (b - a) << endl;
}

vector<Point2f> LaneDetection::Ransac(vector<Point2f> data){

	vector<Point2f> res;
	int maxInliers = 0;
	int len = data.size();
	// Picking up the first sample
	for (int i = 0; i < len; i++){
		Point2f p1 = data[i];

		// Picking up the second sample
		for (int j = i + 1; j < len; j++){
			Point2f p2 = data[j];
			int n = 0;

			// Finding the total number of inliers
			for (int k = 0; k < len; k++){
				Point2f p3 = data[k];
				float normalLength = norm(p2 - p1);
				float distance = abs((float)((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / normalLength);
				if (distance < 0.01) n++;
			}

			// if the current selection has more inliers, update the result and maxInliers
			if (n > maxInliers) {
				res.clear();
				maxInliers = n;
				res.push_back(p1);
				res.push_back(p2);
			}

		}

	}
	return res;
}