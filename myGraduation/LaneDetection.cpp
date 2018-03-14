#include"LaneDetection.h"
#include"EdgeDetect.h"
#include"cross.h"
#include <time.h>
#include<fstream>
#include<Windows.h>
#define DIR_1 0.32175 // 17.836 ° 
#define DIR_2 1.249   // 51.318 °      
#define DIR_3 1.85159  // 61.628 °    
#define DIR_4 2.81984   // 70.474 °
#define ERROR_FRAME 5
#define DEPARTURE_COUTN 1
#define DEPARTURE_THRES 2

using namespace std;

LaneDetection::LaneDetection()
{
	leftFlag = false;
	rightFlag = false;
	tclFlag = false;
	leftErrorCount = 0;
	rightErrorCount = 0;
	leftKalman = NULL;
	rightKalman = NULL;
	currentStatue = 0;
	departureCount = 0;
	pyrdown = false;
}
void LaneDetection::getFrame(Mat& src)
{
	//gray = src;
	Rect r1(left, top, roiwidth, roiheight);
	gray = src(r1);
	if (roiwidth*roiheight > 480*320)
	{
		Size dsize = Size(roiwidth>>1, roiheight>>1);
		roi = Mat(dsize, CV_8U);
		resize(gray, roi, dsize);
		pyrdown = true;
	}
	else{
		roi = gray.clone();
		pyrdown = false;
	}
}

void LaneDetection::init(int left, int top, int right, int bottom, int frameWidth, int frameHeight)
{
	this->left = left;
	this->right = right;
	this->top = top;
	this->bottom = bottom;
	this->width = frameWidth;
	this->height = frameHeight;
	roiwidth = right - left;
	roiheight = bottom - top;
}

LaneDetection::~LaneDetection()
{

}
/*
进行边缘检测
*/
void LaneDetection::EdgeDetect()
{
	edgeDetect(roi,edge);
}

/*
进行霍夫变化
*/

void LaneDetection::houghTransform()
{
	double t1 = getTickCount();
	//clock_t a = clock();
	HoughLines(edge, lines, 1, CV_PI / 180, 80,0,0);	
	double t2 = getTickCount();
	//cout << "hough"<<(t2 - t1)/getTickFrequency() << endl;
	 
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
		double t3 = getTickCount();
		//cout << "time" << (t3 - t2) / getTickFrequency() << endl;
		//if (left.size() < 1 || right.size() < 1 ||(float)(cos((left[0].y + left[1].y) / 2) * cos((right[0].y + right[1].y) / 2)) >= 0) 
		//	return ;
		if (left.size() >= 2)
		{
			 lthe = (left[0].y + left[1].y) / 2;
			 lp = (left[0].x + left[1].x) / 2;
			 ofstream out;
			 out.open("shiji2.txt", ios::out | ios::app);
			 out << lthe / (CV_PI / 180);
			 out << "\n";
			 out.close();
		  cout << "lthe:" << lthe / (CV_PI / 180) << endl;
		//	if (((lthe > CV_PI / 9) && (lthe<CV_PI * 7/ 18)))
			leftLines.push_back(Vec2f(lp, lthe));

			if (leftLines.size() >= 1)
			{
				if (leftKalman == NULL)
				{
					preLeftResult = leftLines;
					leftKalman = new Kalman(preLeftResult);
				}
				else
				{	
					/*ofstream out;
					out.open("shiji.txt", ios::out | ios::app);
					out << lthe / (CV_PI / 180);
					out << "\n";
					out.close();*/
					//clock_t a = clock();
					(*leftKalman).predict();
					preLeftResult = (*leftKalman).update(leftLines);
				/*	float theta = preLeftResult[0][1];
					ofstream out1;
					out1.open("jiaozheng.txt", ios::out | ios::app);
					out1 << theta / (CV_PI / 180);
					out1 << "\n";
					out1.close();
					cout <<"leftpre"<< theta / (CV_PI / 180) << endl;
					clock_t b = clock();*/

				}
				if (leftErrorCount > 0)
					leftErrorCount = 0;
			}
			else
			{
				lthe = -1;
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
			lthe = -1;
			leftErrorCount++;
		//  cout << "lefterror"<<leftErrorCount << endl;
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
			 rthe = (right[0].y + right[1].y) / 2;
			 rp = (right[0].x + right[1].x) / 2;
		//	cout << "right:"<<rthe / (CV_PI / 180) <<endl;
		//	if ((rthe > CV_PI * 11/ 18) && (rthe<CV_PI * 16 / 18))
			rightLines.push_back(Vec2f(rp, rthe));
			if (rightLines.size() >= 1)
			{
				if (rightKalman == NULL)
				{
					preRightResult = rightLines;
					rightKalman = new Kalman(preRightResult);

				}
				else{
					/*ofstream out;
					out.open("shiji1.txt", ios::out | ios::app);
					out << rthe / (CV_PI / 180);
					out << "\n";
					out.close();*/
					rightKalman->predict();
					preRightResult = rightKalman->update(rightLines);
				/*	float theta = preRightResult[0][1];
					ofstream out1;
					out1.open("jiaozheng1.txt", ios::out | ios::app);
					out1 << theta / (CV_PI / 180);
					out1 << "\n";
					out1.close();*/
				}
				if (rightErrorCount > 0)
					rightErrorCount = 0;
			}
			else
			{
				rthe = -1;
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
			rthe = -1;
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
	//double t2 = getTickCount();
	//cout << (t2 - t1) << endl;
	//b = clock();
	//cout << (b - a) << endl;
}

void LaneDetection::track()
{
	
		
}

int LaneDetection::drawLines(Mat &src)
{
	//if (leftLines.size() <= 0)
	//	leftLines = preLeftResult;
	LINE line1, line2;
	int leftcount = preLeftResult.size();
	//int le = 0, ri = 0;
	for (int i = 0; i < leftcount; i++)
	{
		float rho = preLeftResult[i][0], theta = preLeftResult[i][1];
		//float mm = theta / (CV_PI / 180);
		//cout << "lkaer"<<theta/(CV_PI/180)<< endl;
			Point pt1, pt2;
			double a = cos(theta), b = sin(theta);
			double x0 = a*rho, y0 = b*rho;
			//line1.pStart.x = cvRound(x0 + 1000 * (-b));
			//line1.pStart.y = cvRound(y0 + 1000 * (a));
			//line1.pEnd.x = cvRound(x0 - 1000 * (-b));
			//line1.pEnd.y = cvRound(y0 - 1000 * (a));
			if (pyrdown)
			pt1.x = 2*cvRound(rho/a) + left;
			else line1.pStart.x = pt1.x = cvRound(rho / a) + left;
			line1.pStart.y=pt1.y = top;
			if (pyrdown)
			{
				pt2.x = 2 * cvRound((rho - (roiheight>>1)*b) / a) + left;
				pt2.y = top +roiheight;
			}
			else
			{
				line1.pEnd.x=pt2.x = cvRound((rho - roiheight*b) / a) + left;
				line1.pEnd.y=pt2.y = top + roiheight;
			}
		//	if (pt1.x>width *2/ 3)
		//		break;
			if (currentStatue == 0)
				line(src, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
			else line(src, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	//		cout <<"ptl:"<< pt2.x << endl;
	//		le = pt2.x;
	}
	//if (rightLines.size() <= 0)
	//	rightLines = preRightResult;
	int rightcount = preRightResult.size();
	for (int i = 0; i < rightcount; i++)
	{
		float rho = preRightResult[i][0], theta = preRightResult[i][1];
		//float mm = theta / (CV_PI / 180);
		//cout << "rkaer"<<theta/(CV_PI/180)<< endl;
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		//line2.pStart.x = cvRound(x0 + 1000 * (-b));
		//line2.pStart.y = cvRound(y0 + 1000 * (a));
		//line2.pEnd.x = cvRound(x0 - 1000 * (-b));
		//line2.pEnd.y = cvRound(y0 - 1000 * (a));
		if (pyrdown)
			pt1.x = 2 * cvRound(rho / a) + left;
		else line2.pStart.x=pt1.x = cvRound(rho / a) + left;
		line2.pStart.y=pt1.y = top;
		if (pyrdown)
		{
			pt2.x = 2 * cvRound((rho - (roiheight>>1)*b) / a) + left;
			pt2.y = top + roiheight;
		}
		else
		{
			line2.pEnd.x=pt2.x = cvRound((rho - roiheight*b) / a) + left;
			line2.pEnd.y=pt2.y = top + roiheight;
		}
		//if (pt1.x < width / 3)
		//	break;
		if (currentStatue==0)
		line(src, pt1, pt2, Scalar(0, 255, 0), 3, CV_AA);
		else line(src, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
	//	cout << "ptr:" << pt2.x << endl;
	//	ri = pt2.x;
	}
	Point cross = CrossPoint(&line1, &line2);
	cout << "CrossPoint: " << "(" << cross.x << "," << cross.y << ")" << endl;
	circle(src, cross, 10, Scalar(0, 0, 255), 1);
/*	if (ri&&le)
	{   
		if (!tclFlag)
		{
			mid = (ri + le) / 2;
			tclFlag = true;
		}
		cout << "mid:" << mid<< endl;
		int roadWidth = ri - le;
		int catLength = 1.8/3.5*roadWidth;
		int rd = ri - mid - catLength / 2;
		cout << "rd:" <<rd<< endl;
		int ld = mid - le - catLength / 2;
		cout << "ld:"<<ld << endl;
		cout <<(float) rd / ld << endl;
		cout << endl;
	}*/
	
	switch(currentStatue)
	{
	case 0:putText(src, "normal", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8);
		break;
	case -1:putText(src, "left", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8);
		break;
	case 1:putText(src, "right", Point(50, 60), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 1, 8);
		break;
	}
	namedWindow("src");
	imshow("src", src);
	namedWindow("roi");
	imshow("roi", roi);
	namedWindow("edge");
	imshow("edge", edge);
	//clock_t a = clock();
	if (waitKey(30) == 27)
	{
		if (waitKey(0) == 27)
			exit(0);
	}
	//clock_t b = clock();
	//cout << (b - a) << endl;
	return leftcount + rightcount;
}

vector<Point2f> LaneDetection::Ransac(vector<Point2f> data){

	vector<Point2f> res;
	int maxInliers = 0;
	int len = data.size();
	if (len <= 2)
		res = data;
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
void LaneDetection::TCLJudge(Mat &src)
{
	int le=0,ri=0;
	if (lthe > 0 && rthe > 0)
	{
		Point pt1, pt2;
		double a = cos(lthe), b = sin(lthe);
		//double x0 = a*lp, y0 = b*lp;
		cout << "lp:" << lp << endl;
		pt1.x = cvRound((lp - (bottom - bottom)*b) / a) + left;
		pt1.y = top;
		pt2.x = cvRound((lp - (bottom - top)*b) / a) + left;
		pt2.y = bottom;
		cout << "ptl:" << pt2.x << endl;
		le = pt2.x;
		if (le < 0)
			waitKey(0);
		Point pt3, pt4;
		double a2 = cos(rthe), b2 = sin(rthe);
		//double x0 = a2*rp, y0 = b2*rp;
		pt3.x = cvRound((rp - (bottom - bottom)*b2) / a2) + left;
		pt3.y = top;
		pt4.x = cvRound((rp - (bottom - top)*b2) / a2) + left;
		pt4.y = bottom;	
		cout << "ptr:" << pt4.x << endl;
		ri = pt4.x;
	}
	if (ri&&le)
	{
		if (!tclFlag)
		{
			mid = (ri + le) / 2;
			tclFlag = true;
		}
		cout << "mid:" << mid << endl;
		int roadWidth = ri - le;
		int catLength = 1.8 / 3.5*roadWidth;
		int rd = ri - mid - catLength / 2;
		cout << "rd:" << rd << endl;
		int ld = mid - le - catLength / 2;
		cout << "ld:" << ld << endl;
		cout << (float)rd / ld << endl;
		cout << endl;
	}
}
void LaneDetection::judgeThres()
{
	float lefthe = lthe / (CV_PI / 180);//左角度
	float righthe = rthe / (CV_PI / 180);//右角度
}
void LaneDetection::judgeTest()
{
	float lefthe = lthe / (CV_PI / 180);//左角度
	float righthe = rthe / (CV_PI / 180);//右角度
	float llthe = 90 - lefthe;
	float rrthe = righthe - 90;
	//cout << "llthe:" << llthe << "rrthe:" << rrthe << endl;
	if (currentStatue == 0)
	{
		if (lefthe > 0 && righthe> 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (abs(llthe - rrthe) <= 20 || (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1))
			{
				if (departureCount > 0)
					departureCount--;
				else if (departureCount < 0)
					departureCount++;
			}
			else if (llthe > rrthe&&llthe > ltheta&&llthe > 40)
			{
					departureCount--;
			}
			else if (llthe < rrthe&&rrthe>rtheta&&rrthe > 40)
			{
					departureCount++;
			}
		}
		else if (lefthe < 0 && righthe>0)
		{
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (rrthe>rtheta&& rrthe>40)
				departureCount++;
		}
		else if (lefthe > 0 && righthe < 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe > ltheta&& llthe>40)
				departureCount--;
		}
		else{
			if (departureCount > 0)
				departureCount--;
			else if (departureCount < 0)
				departureCount++;
		}
	}
	else if (currentStatue == -1)
	{
		if (lefthe > 0 && righthe > 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (llthe > ltheta&&rrthe < rtheta)
				departureCount--;
			else if (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1)
				departureCount++;
			else if (llthe <ltheta&&rrthe>rtheta)
				departureCount++;
		}
		else if (lefthe < 0 && righthe>0)
		{
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (rrthe < rtheta)
				departureCount--;
			else if (abs(rrthe - rtheta) < 1)
				departureCount++;
			else if (rrthe > rtheta + 2)
				departureCount++;
		}
		else if (lefthe > 0 && righthe < 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe > ltheta)
				departureCount--;
			else if (abs(llthe - ltheta) < 1)
				departureCount++;
			else if (llthe < ltheta - 2)
				departureCount++;
		}	

	}
	else if (currentStatue == 1)
	{
		if (lefthe > 0 && righthe> 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (llthe< ltheta&&rrthe>rtheta)
				departureCount++;
			else if (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1)
				departureCount--;
			else if (llthe > ltheta&&rrthe < rtheta)
				departureCount--;
		}
		else if (lefthe < 0 && righthe>0)
		{
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (rrthe > rtheta)
				departureCount++;
			else if (abs(rrthe - rtheta)< 1)
				departureCount--;
			else if (rrthe < rtheta - 2)
				departureCount--;
		}
		else if (lefthe > 0 && righthe < 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe < ltheta)
				departureCount++;
			else if (abs(llthe - ltheta) < 1)
				departureCount--;
			else if (llthe > ltheta + 2)
				departureCount--;
		}	
	}
	if (departureCount > DEPARTURE_THRES)
		departureCount = DEPARTURE_THRES;
	else if (departureCount <- DEPARTURE_THRES)
		departureCount = -DEPARTURE_THRES;
	//cout << departureCount << endl;
	if (departureCount>DEPARTURE_COUTN)
		currentStatue = 1;
	else if (departureCount <-DEPARTURE_COUTN)
		currentStatue = -1;
	else currentStatue = 0;
}
void LaneDetection::judge(Mat &src)
{
	float lefthe = lthe / (CV_PI / 180);//左角度
	float righthe = rthe / (CV_PI / 180);//右角度
	float llthe = 90 - lefthe;
	float rrthe = righthe - 90;
	//float ltheta = preLeftResult[0][1], rtheta = preRightResult[0][1];
	//cout << "lthe:" << left << endl;
	//cout << "right:" << right << endl;
	if (currentStatue == 0)
	{
		if (lefthe > 0 && righthe> 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			//	cout << "llthe:" << llthe << "rrthe:" << rrthe << endl;
			//	cout << "ltheta:" << ltheta << "rtheta" << rtheta << endl;
			//	cout << endl;
			if (abs(llthe - rrthe) <= 20 || (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1))
				currentStatue = 0;
			else if (llthe > rrthe&&llthe > ltheta&&llthe>40)
				currentStatue = -1;
			else if (llthe < rrthe&&rrthe>rtheta&&rrthe>40)
				currentStatue = 1;
		}
		else if (lefthe < 0 && righthe>0)
		{
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (rrthe>rtheta && rrthe>40)
				currentStatue = 1;
		}
		else if (lefthe > 0 && righthe < 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe > ltheta  && llthe>40)
				currentStatue = -1;
		}
	}
	else if (currentStatue == -1)
	{
		//cout << "left" << endl;
		if (lefthe > 0 && righthe > 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (llthe > ltheta&&rrthe<rtheta)
				currentStatue = -1;
			else if (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1)
				currentStatue = 0;
			else if (llthe <ltheta&&rrthe>rtheta)
				currentStatue = 1;
		}
		else if (lefthe < 0 && righthe>0)
		{
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			if (rrthe < rtheta)
				currentStatue = -1;
			else if (abs(rrthe - rtheta) < 1)
				currentStatue = 0;
			else if (rrthe > rtheta + 2)
				currentStatue = 1;
		}
		else if (lefthe > 0 && righthe < 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe > ltheta)
				currentStatue = -1;
			else if (abs(llthe - ltheta) < 1)
				currentStatue = 0;
			else if (llthe < ltheta - 2)
				currentStatue = 1;
		}
	}
	else if (currentStatue == 1)
	{
		//cout << "right" << endl;
		if (lefthe > 0 && righthe> 0)
		{
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180), rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;
			//cout << "00" << endl;
			//cout << "llthe:" << llthe << "rrthe:" << rrthe << endl;
			//cout << "ltheta:" << ltheta << "rtheta" << rtheta << endl;
			if (llthe< ltheta&&rrthe>rtheta)
				currentStatue = 1;
			else if (abs(llthe - ltheta) < 1 && abs(rrthe - rtheta) < 1)
				currentStatue = 0;
			else if (llthe > ltheta&&rrthe < rtheta)
				currentStatue = -1;

		}
		else if (lefthe < 0 && righthe>0)
		{
			//	cout << "10" << endl;
			float rtheta = preRightResult[0][1] / (CV_PI / 180) - 90;

			if (rrthe > rtheta)
				currentStatue = 1;
			else if (abs(rrthe - rtheta)< 1)
				currentStatue = 0;
			else if (rrthe < rtheta - 2)
				currentStatue = -1;

		}
		else if (lefthe > 0 && righthe < 0)
		{
			//	cout << "01" << endl;
			float ltheta = 90 - preLeftResult[0][1] / (CV_PI / 180);
			if (llthe < ltheta)
				currentStatue = 1;
			else if (abs(llthe - ltheta) < 1)
				currentStatue = 0;
			else if (llthe > ltheta + 2)
				currentStatue = -1;
		}
	}
}