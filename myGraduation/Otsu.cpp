#include"Otsu.h"
#include<iostream>
using namespace std;


int Otsu(Mat& gray)
{
	int hist[256];
	memset(hist, 0, sizeof(hist));
	int nl = gray.rows;
	int nc = gray.cols;
	if (gray.isContinuous())
	{
		nc = nc*nl;
		nl = 1;
	}
	for (int j = 0; j < nl; j++)
	{
		uchar *data = gray.ptr<uchar>(j);
		for (int i = 0; i < nc; i++)
		{
			hist[data[i]]++;
		}
	}
	int start = 0,end=255;
	while (hist[start] == 0)
		start++;
	while (hist[end] == 0)
		end--;
	double countValue = 0;
	int count = nl*nc;
	for (int i = start; i <= end; i++)
		countValue += (double)i*hist[i] / count;
	double	pHist = 0;
	double a1 = 0, a2 = 0, u1 = 0, u2 = 0, threshold = 0,maxvalue=0,pos=0,sum=0;
	for (int i = start; i <=end; i++)
	{
		pHist =(double) hist[i] / count;
		a1 += pHist;
		a2 = 1 - a1;
		if (a2 < DBL_EPSILON || a1<DBL_EPSILON)
			continue;
		sum += i*pHist;
		u1 = sum / a1;
		u2 = (countValue - a1*u1) / a2;
	//	cout << a1 << " " << a2 <<"  " << u1<<"  "<<u2<<endl;
		threshold = a1*a2*(u1 - u2)*(u1 - u2);
	//	cout << threshold << endl;
		if (threshold > maxvalue)
		{
			maxvalue = threshold;
			pos = i;
		}
	}
	//cout << pos << endl;
	return pos;
}