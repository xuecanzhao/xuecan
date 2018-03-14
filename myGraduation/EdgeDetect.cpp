#include"EdgeDetect.h"
#include"Otsu.h"

void edgeDetect(Mat& src,Mat&dst)
{
	dst = Mat::zeros(src.size(), CV_8UC1);
	//GaussianBlur(src, src, Size(3, 3), 0, 0);
	//int thres=Otsu(src);
	//threshold(src, src, thres, 255, 0);
	int rows = src.rows;
	int start =15;
	//int cols = src.cols-start;
	int difference = 0;
	for (int i = 0; i < rows; i++)
	{
		uchar* ptr = src.ptr<uchar>(i);
		uchar* data = dst.ptr<uchar>(i);
		int cols = src.cols - start;
		for (int j = start; j < cols; j++)
		{
			if (ptr[j] != 0)
			{
				//difference = 2 * ptr[j] - ptr[j + start] - ptr[j - start]- abs((int)(ptr[j - start] - ptr[j + start]));
				difference = 2 * ptr[j] - ptr[j + start] - ptr[j - start];
				if (difference < 0)
					difference = 0;
				if (difference > 255)
					difference = 255;
				data[j] = (unsigned char)(difference);
			}
		}
		//if (start > 10)
		//	start--;
	}
/*	Mat mm;
	int width = dst.cols;
	int height = dst.rows;
	int left = 0;
	int right = width;
	int top = height / 3;
	int bottom = height - 2;
	Rect r1(left, top, right - left, bottom - top);
	dst(r1).copyTo(mm);*/

	int thres = Otsu(dst);
	//medianBlur(dst, dst, 3);
	GaussianBlur(dst, dst, Size(3, 3), 0, 0);
	threshold(dst, dst, thres, 255, 0);
	//namedWindow("src");
	//imshow("src", dst);
	//waitKey(0);
	/*for (int i = 0; i < rows; i++)
	{
		uchar* ptr = dst.ptr<uchar>(i);
		int cols = src.cols - start;
		int j = start;
		while (j < cols)
		{
			while (j < cols&&ptr[j] == 0)
				j++;
			int k = j;
			while (k < cols&&ptr[k] == 255)
				k++;
			for (int m = j; m < j + 3 * (k - j) / 8; m++)
				ptr[m] = 0;
			for (int m = j; m < j + 5 * (k - j) / 8; m++)
				ptr[m] = 0;
			j = k;
		}

	}*/
}