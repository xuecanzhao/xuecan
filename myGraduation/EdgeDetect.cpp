#include"EdgeDetect.h"
#include"Otsu.h"

void edgeDetect(Mat& src,Mat&dst)
{
	dst = Mat::zeros(src.size(), CV_8UC1);
	GaussianBlur(src, src, Size(3, 3), 0, 0);
	//int thres=Otsu(src);
	//threshold(src, src, thres, 255, 0);
	int rows = src.rows;
	int start =20;
	int cols = src.cols-start;
	int difference = 0;
	for (int i = 0; i < rows; i++)
	{
		uchar* ptr = src.ptr<uchar>(i);
		uchar* data = dst.ptr<uchar>(i);
		for (int j = start; j < cols; j++)
		{
			if (ptr[j] != 0)
			{
				difference = 2 * ptr[j] - ptr[j + start] - ptr[j - start]- abs((int)(ptr[j - start] - ptr[j + start]));
				if (difference < 0)
					difference = 0;
				if (difference > 255)
					difference = 255;
				data[j] = (unsigned char)(difference);
			}
		}
	}
	int thres = Otsu(dst);
	//GaussianBlur(dst, dst, Size(3, 3), 0, 0);
	threshold(dst, dst, thres, 255, 0);
	//namedWindow("src");
	//imshow("src", dst);
	//waitKey(0);
}