#include <opencv2\highgui\highgui.hpp>  
#include <opencv2\opencv.hpp>  
using namespace std;
using namespace cv;

struct PT
{
	int x;
	int y;
};
struct LINE
{
	PT pStart;
	PT pEnd;
};
Point CrossPoint(const LINE *line1, const LINE *line2);