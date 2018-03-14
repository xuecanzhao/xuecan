#include"cross.h"

Point CrossPoint(const LINE *line1, const LINE *line2)
{
	//    if(!SegmentIntersect(line1->pStart, line1->pEnd, line2->pStart, line2->pEnd))  
	//    {// segments not cross     
	//        return 0;  
	//    }  
	Point pt;
	// line1's cpmponent  
	double X1 = line1->pEnd.x - line1->pStart.x;//b1  
	double Y1 = line1->pEnd.y - line1->pStart.y;//a1  
	// line2's cpmponent  
	double X2 = line2->pEnd.x - line2->pStart.x;//b2  
	double Y2 = line2->pEnd.y - line2->pStart.y;//a2  
	// distance of 1,2  
	double X21 = line2->pStart.x - line1->pStart.x;
	double Y21 = line2->pStart.y - line1->pStart.y;
	// determinant  
	double D = Y1*X2 - Y2*X1;// a1b2-a2b1  
	//   
	if (D == 0) return 0;
	// cross point  
	pt.x = (X1*X2*Y21 + Y1*X2*line1->pStart.x - Y2*X1*line2->pStart.x) / D;
	// on screen y is down increased !   
	pt.y = -(Y1*Y2*X21 + X1*Y2*line1->pStart.y - X2*Y1*line2->pStart.y) / D;
	// segments intersect.  
	if ((abs(pt.x - line1->pStart.x - X1 / 2) <= abs(X1 / 2)) &&
		(abs(pt.y - line1->pStart.y - Y1 / 2) <= abs(Y1 / 2)) &&
		(abs(pt.x - line2->pStart.x - X2 / 2) <= abs(X2 / 2)) &&
		(abs(pt.y - line2->pStart.y - Y2 / 2) <= abs(Y2 / 2)))
	{
		return pt;
	}
	return 0;
}