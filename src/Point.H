/* 
 * Point class
 * 	mhossain
 *	07 Sept. 2010
 */


#ifndef _POINT_H
#define	_POINT_H

#include <vector>



using namespace std;

class Point 
{
private:
	double x;
	double y;
	//double xth;
	double yth;
	int id;
        double oAngle;
public:	
	Point(){};
	Point(double a, double b);
	~Point(){};

	double X();
	double Y();
	void set(double a, double b);
	void display();
	Point operator-(Point p);
	Point operator+(Point p);
	double operator*(Point p);

	//double angleWithYaxis();
	
	double getYth();
	void setYth();
	double getID();
	void setID(int a);
        
        void setOAngle(double ang);
        double getOAngle();

};
void displayPoints(vector<Point> points);

void addTwoVectorsOfPoints(vector<Point>& result, vector<Point> points);
bool sortA2OrtAngle(Point p1, Point p2);

#endif
