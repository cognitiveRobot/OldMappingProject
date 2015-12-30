/*		 Surface Class
			by Jhon
			modified by mhossain
*/

#include <iostream>
#include <vector>


#include "Point.H"
#include <cmath>

using namespace std;
#define PI 3.14159265


Point::Point(double a, double b)
{
	x=a; y=b;
}

double Point::X(){ return x;}
double Point::Y(){ return y;}

void Point::set(double a, double b)
{
	x=a; y=b;
}

void Point::display()
{
	//setYth();
	cout<<id<<"  X: "<<x<<", Y: "<<y<<" yth: "<<yth<<endl;
}

Point Point::operator-(Point p)
{
 return Point(x-p.x, y-p.y);
}

Point Point::operator+(Point p)
{
 return Point(x+p.x, y+p.y);
}

// dot product of two points/vectors
double Point::operator*(Point p)
{
 return ((x*p.x)+(y*p.y));
}

void Point::setYth() {
	double x21=x-0;
	double y21=y-0;

	double length=sqrt(x*x+y*y);
	
	double angle1=acos(x21/length);

	if(y21 < 0)
	angle1 = 2*PI - angle1;	

	yth=(90-((180/PI)*angle1));
}

double Point::getYth() {
	return yth;
}

double Point::getID() {
	return id;
}

void Point::setID(int a) {
	id=a;
}

void Point::setOAngle(double ang) {
    oAngle = ang;
}
        double Point::getOAngle() {
            return oAngle;
        }

void displayPoints(vector<Point> points) 
{
	for(int i=0;i<int(points.size());i++) {
		points[i].display();
	}
}

void addTwoVectorsOfPoints(vector<Point> &result, vector<Point> points) {
    for(unsigned int i=0; i<points.size(); i++) {
        result.push_back(points[i]);
    }
}

bool sortA2OrtAngle(Point p1, Point p2) {
    return p1.getOAngle() > p2.getOAngle();
}

