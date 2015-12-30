#include <vector>
#include <iostream>
#include <cmath>

#include <iostream>
#include <cmath>
#include <deque>
#include "readAndwriteASCII.H"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

//#include "clipper.hpp"

#include "GeometricOp.H"

#include "Point.H"
#include "Object.H"
#include "mfisOp.H"
#include "Minfo.H"
#include "Plotting.H"
using namespace std;
namespace bg = boost::geometry;

#define PI 3.14159265



//transform previous view into current view

vector<Object> xformPVIntoCV(vector<Object> pview, Point rpose, double angle) {//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
    vector<Object> result;
    double x1, y1, x2, y2;
    //angle=angle*(180/PI);
    for (int i = 0; i<int(pview.size()); i++) {

        double a = pview[i].X1() - rpose.X(); //x-x0
        double b = pview[i].Y1() - rpose.Y(); //y-y0

        x1 = a * cos(angle) + b * sin(angle);
        y1 = b * cos(angle) - a * sin(angle);

        double c = pview[i].X2() - rpose.X(); //x-x0
        double d = pview[i].Y2() - rpose.Y(); //y-y0

        x2 = c * cos(angle) + d * sin(angle);
        y2 = d * cos(angle) - c * sin(angle);

        Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());
        s.setLocalEnvID(pview[i].getLocalEnvID());
        result.push_back(s);
    }
    return result;
}

vector<Point> xformPointsIntoCV(vector<Point> pview, Point rpose, double angle) {
    vector<Point> result;
    double x1, y1;
    //angle=angle*(180/PI);
    for (int i = 0; i<int(pview.size()); i++) {

        double a = pview[i].X() - rpose.X(); //x-x0
        double b = pview[i].Y() - rpose.Y(); //y-y0

        x1 = a * cos(angle) + b * sin(angle);
        y1 = b * cos(angle) - a * sin(angle);

        
        //Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());
        result.push_back(Point(x1,y1));
    }
    return result;
}


//transform potential objects of pv into current view
vector<Object> xformPObjectsIntoCV(vector<Object> pview, Point rpose, double angle)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
//angle=angle*(180/PI);
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X1()-rpose.X(); //x-x0
		double b=pview[i].Y1()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);
		
		double c=pview[i].X2()-rpose.X(); //x-x0
		double d=pview[i].Y2()-rpose.Y(); //y-y0
	
		x2=c*cos(angle)+d*sin(angle);
		y2=d*cos(angle)-c*sin(angle);
		
//                Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());				
//                s.setKP(pview[i].getKP());
//                result.push_back(s);
		
		
				Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());				s.setKP(pview[i].getKP());
				result.push_back(s);
	
		}
	return result;
}

//transform potential objects of pv into current view
vector<Object> xformPObjectsIntoCVForVision(vector<Object> pview, Point rpose, double angle)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
//angle=angle*(180/PI);
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X1()-rpose.X(); //x-x0
		double b=pview[i].Y1()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);
		
		double c=pview[i].X2()-rpose.X(); //x-x0
		double d=pview[i].Y2()-rpose.Y(); //y-y0
	
		x2=c*cos(angle)+d*sin(angle);
		y2=d*cos(angle)-c*sin(angle);
		
//                Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());				
//                s.setKP(pview[i].getKP());
//                result.push_back(s);
		
		if(y1 > 0) {
			
			
			if(y2 > 0) {
				Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());				s.setKP(pview[i].getKP());
				result.push_back(s);
			}
		}
		//for first object
		if(y1 < 0 && y2 > 500 && pview[i].getKP() > 1) {
			Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());			s.setKP(2);
			result.push_back(s);
		}
		//for last object
		if(y1 > 500 && y2 < 0 && pview[i].getKP() == 1) {
			Object s(x1, y1, x2, y2, pview[i].getID(), pview[i].nearness(), pview[i].getP1OS(), pview[i].getP2OS(), pview[i].getGID());			s.setKP(1);
			result.push_back(s);
		}
		//cout<<"id "<<pview[i].getID()<<" KP- "<<pview[i].getKP()<<" y1 "<<y1<<" y2 "<<y2<<endl;
	}
	return result;
}



vector<Object> discardLinesIFoR(vector<Object> pview, Point rpose, double angle, vector<Object> cview, Object rmfis,Object rcv,int rp)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
	//finding left and right boundary from current view
	double leftb=0;
	double rightb=0;
	double max=0;
	for(int i=0;i<int(cview.size());i++) {
		if(cview[i].X1() < leftb) {//have problem
			leftb=cview[i].X1();
		}
		if(cview[i].X2() < leftb) {
			leftb=cview[i].X2();
		}

		if(cview[i].X1() > rightb) {
			rightb=cview[i].X1();
		}
		if(cview[i].X2() > rightb) {
			rightb=cview[i].X2();
		}

		if(cview[i].Y1() > max) {
			max=cview[i].Y1();
		}
		if(cview[i].Y2() > max) {
			max=cview[i].Y2();
		}
	}
	
	//cout<<"cv "<<endl;
	//displayObjects(cview);
	//cout<<endl<<"left b "<<leftb<<" right b"<<rightb<<" max "<<max<<endl;
//angle=angle*(180/PI);
/*
	if(leftb > 10000) {
		leftb = 10000;
	}
	if(rightb > 10000) {
		rightb=10000;
	}
	if(max > 10000) {
		max=10000;
	}*/
leftb=-10000;rightb=10000;max=12000;

	bool insert_first_line_of_cv=true;
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X1()-rpose.X(); //x-x0
		double b=pview[i].Y1()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);

		double c=pview[i].X2()-rpose.X(); //x-x0
		double d=pview[i].Y2()-rpose.Y(); //y-y0

		x2=c*cos(angle)+d*sin(angle);
		y2=d*cos(angle)-c*sin(angle);
		
		bool blines=true;
		/*
			condition (y1>0 or y2>0) is to replace the line which is on both side
			condition (y1>0) and then again (y2>0) is to keep both of them

		*/
		//for replacement
		if(y1 > 0|| y2 > 0) {
			
			
			//if(y2 > 0) {
			if(x1 < 0 && x1 > leftb) {
				//blines=false;
				if(y1 > 0 && y2 > 0) {
					if(y1 < max && y2 < max) 
						blines=false;
				}
			}
			if(x1 > 0 && x1 < rightb) {
				//blines=false;
				if(y1 > 0 && y2 > 0) {
					if(y1 < max && y2 < max) 
						blines=false;
				}
			}

			
			//}
		}

		//for using both
	/*	if(y1 > 0 && y2 > 0) {
			
			
			//if(y2 > 0) {
			if(x1 < 0 && x1 > leftb) {
				blines=false;
			}
			if(x1 > 0 && x1 < rightb) {
				blines=false;
			}
			//}
		}*/


		//for extending
		if(y1 < 0 && y2 > 0) {
			Object line1=remakeLineP2(rmfis,rcv,cview[0],1,0, rp);
			double s_dist=pview[i].perpendicularDistOfPoint(line1.X1(),line1.Y1());
			//cout<<"extending mfis Object as first Object of cview "<<s_dist<<endl;
			if(s_dist < 300) {
				pview[i].setP2(line1.X2(),line1.Y2());
				result.push_back(pview[i]);
				blines = false;
			}
			else
				result.push_back(line1);
			insert_first_line_of_cv=false;
		}

		if(y1 > 0 && y2 < 0) {
			Object line1=remakeLineP2(rmfis,rcv,cview[cview.size()-1],1,0, rp);
			double s_dist=pview[i].perpendicularDistOfPoint(line1.X1(),line1.Y1());
			//cout<<"extending mfis Object as last Object of cview "<<s_dist<<endl;
			if(s_dist < 300) {
				pview[i].setP1(line1.X1(),line1.Y1());
				result.push_back(pview[i]);
				blines = false;
			}
			//else
			//	result.push_back(line1);
		}
		

		
		if(blines==true) {
			result.push_back(pview[i]);
		}

		
	}
	if(insert_first_line_of_cv == true){
		Object line1=remakeLineP2(rmfis,rcv,cview[0],1,0, rp);
		result.push_back(line1);
	}
		
	return result;
}




/*
//transform previous ref points into current view
vector<RfPoint> xformRefPointsIntoCV(vector<RfPoint> pview, Point rpose, double angle)
{
	vector<RfPoint> result;
	double x1, y1;
//angle=angle*(180/PI);
	for(int i=0;i<int(pview.size());i++)
	{
				
		double a=pview[i].X()-rpose.X(); //x-x0
		double b=pview[i].Y()-rpose.Y(); //y-y0

		x1=a*cos(angle)+b*sin(angle);
		y1=b*cos(angle)-a*sin(angle);
		
		if(y1 > 0) {
			RfPoint p(x1, y1);
			p.setYth();
			p.setID(pview[i].getID());
			result.push_back(p);
		}
	}
	return result;
}
*/

//most probably it doesn't work. CHECK
//transform current view into previous view
vector<Object> xformCVIntoPV(vector<Object> cview, Point rpose, double angle)
{//cout<<"r pos: "<<rpose.X()<<" "<<rpose.Y()<<" angle: "<<angle<<endl;
	vector<Object> result;
	double x1, y1, x2, y2;
//angle=angle*(180/PI);
	for(int i=0;i<int(cview.size());i++)
	{
				
		x1=cview[i].X1()*cos(angle)-cview[i].Y1()*sin(angle)+rpose.X();
		y1=cview[i].X1()*sin(angle)+cview[i].Y1()*cos(angle)+rpose.Y();
		
		x2=cview[i].X2()*cos(angle)-cview[i].Y2()*sin(angle)+rpose.X();
		y2=cview[i].X2()*sin(angle)+cview[i].Y2()*cos(angle)+rpose.Y();
		
		Object s(x1, y1, x2, y2, cview[i].getID(), cview[i].nearness(), 
                                        cview[i].getP1OS(), cview[i].getP2OS(), cview[i].getGID());
		result.push_back(s);
		
	}
	return result;
}


//returns intersection point of two lines
vector<double> getIntersectionPoint(Object s1, Object s2)
{
	
	double x1= s1.X1();	
	double y1= s1.Y1();		
	double x2= s1.X2();	
	double y2= s1.Y2();	

	double x3= s2.X1();
	double y3= s2.Y1();
	double x4= s2.X2();
	double y4= s2.Y2();

	double y43= y4-y3;
	double x21= x2-x1;
	double x43= x4-x3;
	double y21= y2-y1;

	double u_deno=y43*x21-x43*y21;
	
	double y13= y1-y3;
	double x13= x1-x3;
	double ua_num= x43*y13-y43*x13;
	//double ub_num= x21*y13-y21*x13;

	double ua=ua_num/u_deno;
	//double ub=ub_num/u_deno;
	
	double x=x1+ua*x21;
	double y=y1+ua*y21;
	
	vector<double> result;
	result.push_back(x);
	result.push_back(y);

	//cout<<" x "<<x<<" y "<<y<<endl;
	return result;
}

//return 1 if both segment has intersection point
//works. 11.08.14
double checkForIntersection (Object s1, Object s2)
{
					
 
	double x1= s1.X1();	
	double y1= s1.Y1();
	double x2= s1.X2();
	double y2= s1.Y2();

	double x3= s2.X1();
	double y3= s2.Y1();
	double x4= s2.X2();
	double y4= s2.Y2();

	double y43= y4-y3;
	double x21= x2-x1;
	double x43= x4-x3;
	double y21= y2-y1;

	double u_deno=y43*x21-x43*y21;
	
	double y13= y1-y3;
	double x13= x1-x3;
	double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	//cout<<"ua "<<ua<<" ub "<<ub<<endl;
	if(ua>=0 && ua<=1)
		if(ub>=0 && ub<=1)
			return 1;
		else
		return 0;
	else
	return 0;
}


//it returns abs distance between two points
double getDistBtw2Points(double x1, double y1, double x2, double y2)
{
	double x=x1-x2;
	double y=y1-y2;
	return sqrt(x*x+y*y);
}

double angleObjectAndPoint(Object s, double x3, double y3)
{
	double x21=s.X2()-s.X1();
	double y21=s.Y2()-s.Y1();
	double x31=x3-s.X1();
	double y31=y3-s.Y1();
	
	double la=s.length();
	double lb=getDistBtw2Points(s.X1(), s.Y1(), x3, y3);

	/*if(lb==0)
	return 0;
	
	double r=(x21*x31+y21*y31)/(la*lb);*/
	double r1=x21/la;
	double angle1=acos(r1);

	double r2=x31/lb;
	double angle2=acos(r2);
	
	if(lb==0)
	return 0;

	if(y21 < 0)
	angle1 = 2*PI - angle1;	
	angle1=((180/PI)*angle1);
	
	if(y31<0)
	angle2=2*PI-angle2;
	angle2=((180/PI)*angle2);
	
	double diff=angle2-angle1;

	if(diff>0)
	return diff;
	else 
	return 360+diff;
}

double angleObjectAndXaxis(Object s)
{
	double x21=s.X2()-s.X1();
	double y21=s.Y2()-s.Y1();
	
	
	double la=s.length();
	
	/*if(lb==0)
	return 0;
	
	double r=(x21*x31+y21*y31)/(la*lb);*/
	double r1=x21/la;
	double angle1=acos(r1);

	

	if(y21 < 0)
	angle1 = 2*PI - angle1;	
	

	return ((180/PI)*angle1);	
}
//not general it's only for robot
//converts polar to cartesian coordinates for robot position 
vector<Point> p2cCoordinate(vector<double> vec)
{
	vector<Point> result;
	for(int i=0;i<int(vec.size());i+=2)
	{
		double angle=vec[i];
		double dist=vec[i+1];

		angle = (angle/180)*PI;
		double x=dist*sin(angle);
		double y=dist*cos(angle);
		
		Point p(x, y);
		result.push_back(p);
	}
	return result;
}

//it returns x coordinate in certesian system
//angle in deg and with respect to y axis
double getX(double angle,double dist) {
	return dist*sin(-(PI/180)*angle);
}

//it returns y coordinate in certesian system
//angle in deg and with respect to y axis
double getY(double angle,double dist) {
	return dist*cos(-(PI/180)*angle);
}


//it return 1 if inside else 0
//Ref: http://paulbourke.net/geometry/insidepoly/
//solution 3
bool pointInPolygon(Object a, Object b, double x, double y) {
	//double x0,y0,x1,y1;
	double test;
	vector<double> px, py;
	px.push_back(a.X1());
	px.push_back(a.X2());
	px.push_back(b.X2());
	px.push_back(b.X1());
	px.push_back(a.X1());

	py.push_back(a.Y1());
	py.push_back(a.Y2());
	py.push_back(b.Y2());
	py.push_back(b.Y1());
	py.push_back(a.Y1());

	bool inside=true;
	for(int i=0;i<4;i++) {
		test=((y-py[i])*(px[i+1]-px[i]))-((x-px[i])*(py[i+1]-py[i]));
		if(test > 0) {
			inside = false;
			return inside;
		}
	}
	return inside;
			
}

vector<Object> breakTheLineInto(Object smfis, int gapBetweenDots)
{
	vector<Object> output;
	double x1, y1, x2, y2;
	double angle=0;
	double dist=0;
	x1=smfis.X1();
	y1=smfis.Y1();

		int id=0;
		while(dist < double(smfis.length()-50)) {
		dist=dist+50;
		x2=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle);
		y2=((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X1();
		y2=y2*dist+smfis.Y1();

		Object oneline(x1,y1,x2,y2, id+1);
		output.push_back(oneline);

		dist=dist + gapBetweenDots;
		x2=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle);
		y2=((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X1();
		y2=y2*dist+smfis.Y1();
		x1=x2;
		y1=y2;
		}
	return output;
}

vector<Object> breakTheLinesInto(vector<Object> allLines) {
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = breakTheLineInto(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
}

//it makes a line with respect to object using angle and dist.

Object makeLineAtPointWithObject(double angle, double dist, Object smfis) {
    Object output;
    double x1, y1, x2, y2;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    angle = (angle / 180) * PI; //angle in radian

    x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x1 = x1 * dist + smfis.X1();
    y1 = y1 * dist + smfis.Y1();

    dist = dist + 300;
    x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x2 = x2 * dist + smfis.X1();
    y2 = y2 * dist + smfis.Y1();

    output.set(x1, y1, x2, y2, 1);

    return output;
}

//it makes a line with respect to object using angle and dist.
//length is the length of object

Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis) {
    Object output;
    double x1, y1, x2, y2;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    angle = (angle / 180) * PI; //angle in radian

    x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x1 = x1 * dist + smfis.X1();
    y1 = y1 * dist + smfis.Y1();

    dist = dist + length;
    x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x2 = x2 * dist + smfis.X1();
    y2 = y2 * dist + smfis.Y1();

    output.set(x1, y1, x2, y2, 1);

    return output;
}

//it makes a line with respect to object using angle and dist.
//length is the length of object
//argument angle is in radian

Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis, int a) {
    Object output;
    double x1, y1, x2, y2;
    x1 = smfis.X1();
    y1 = smfis.Y1();

    //angle = (angle / 180) * PI; //angle in radian

    x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x1 = x1 * dist + smfis.X1();
    y1 = y1 * dist + smfis.Y1();

    dist = dist + length;
    x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle);
    y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle);

    x2 = x2 * dist + smfis.X1();
    y2 = y2 * dist + smfis.Y1();

    output.set(x1, y1, x2, y2, 1);

    return output;
}

vector<Object> makeSquareAtLoSPoints(vector<Object> lineOfSitePoints) {
    Object obj1,obj2,side;
    vector<Object> output;
    for(int i=0;i<int(lineOfSitePoints.size());i++) {
        obj1 = makeLineAtPointWithObject(45,0,500,lineOfSitePoints[i]);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        obj1.reverse();
        obj1 = makeLineAtPointWithObject(90,0,500,obj1);
        output.push_back(obj1);
        
//        obj2 = makeLineAtPointWithObject(315,0,lineOfSitePoints[i]);
//        
//        
//        side.set(obj1.X2(),obj1.Y2(),lineOfSitePoints[i].X2(),lineOfSitePoints[i].Y2(),2);
//        output.push_back(side);
//        side.set(lineOfSitePoints[i].X2(),lineOfSitePoints[i].Y2(),obj2.X2(),obj2.Y2(),3);
//        output.push_back(side);
//        output.push_back(obj2);
//        output.push_back(lineOfSitePoints[i]);
    }
    return output;
}

//it takes angle and dist of two points with respect to the given point
//make and return the line at those two point
///*
Object makeLineAtTwoPointsWithObject(double angle1, double dist1, double angle2, double dist2, Object smfis,int referencePoint) {
    Object output;
    double x1, y1, x2, y2;


    angle1 = (angle1 / 180) * PI; //angle in radian
    angle2 = (angle2 / 180) * PI; //angle in radian

    if (referencePoint == 1) {
        x1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle1)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle1);
        y1 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle1)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle1);

        x1 = x1 * dist1 + smfis.X1();
        y1 = y1 * dist1 + smfis.Y1();

        x2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * cos(angle2)-((smfis.Y2() - smfis.Y1()) / smfis.length()) * sin(angle2);
        y2 = ((smfis.X2() - smfis.X1()) / smfis.length()) * sin(angle2)+((smfis.Y2() - smfis.Y1()) / smfis.length()) * cos(angle2);

        x2 = x2 * dist2 + smfis.X1();
        y2 = y2 * dist2 + smfis.Y1();
    }
    else {
        x1 = ((smfis.X1() - smfis.X2()) / smfis.length()) * cos(angle1)-((smfis.Y1() - smfis.Y2()) / smfis.length()) * sin(angle1);
        y1 = ((smfis.X1() - smfis.X2()) / smfis.length()) * sin(angle1)+((smfis.Y1() - smfis.Y2()) / smfis.length()) * cos(angle1);

        x1 = x1 * dist1 + smfis.X2();
        y1 = y1 * dist1 + smfis.Y2();

        x2 = ((smfis.X1() - smfis.X2()) / smfis.length()) * cos(angle2)-((smfis.Y1() - smfis.Y2()) / smfis.length()) * sin(angle2);
        y2 = ((smfis.X1() - smfis.X2()) / smfis.length()) * sin(angle2)+((smfis.Y1() - smfis.Y2()) / smfis.length()) * cos(angle2);

        x2 = x2 * dist2 + smfis.X2();
        y2 = y2 * dist2 + smfis.Y2();
    }

    output.set(x1, y1, x2, y2, 1);

    return output;
}

void waitHere() {
    char wait[10];
    cout << "Press any key to continue .. ..";
    cin>>wait;
    return;
}

vector<Object> makeSquare(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, 1000, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //right side line
            exit.push_back(tempObj);
            
            return exit;
}

vector<Object> makeAllSquare(vector<Object> allLines) {
    
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = makeSquare(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
    
}

vector<Object> makeRectangle(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, 500, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
            exit.push_back(tempObj);
            
            return exit;
}

vector<Object> makeAllRectangle(vector<Object> allLines) {
    
    vector<Object> result;
    vector<Object> temp;
    for(unsigned int i=0;i<allLines.size();i++) {
        temp = makeRectangle(allLines[i]);
        for(unsigned int j=0;j<temp.size();j++)
                result.push_back(temp[j]);
    }
    return result;
    
}

Object makeParallelObject(Object oneSide, double dist, const char side) {
    Object tempObj;
    //cout<<side<<endl;
    if(side == 'left') {
        
    tempObj = makeLineAtPointWithObject(90, 0, dist, oneSide);
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(-90, 0, 1000, tempObj);
    }
    else
      tempObj = makeLineAtPointWithObject(-90, 0, dist, oneSide);  
    
    
    
    return tempObj;
    //waitHere();
}

Object makeParallelObject(Object oneSide, double dist, int side) {
    Object tempObj;
    //cout<<side<<endl;
    if(side == 1) {//left
        
    tempObj = makeLineAtPointWithObject(90, dist, dist, oneSide);
    //tempObj.reverse();
    tempObj = makeLineAtPointWithObject(-90, 0, oneSide.length(), tempObj);
    }
    else
      tempObj = makeLineAtPointWithObject(-90, 0, dist, oneSide);  
    
    
    
    return tempObj;
    //waitHere();
}

vector<Object> makeArrow(Object oneSide) {
    vector<Object> exit;
    Object tempObj;
    exit.push_back(oneSide);
    
    oneSide.reverse();
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(45, 0, 500, oneSide); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            
            tempObj = makeLineAtPointWithObject(-45, 0, 500, oneSide); //parallel line of original exit
            exit.push_back(tempObj);
                        
            return exit;
}

//it computes a boundary which is 30cm outside from cView
vector<Surface> makePolygonOfCV(vector<Object> cView) {
       vector<Surface> polygon;
       
       vector<Object> newCView;
       for(unsigned int i=0;i<cView.size();i++) {
           newCView.push_back(makeParallelObject(cView[i],1000,1));
           
       }
       //plotObjects("Maps/wideCV.png",newCView,cView);
       cView = newCView;
       //waitHere();
    Surface tempSurf;
    //tempSurf = Surface(PointXY(0,0),PointXY(1000,1000));
    tempSurf = Surface(PointXY(0,0),PointXY(cView[0].X1(),cView[0].Y1()));//,1);
    polygon.push_back(tempSurf);//for 0,0 to p1 of first object
    vector<double> tempPoint;
    for (unsigned int i = 0; i < cView.size() - 1; i++) {
        if (checkForIntersection(cView[i], cView[i + 1]) == 1) {
            tempPoint = getIntersectionPoint(cView[i], cView[i + 1]);
            tempSurf = Surface(PointXY(cView[i].X1(), cView[i].Y1()), PointXY(tempPoint[0],tempPoint[1])); //,(i*2)+2);
            polygon.push_back(tempSurf);
            //tempSurf = Surface(PointXY(tempPoint[0],tempPoint[1]), PointXY(cView[i + 1].X2(), cView[i + 1].Y2())); //,(i*2)+3)
            //polygon.push_back(tempSurf);
            cView[i+1].setP1(tempPoint[0],tempPoint[1]);
        } else {
            tempSurf = Surface(PointXY(cView[i].X1(), cView[i].Y1()), PointXY(cView[i].X2(), cView[i].Y2())); //,(i*2)+2);
            polygon.push_back(tempSurf);
            tempSurf = Surface(PointXY(cView[i].X2(), cView[i].Y2()), PointXY(cView[i + 1].X1(), cView[i + 1].Y1())); //,(i*2)+3)
            polygon.push_back(tempSurf);
        }
    }
    tempSurf = Surface(PointXY(cView[cView.size()-1].X1(),cView[cView.size()-1].Y1()),
                                                 PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()));//,(cView[cView.size()-1]*2)+2);
    polygon.push_back(tempSurf);//for last surface
    tempSurf = Surface(PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()),
                                                 PointXY(0,0));//,(cView[cView.size()-1]*2)+3);
    polygon.push_back(tempSurf);//for last surface to 0,0
    
   
    
    return polygon;
    
}

vector<Surface> constructPolygon(vector<Object> cView) {
    cout<<"Constructing Polygon!!!"<<endl;
    vector<Object> originToPoints;
    Object originToPoint;
    
    
    originToPoint.set(0,0,cView[0].X1(),cView[0].Y1(),1);//for first point
    originToPoints.push_back(originToPoint);//for first point
    for(unsigned int i=0; i<cView.size()-1; i++) {
        if(cView[i].distP2ToP1(cView[i+1]) < 200) {//for corner points
            originToPoint.set(0,0,cView[i+1].X1(),cView[i+1].Y1(),1);
            originToPoints.push_back(originToPoint);
        } else {//for other points
            originToPoint.set(0,0,cView[i].X2(),cView[i].Y2(),1);
            originToPoints.push_back(originToPoint);
            originToPoint.set(0,0,cView[i+1].X1(),cView[i+1].Y1(),1);
            originToPoints.push_back(originToPoint);
        }
    }
    originToPoint.set(0,0,cView[cView.size()-1].X2(),cView[cView.size()-1].Y2(),1);//for last point
    originToPoints.push_back(originToPoint);//for last point
    cout<<"Total number of points: "<<originToPoints.size()<<endl;
    
    
    
    //to debug the above code
//    vector<Object> temps;
//    for(unsigned int i=0; i<originToPoints.size(); i++) {
//        temps.push_back(originToPoints[i]);
//        plotObjects("Maps/polygonGrowing.png",cView,temps);
//        cout<<"Polygon growing "<<endl;
//        waitHere();
//    }
    
    //extending originToPoints lines
    vector<Object> temps;
    for(unsigned int i=0; i<originToPoints.size(); i++) {
        temps.push_back(makeLineAtPointWithObject(0.0,0.0,originToPoints[i].length()+300,originToPoints[i]));
//        plotObjects("Maps/polygonGrowing.png",cView,temps);
//        cout<<"Polygon growing "<<endl;
//        waitHere();
    }
    
    //making final polygon
    vector<Surface> polygon;
    Surface tempSurf;
    tempSurf = Surface(PointXY(0.0,0.0),PointXY(temps[0].X2(),temps[0].Y2()));//,1);
    polygon.push_back(tempSurf);
    for(unsigned int i=0; i<temps.size()-1; i++) {
        tempSurf = Surface(PointXY(temps[i].X2(),temps[i].Y2()),PointXY(temps[i+1].X2(),temps[i+1].Y2()));//,1);
        polygon.push_back(tempSurf);
    }
    tempSurf = Surface(PointXY(temps[temps.size()-1].X2(),temps[temps.size()-1].Y2()),PointXY(0.0,0.0));//,1);
    polygon.push_back(tempSurf);
    
    return polygon;
}

//it computes a exact boundary from cView 
//added on 26 Nov 2013
vector<Surface> findExactBoudaryFrom(const vector<Object>& cView, const Object & rPosition) {
       vector<Surface> polygon;

    Surface tempSurf;
    tempSurf = Surface(PointXY(rPosition.X1(),rPosition.Y1()),PointXY(cView[0].X1(),cView[0].Y1()));//,1);
    polygon.push_back(tempSurf);//for 0,0 to p1 of first object
    for(unsigned int i=0;i<cView.size()-1;i++) {
        tempSurf = Surface(PointXY(cView[i].X1(),cView[i].Y1()),PointXY(cView[i].X2(),cView[i].Y2()));//,(i*2)+2);
        polygon.push_back(tempSurf);
        tempSurf = Surface(PointXY(cView[i].X2(),cView[i].Y2()),PointXY(cView[i+1].X1(),cView[i+1].Y1()));//,(i*2)+3)
        polygon.push_back(tempSurf);
    }
    tempSurf = Surface(PointXY(cView[cView.size()-1].X1(),cView[cView.size()-1].Y1()),
                                                 PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()));//,(cView[cView.size()-1]*2)+2);
    polygon.push_back(tempSurf);//for last surface
    tempSurf = Surface(PointXY(cView[cView.size()-1].X2(),cView[cView.size()-1].Y2()),
                                                 PointXY(rPosition.X1(),rPosition.Y1()));//,(cView[cView.size()-1]*2)+3);
    polygon.push_back(tempSurf);//for last surface to 0,0  
   
    
    return polygon;
    
}

//it actually makes all the objects are dotted which have even number index value.
vector<Object> makeGapsAreDotted(const vector<Object>& objs) {
    vector<Object> result;
    
    vector<Object> temp;
    
    for(unsigned int i=0; i<objs.size(); i++) {
        if(i % 2 == 0) {
            temp = breakTheLineInto(objs[i], 300);
            for(unsigned int j=0; j<temp.size(); j++) {
                result.push_back(temp[j]);
            }
        } else {
            result.push_back(objs[i]);
        }
    }
    
    return result;
}

bool sortIntAscendingOrder(int a, int b) {
    return a < b;
}

bool isThisSurfaceAcrossThePolygon(vector<Object> polygonObjects, Object thisObject) {
    int numberOfInterSect = 0;
    for(unsigned int i=0; i < polygonObjects.size(); i++) {
        if(checkForIntersection(polygonObjects[i],thisObject) == 1) {
            numberOfInterSect++;
        }
        
        if(numberOfInterSect > 1)
            return true;
    }
    
    return false;
}



bool polygonGen(const vector<Object>& ViewA, Point robotPositionA, const vector<Object>& ViewB, Point robotPositionB)
{
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    
    char coordinates_1[10000];
    char coordinates_2[10000];
    
    char para_1[10000];
    char para_2[10000];
    
    double Varea;
    double Iarea;
    
//    vector<Object> surface1;
//    vector<Object> surface2;
    
    polygon poly1,poly2, polyOverlap;
    std::deque<polygon> output;
    
     sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotPositionA.X(), " ", robotPositionA.Y(), ",");   
    
    //read the coordinates of each points, 
    for ( int i = 0; i < ViewA.size(); i++) 
    {
        sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");   
        sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
        if (i != ViewA.size() - 1) 
        {
            sprintf(coordinates_1, "%s%s", coordinates_1, ",");
        }
        else
        {
            //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
            sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotPositionA.X(), " ", robotPositionA.Y());
        }

    }
    
     
    sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotPositionB.X(), " ", robotPositionB.Y(), ",");
    for ( int i = 0; i < ViewB.size(); i++) 
    {
        sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
        sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());
        
        if (i != ViewB.size() - 1) 
        {
            sprintf(coordinates_2, "%s%s", coordinates_2, ",");
        }
        else
        {
            //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
            sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotPositionB.X(), " ", robotPositionB.Y());
        }

    }
    
    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");
    
    // Calculate the area of a cartesian polygon
    
    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    

    
    bg::intersection(poly1, poly2, output);
    
    BOOST_FOREACH(polyOverlap, output)
    {
        Iarea = bg::area(polyOverlap) / 100000;
        std::cout <<"Intersection area:" << Iarea << std::endl;
    }
    
    Varea = bg::area(poly2) / 100000;
    std::cout << "View 'B' area:" << Varea <<std::endl;


    double percent = Iarea / Varea * 100;
    
    char ImgFileName1[80];
    char ImgFileName2[80];
    char ImgFileName3[80];
        sprintf(ImgFileName1, "%s", "Maps/intersectionmap1.svg");
    sprintf(ImgFileName2, "%s", "Maps/intersectionmap2.svg");
    sprintf(ImgFileName3, "%s", "Maps/intersectionmap3.svg");
    std::ofstream svg1(ImgFileName1);
    std::ofstream svg2(ImgFileName2);
    std::ofstream svg3(ImgFileName3);

    boost::geometry::svg_mapper<point_type> mapper1(svg1, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper2(svg2, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper3(svg3, 2000, 2000);
    // Add geometries such that all these geometries fit on the map


    mapper1.add(poly1);

    mapper2.add(poly1);
    mapper2.add(poly2);

    mapper3.add(poly1);
    mapper3.add(poly2);
    mapper3.add(polyOverlap);


    mapper1.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(polyOverlap, "fill-opacity:0.8;fill:rgb(0,255,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    
//    drawTwoOverlappingPics(ViewA,robotPositionA,ViewB,robotPositionB);
    
    
    if(percent >= 50)
        return true;
    else
        return false;
}


