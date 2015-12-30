/* 
 * Match Information class
 * 			mhossain
 *			08 Sept. 2010
 */

#ifndef _MINFO_H
#define	_MINFO_H

#include <vector>

using namespace std;

class Minfo 
{
private:
	int id1;
	int id2;
	double dist; //dist from matched point to intersection point//used to store fitness
	int RP; //reference point
        double angleDiff;
        double distDiff;
        double length;
        double distFromMean;
        double distFromRobot;
        double goodnessValue;
public:	
	Minfo(){};
	Minfo(int a, int b, double dis, int p);
	~Minfo(){};

	int getPID();
	int getCID();
	void setPID(int a);
	void setCID(int a);
	double getDist();
	void setDist(double dis);
	int getRP();
	void setRP(int p);
	void set(int a, int b, double dis);
	void display();
        
        void setAngleDiff(double a);
        double getAngleDiff();
        
        void setDistDiff(double a);
        double getDistDiff();
        
        void setLength(double a);
        double getLength();
        
        void setGoodnessValue(double a);
        double getGoodnessValue();
        
        void setDistFromMean(double a);
        double getDistFromMean();
        
        void setDistFromRobot(double a);
        double getDistFromRobot();

};
void displayMinfo(vector<Minfo> mi);
bool DataSortMethod(Minfo p1, Minfo p2);
bool SortBasedOnGoodnessValue(Minfo p1, Minfo p2);
bool sortAsCID(Minfo p1, Minfo p2);
bool SortBasedOnAngleDiff(Minfo p1, Minfo p2);
bool SortBasedOnLength(Minfo p1, Minfo p2);
bool SortBasedOnDistDiff(Minfo p1, Minfo p2);
bool SortBasedOnDistFromMean(Minfo p1, Minfo p2);
bool SortBasedOnDistFromRobot(Minfo p1, Minfo p2);

/*
class RPoints
{
private:
	double x1;
	double y1;
	double x2;
	double y2;
	double dist;
public:
	RPoints(){};
	RPoints(double a, double b, double c, double d, double e);
	~RPoints(){};

	void display();
	void set(double a, double b, double c, double d, double e);
	double getDistFromRobot();

	double X1();
	double Y1();
	double X2();
	double Y2();
};
void displayRefPoints(vector<RefPoint> refpoints);
*/



//p1 is of MFIS n p2 is of cv
class RefPoint
{
private:
	double x1;
	double y1;
	double x2;
	double y2;
	double edist;
public:
	RefPoint(){};
	RefPoint(double a, double b, double c, double d, double e);
	~RefPoint(){};

	void display();
	void set(double a, double b, double c, double d, double e);
	double getErrorDist();

	double mX();
	double mY();
	double cX();
	double cY();
};
void displayRefPoints(vector<RefPoint> refpoints);
bool DataSort(RefPoint p1, RefPoint p2);

class MI2ID
{
private:
	int gid1;
	int id1;
	int gid2;
	int id2;
public:	
	MI2ID(){};
	MI2ID(int a, int b, int c, int d);
	~MI2ID(){};

	int getPID();
	int getPGID();
	int getCID();
	int getCGID();
	void set(int a, int b, int c, int d);
	void set(int a, int b);
	void display();

};
void displayMI2ID(vector<MI2ID> mi);
#endif
