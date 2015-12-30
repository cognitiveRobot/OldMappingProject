/*		Matching Information Class
			by mhossain
			
*/

#include <iostream>

#include <cmath>
#include <vector>
#include "Minfo.H"
using namespace std;

/**********  ref lines information Class ************/
Minfo::Minfo (int a, int b, double dis, int p)
{
	id1=a; id2=b; 
	dist=dis;
	RP=p;
}

int Minfo::getPID(){ return id1;}
int Minfo::getCID(){ return id2;}

void Minfo::setPID(int a) {
	id1=a;
}

void Minfo::setCID(int a) {
	id2=a;
}
double Minfo::getDist()
{
	return dist;
}
void Minfo::setDist(double dis)
{
	dist=dis;
}
int Minfo::getRP()
{
	return RP;
}
void Minfo::setRP(int p)
{
	RP=p;
}

void Minfo::setAngleDiff(double a) {
    angleDiff = a;
}
        double Minfo::getAngleDiff() {
            return angleDiff;
        }
void Minfo::setDistDiff(double a) {
    distDiff = a;
}
        double Minfo::getDistDiff() {
            return distDiff;
        }
        
        void Minfo::setLength(double a) {
            length = a;
        }
        double Minfo::getLength() {
            return length;
        }
        
        void Minfo::setGoodnessValue(double a) {
            goodnessValue = a;
        }
        double Minfo::getGoodnessValue() {
            return goodnessValue;
        }
        
        void Minfo::setDistFromMean(double a) {
            distFromMean = a;
        }
        double Minfo::getDistFromMean() {
            return distFromMean;
        }
        void Minfo::setDistFromRobot(double a) {
            distFromRobot = a;
        }
        double Minfo::getDistFromRobot() {
            return distFromRobot;
        }
        
void Minfo::set(int a, int b, double dis)
{
	id1=a; id2=b;
	dist=dis;
}


void Minfo::display()
{
	cout<<"Minfo id1: "<<id1<<", id2: "<<id2<<" fitness- "<<dist<<" distDiff "<<distDiff<<" angleDiff "<<angleDiff<<" length "<<length<<" DistFMean "<<distFromMean<<" ref point: "<<RP<<endl;
}

void displayMinfo(vector<Minfo> mi)
{
	for(int i=0;i<int(mi.size());i++)
		mi[i].display();
}

//last one max
bool DataSortMethod(Minfo p1, Minfo p2) {
  return p1.getDist() < p2.getDist();
}

bool SortBasedOnGoodnessValue(Minfo p1, Minfo p2) {
  return p1.getGoodnessValue() < p2.getGoodnessValue();
}

bool SortBasedOnLength(Minfo p1, Minfo p2) {
    return p1.getLength() < p2.getLength();
}

bool sortAsCID(Minfo p1, Minfo p2) {
  return p1.getCID() < p2.getCID();
}

bool SortBasedOnDistFromRobot(Minfo p1, Minfo p2) {
    return p1.getDistFromRobot() < p2.getDistFromRobot();
}

//last one min
bool SortBasedOnAngleDiff(Minfo p1, Minfo p2) {
    return p1.getAngleDiff() > p2.getAngleDiff();
}

bool SortBasedOnDistDiff(Minfo p1, Minfo p2) {
    return p1.getDistDiff() < p2.getDistDiff();
}

bool SortBasedOnDistFromMean(Minfo p1, Minfo p2) {
    return p1.getDistFromMean() > p2.getDistFromMean();
}





/*************** Reference Point Class ************/
RefPoint::RefPoint(double a, double b, double c, double d, double e) {
	x1=a;	y1=b;
	x2=c;	y2=d;
	edist=e;
}

void RefPoint::display() {
	cout<<" Ref Point: "<<x1<<" "<<y1<<" ::: "<<x2<<" "<<y2<<" error: "<<edist<<endl;
}

void RefPoint::set(double a, double b, double c, double d, double e) {
	x1=a;	y1=b;
	x2=c;	y2=d;
	edist=e;
}

double RefPoint::getErrorDist() {
	return edist;
}

double RefPoint::mX() {return x1;}
double RefPoint::mY() {return y1;}
double RefPoint::cX() {return x2;}
double RefPoint::cY() {return y2;}

void displayRefPoints(vector<RefPoint> refpoints) {
	for(int i=0;i<int(refpoints.size());i++) 
		refpoints[i].display();
}

bool DataSort(RefPoint p1, RefPoint p2) {
  return p1.getErrorDist() < p2.getErrorDist();
}














MI2ID::MI2ID (int a, int b, int c, int d)
{
	gid1=a; id1=b; 
	gid2=c; id2=d;
}

int MI2ID::getPGID(){ return gid1;}
int MI2ID::getPID(){ return id1;}
int MI2ID::getCGID(){ return gid2;}
int MI2ID::getCID(){ return id2;}

void MI2ID::set(int a, int b, int c, int d)
{
	gid1=a; id1=b; 
	gid2=c; id2=d;
}
void MI2ID::set(int a, int b)
{
	gid1=a; id1=b;
}
void MI2ID::display()
{
	cout<<"p gid: "<<gid1<<" id: "<<id1<<", c gid: "<<gid2<<" id "<<id2<<endl;
}

void displayMI2ID(vector<MI2ID> mi)
{
	for(int i=0;i<int(mi.size());i++)
		mi[i].display();
}
