
#ifndef GEOMETRICOP_H
#define GEOMETRICOP_H

#include <iostream>
#include <vector>
#include "Point.H"
#include "Object.H"

using namespace std;


vector<Object> xformPVIntoCV(vector<Object> pview, Point rpose, double angle);
vector<Point> xformPointsIntoCV(vector<Point> pview, Point rpose, double angle);
vector<Object> xformPObjectsIntoCV(vector<Object> pview, Point rpose, double angle);
vector<Object> xformPObjectsIntoCVForVision(vector<Object> pview, Point rpose, double angle);
vector<Object> discardLinesIFoR(vector<Object> pview, Point rpose, double angle, vector<Object> cview, Object rmfis,Object rcv,int rp);
//vector<RfPoint> xformRefPointsIntoCV(vector<RfPoint> pview, Point rpose, double angle);
vector<Object> xformCVIntoPV(vector<Object> cview, Point rpose, double angle);
double checkForIntersection (Object s1, Object s2);
vector<double> getIntersectionPoint(Object s1, Object s2);
double getDistBtw2Points(double x1, double y1, double x2, double y2);
double angleObjectAndPoint(Object s, double x3, double y3);
double angleObjectAndXaxis(Object s);
vector<Point> p2cCoordinate(vector<double> vec); //for robot only
double getX(double angle,double dist);
double getY(double angle,double dist);
bool pointInPolygon(Object a, Object b, double x, double y);
vector<Object> breakTheLineInto(Object smfis, int gapBetweenDots = 200);
vector<Object> breakTheLinesInto(vector<Object> allLines);
Object makeLineAtPointWithObject(double angle, double dist, Object smfis);
Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis);
Object makeLineAtPointWithObject(double angle, double dist, double length, Object smfis, int a);
vector<Object> makeSquareAtLoSPoints(vector<Object> lineOfSitePoints);
Object makeLineAtTwoPointsWithObject(double angle1, double dist1, double angle2, double dist2, Object smfis, int referencePoint);
void waitHere();

vector<Object> makeSquare(Object oneSide);
vector<Object> makeAllSquare(vector<Object> allLines);

vector<Object> makeRectangle(Object oneSide);
vector<Object> makeAllRectangle(vector<Object> allLines);

vector<Object> makeArrow(Object oneSide);

Object makeParallelObject(Object oneSide, double dist,const char side);
Object makeParallelObject(Object oneSide, double dist, int side);

vector<Surface> makePolygonOfCV(vector<Object> cView);

vector<Surface> constructPolygon(vector<Object> cView);

vector<Surface> findExactBoudaryFrom(const vector<Object>& cView, const Object & rPosition);
vector<Object> makeGapsAreDotted(const vector<Object>& objs);

bool sortIntAscendingOrder(int a, int b);

bool isThisSurfaceAcrossThePolygon(vector<Object> polygonObjects, Object thisObject);

bool polygonGen(const vector<Object>& ViewA, Point robotPositionA, const vector<Object>& ViewB, Point robotPositionB);

#endif