
#ifndef _PATHPLANNING_H_
#define _PATHPLANNING_H_

#include <cmath>
#include <iostream>
#include <vector>
#include "Object.H"
#include "asr.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"


//#define M_PI 3.14159265358979323846

class Exit
{
private:
	double x1;
	double y1;
	double x2;
	double y2;
	int id; //exit id
	int p1id; //id of object containg first vertex of exit
	int p2id;//id of object containg second vertex of exit
	double angle; //angle btw mp and first point of exit direction(clearness)
	double p2dist;//dist btw p2 and robot position
public:	
	Exit(){};
	Exit(double a, double b,double c, double d);
	~Exit(){};

	double X1();
	double Y1();
	double X2();
	double Y2();

	void setID(int a);
	int getID();
	void set(double a, double b, double c, double d);
	void set(Object tmp);
	void set(Exit e);
	void display();
	void setP1ID(int a);
	int getP1ID();
	void setP2ID(int a);
	int getP2ID();
	void setP2DistFromRobot(double a);
	double getP2DistFromRobot();
	double length();
	double distP1ToPoint(double a, double b);
	double distP2ToPoint(double a, double b);
	double mpX();
	double mpY();
	void setAngle(double a);
	double getAngle();
                  double midToPoint(double a, double b);
	
};
void displayExits(vector<Exit> exits);
bool sortExitsA2L(Exit d1, Exit d2);//sort exits according to length
bool sortExitsA2A(Exit e1, Exit e2); //sort exits according to angle tag
bool sortDistA2V(double a, double b);//

vector<double> exitsFromorign(vector<Exit> exitsFromCurrent);


class Destination {
private:
    double angle;
    double dist;
    int type; //1 means normal 2 means obs avoidance
    int attemptsToAvoidObstacles; //Num of trying done to avoid obstacle
    vector<Object> destinationExits;
    vector<pair<double,double> > otherExitLocations;
public:

    Destination() : attemptsToAvoidObstacles(0), angle(0), dist(0), destinationExits(0) {
    };
    Destination(double a, double b);

    ~Destination() {
    };

    void setAngle(double a);
    double getAngle();
    void setDist(double a);
    double getDist();
    void setType(int a);
    int getType();
    void display();
    
    void oneAttemptToAO();
    int getAttemptsToAO();
    void setAttemptsToAO(int attempts);
    void setDestinationExits(vector<Object> destExits);
    vector<Object> getDestinationExits();
    
    vector<pair<double,double> > getOtherExitLocations();
    

    void findNextDestination(vector<Object> currentView, vector<Object> referenceObjects, int viewNumber);
    void findNextStepLocation(vector<Object> currentView, vector<Object> referenceObjects, pair<double,double> goal, int viewNumber);
};


vector<Exit> findExits(vector<Object> cv);
vector<Exit> findShortestExits(vector<Object> cv);//modified version of findExits(ie modified version of yeap's theory)
vector<Object> findDestinationExits(vector<Object> cv, vector<Object> refObjects);//modified version of shortest Exits
vector<Object> findAllDestinationExits(vector<Object> cv, vector<Object> refObjects);//modified version of findDestinationExits
vector<Object> tranformCVExitsInMFIS(vector<Object> cvExits, vector<Object> refObjects);
vector<Exit> findGateways(vector<Object> cv);//hossain's gateway theorem
vector<Exit> findGatewaysNew(vector<Object> cv);
Destination findDestinationPoint(vector<Object> cv, vector<Exit> exits);
Destination findDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides);
Destination findNextDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides);
Destination findDestToMaxPath(vector<Object> cv, vector<Object> robsides, int viewno);
vector<double> findMovementDirection(vector<Object> pexits);//obsolete
vector<Object> exitsInMFIS(vector<Exit> exits,Object rmfis, Object rcv,int refpoint);
vector<Object> convertExitToObject(vector<Exit> exits);//convert exit to object type

//developed on Tue Oct 11, 2011
vector<Exit> findNewASRFormingExits(vector<Object> cv);

Destination avoidObstacle(double angleToDest,double distanceToDest, vector<Object> currentView, int attempts);
vector<pair<double, double> > findInvalidatedGoals(vector<Object> currentView, int goalType);
//Destination verifyDestination(vector<pair<double, double> > invalidatedGoals, vector<Object> currentView);
Destination verifyDestination(vector<pair<double, double> > invalidatedExitGoals, vector<pair<double, double> > invalidatedGoals, vector<Object> currentView);
Destination avoidObstacleUsingPathPolygon(double angleToDest,double distanceToDest,vector<Object> currentView);

vector<Object> findExitToReachGoal(vector<Object> currentView,pair<double,double> goal,vector<Object> referenceObjects, int viewNumber);


#endif

vector<Object> findGapsForGoalExit(vector<Object> currentView,vector<Object> referenceObjects, Object goalExit,int viewNumber);
pair<double,double> findCurrentGoal(vector<Object> goalGap);
bool isThePathClear(vector<Object> currentView, Object goalExit);
Object recognizeGoalExit(vector<Object> exits, Object goalExit);

vector<Exit> findShortestGap(vector<Object> cv);
vector<Point> findPathToReachGoal(vector<ASR> places) ;

Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno);
Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno);
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno,PointXY goal);
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno,PointXY goal);


vector<Exit> findGapasExits(vector<Object> cv);  // find suitable exits

//vector<Object> BoundaryByExits(vector<Object> CurrentView); // find exit and rebuild this view and return it



