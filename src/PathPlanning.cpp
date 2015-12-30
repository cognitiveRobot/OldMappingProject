#include <iostream>
#include <vector>
#include <algorithm>
//#include <mrpt/base/include/Eigen/src/plugins/BlockMethods.h>  // Needed for sort() method
#include "PathPlanning.H"
#include "Object.H"
#include "GeometricOp.H"
#include "mfisOp.H"
#include "Plotting.H"
#include "CompareASR.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"

#define PI 3.14159265

Exit::Exit(double a, double b,double c, double d) {
	x1=a;	y1=b;
	x2=c;	y2=d;
}
double Exit::X1(){return x1;}
double Exit::Y1(){return y1;}
double Exit::X2(){return x2;}
double Exit::Y2(){return y2;}

void Exit::setID(int a) {
	id=a;
}
int Exit::getID() {
	return id;
}
void Exit::set(double a, double b, double c, double d) {
	x1=a;	y1=b;
	x2=c;	y2=d;
}
void Exit::set(Exit e) {
	x1=e.X1();	y1=e.Y1();
	x2=e.X2();	y2=e.Y2();
	id=e.getID();
	p1id=e.getP1ID();	p2id=e.getP2ID();
	angle=e.getAngle();
	p2dist=e.getP2DistFromRobot();
}
void Exit::set(Object tmp) {
	x1=tmp.X1();	y1=tmp.Y1();
	x2=tmp.X2();	y2=tmp.Y2();
}
void Exit::display() {
	cout<<"ID: "<<id<<" X1: "<<x1<<" Y1: "<<y1<<" ------ X2: "<<x2<<" Y2: "<<y2<<" p1id: "<<p1id<<" p2id: "<<p2id<<" p2dist "<<p2dist<<" length "<<length()<<" angle: "<<angle<<endl;
}
void Exit::setP1ID(int a) {
	p1id=a;
}
int Exit::getP1ID() {
	return p1id;
}
void Exit::setP2ID(int a) {
	p2id=a;
}
int Exit::getP2ID() {
	return p2id;
}
void Exit::setP2DistFromRobot(double a) {
	p2dist=a;
}
double Exit::getP2DistFromRobot() {
	return p2dist;
}

double Exit::length() {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double Exit::distP1ToPoint(double a, double b) {
	return sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));
}

double Exit::distP2ToPoint(double a, double b) {
	return sqrt((x2-a)*(x2-a)+(y2-b)*(y2-b));
}

double Exit::mpX() {
	return (x2+x1)/2;
}
double Exit::mpY() {
	return (y2+y1)/2;
}
void Exit::setAngle(double a) {
	angle=a;
}
double Exit::getAngle() {
	return angle;
}

double Exit::midToPoint(double a, double b)
{
        return sqrt(((x2+x1)/2-a)*((x2+x1)/2-a)+((y2+y1)/2-b)*((y2+y1)/2-b));
}

vector<double> exitsFromorign(vector<Exit> exitsFromCurrent)
{
    vector<double> distancesToexits;
    
    for(int i = 0; i < exitsFromCurrent.size(); i++)
        distancesToexits.push_back(exitsFromCurrent[i].midToPoint(0,0));

    
    return distancesToexits;
}

void displayExits(vector<Exit> exits) {
	//cout<<"no of exits "<<exits.size()<<endl;
	for(int i=0;i<int(exits.size());i++) {
		exits[i].display();
	}
}
bool sortExitsA2L(Exit d1, Exit d2)
{
  return d1.length() > d2.length();
}
bool sortExitsA2A(Exit e1, Exit e2) {
	return e1.getAngle() > e2.getAngle();
}

bool sortDistA2V(double a, double b) {
	return a < b;
}

//****************************************Destination class
Destination::Destination(double a, double b) {
	angle = a;
	dist = b;
}

void Destination::setAngle(double a) {
	angle = a;
}
double Destination::getAngle() {
	return angle;
}
void Destination::setDist(double a) {
	dist=a;
}
double Destination::getDist() {
	return dist;
}
void Destination::setType(int a) {
	type=a;
}
int Destination::getType() {
	return type;
}
void Destination::display() {
	cout<<"Angle: "<<angle<<" Dist: "<<dist<<" type: "<<type<<endl;
}

void Destination::oneAttemptToAO() {
    attemptsToAvoidObstacles++;
}
//it returns how many attempts taken to avoid obstacles 
int Destination::getAttemptsToAO() {
    return attemptsToAvoidObstacles;
}
void Destination::setAttemptsToAO(int attempts) {
    attemptsToAvoidObstacles = attempts;
}

void Destination::setDestinationExits(vector<Object> destExits) {
    destinationExits = destExits;
}
vector<Object> Destination::getDestinationExits() {
    return destinationExits;
}
vector<pair<double,double> > Destination::getOtherExitLocations() {
    return otherExitLocations;
}

void Destination::findNextDestination(vector<Object> currentView, vector<Object> referenceObjects, int viewNumber) {
    cout << "\n\033[1;34m               Looking for NextDestination             \033[0m" << endl << endl;

    //calculating direction to max path
    
    Object robotPose(0, 0, 0, 500, 1);   
    char viewFileName[100];
    //finding exit goals
    vector<pair<double,double> > invalidatedExitGoals;
//    cout<<"destination exits: "<<destinationExits.size()<<endl;
//    if(destinationExits.size() == 0 && viewNumber > 15)
    vector<Object> destinationExitsInCV = findDestinationExits(currentView,referenceObjects);
   
//    cout<<"Destination Exits in CV: "<<destinationExitsInCV.size()<<endl;
    if(destinationExitsInCV.size() > 0) {
        cout<<"Destination Exits found "<<destinationExitsInCV.size()<<endl;
        displayObjects(destinationExitsInCV);
//        currentView[10].display();
//        vector<Object> destinationExitsInCV;
//        for(unsigned int i =0;i<destinationExits.size();i++) {
//            destinationExitsInCV.push_back(remakeLineP2(referenceObjects[1],referenceObjects[0],destinationExits[i],1,0,referenceObjects[0].getKP()));
//        }
       
        invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV,1);//1 bcz these are exit goals
        destinationExitsInCV.push_back(robotPose);
        sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/DestinationExits-", viewNumber, ".png");
        plotObjects(viewFileName,destinationExitsInCV,currentView);
    }
//     if(viewNumber > 12) {
//    vector<Exit> yeapsExits = findExits(currentView);
//    plotObjects("MFIS/yeapsExits.png",convertExitToObject(yeapsExits),currentView);
////    waitHere();
//    }
//    if(viewNumber > 15)
//    waitHere();
    
//    cout<<"Invalidated Goal Angle: "<<angleToMaxPath<<" distance: "<<distanceAlongMaxPath<<endl;  
    
    vector<pair<double,double> > invalidatedGoals = findInvalidatedGoals(currentView,2);//2 bcz these are farthest path goals
    
    for(unsigned int i =0;i<invalidatedGoals.size();i++){
        cout<<"(From NewFunction) Angle: "<<invalidatedGoals[i].first<<" distance: "<<invalidatedGoals[i].second<<endl;
    }
//    waitHere();
    
    //only for printing
    
    double angleToMaxPath, distanceAlongMaxPath;
    if(invalidatedExitGoals.size() > 0) {
        angleToMaxPath = invalidatedExitGoals[0].first;
        distanceAlongMaxPath = invalidatedExitGoals[0].second;
    } else {
        angleToMaxPath = invalidatedGoals[0].first;
        distanceAlongMaxPath = invalidatedGoals[0].second;
    }
    
    PointXY tentativeGoal(getX(angleToMaxPath,distanceAlongMaxPath),getY(angleToMaxPath,distanceAlongMaxPath));
    Object tentativeRobotPose(tentativeGoal.getX(),tentativeGoal.getY(),getX(angleToMaxPath,distanceAlongMaxPath+500),getY(angleToMaxPath,distanceAlongMaxPath+500));
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToMaxPath,distanceAlongMaxPath); 
    
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/BeforeAvoiding-", viewNumber, ".png");
    if(invalidatedExitGoals.size() > 0)
        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
//    if(viewNumber == 42)
    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    
//    cout<<"Test Angle: "<<robotPose.getAngleWithLine(Object(0,0,currentView.back().X1(),currentView.back().Y1(),1))<<endl;


    Destination verifiedDestination;
//    verifiedDestination = avoidObstacle(invalidatedGoals,currentView,1);
    verifiedDestination = verifyDestination(invalidatedExitGoals,invalidatedGoals,currentView);
    angle = verifiedDestination.getAngle();
    dist = verifiedDestination.getDist();      
    //storing all exits for future 
    for(unsigned int i = 0; i<invalidatedExitGoals.size();i++) {
        otherExitLocations.push_back(invalidatedExitGoals[i]);
    }
    cout<<"Verified Destination: "<<angle<<" dist "<<dist<<endl;
   
    
    //just for printing
    PointXY newGoal(getX(angle,dist),getY(angle,dist));
    tentativeRobotPose.set(newGoal.getX(),newGoal.getY(),getX(angle,dist+500),getY(angle,dist+500),1);
//    tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    robotWholePath = myrobot.robotPathForNextDest(angle,dist);  
    if(invalidatedExitGoals.size() > 0)
        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/AfterAvoiding-", viewNumber,".png");
//    if(viewNumber == 42)
    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
}

//created on 03-07-12
void Destination::findNextStepLocation(vector<Object> currentView, vector<Object> referenceObjects, pair<double,double> goal, int viewNumber) {
    cout << "\n\033[1;34m               Looking for NextDestination             \033[0m" << endl << endl;

    //calculating direction to max path
    
    Object robotPose(0, 0, 0, 500, 1);   
    char viewFileName[100];
    //finding exit goals
    vector<pair<double,double> > invalidatedExitGoals, invalidatedGoals;
//    cout<<"destination exits: "<<destinationExits.size()<<endl;
//    if(destinationExits.size() == 0 && viewNumber > 15)
//    vector<Object> destinationExitsInCV = findDestinationExits(currentView,referenceObjects);
//   
////    cout<<"Destination Exits in CV: "<<destinationExitsInCV.size()<<endl;
//    if(destinationExitsInCV.size() > 0) {
//        cout<<"Destination Exits found "<<destinationExitsInCV.size()<<endl;
//        displayObjects(destinationExitsInCV);
////        currentView[10].display();
////        vector<Object> destinationExitsInCV;
////        for(unsigned int i =0;i<destinationExits.size();i++) {
////            destinationExitsInCV.push_back(remakeLineP2(referenceObjects[1],referenceObjects[0],destinationExits[i],1,0,referenceObjects[0].getKP()));
////        }
//       
//        invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV,1);//1 bcz these are exit goals
//        destinationExitsInCV.push_back(robotPose);
//        sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/DestinationExits-", viewNumber, ".png");
////        plotObjects(viewFileName,destinationExitsInCV,currentView);
//    }
//     if(viewNumber > 12) {
//    vector<Exit> yeapsExits = findExits(currentView);
//    plotObjects("MFIS/yeapsExits.png",convertExitToObject(yeapsExits),currentView);
////    waitHere();
//    }
//    if(viewNumber > 15)
//    waitHere();
    
//    cout<<"Invalidated Goal Angle: "<<angleToMaxPath<<" distance: "<<distanceAlongMaxPath<<endl;  
    
//    vector<pair<double,double> > invalidatedGoals = findInvalidatedGoals(currentView,2);//2 bcz these are farthest path goals
//    
//    for(unsigned int i =0;i<invalidatedGoals.size();i++){
//        cout<<"(From NewFunction) Angle: "<<invalidatedGoals[i].first<<" distance: "<<invalidatedGoals[i].second<<endl;
//    }
//    waitHere();
    
    //only for printing
   
    invalidatedExitGoals.push_back(goal);
    double angleToMaxPath, distanceAlongMaxPath;
    if(invalidatedExitGoals.size() > 0) {
        angleToMaxPath = invalidatedExitGoals[0].first;
        distanceAlongMaxPath = invalidatedExitGoals[0].second;
    } else {
        angleToMaxPath = invalidatedGoals[0].first;
        distanceAlongMaxPath = invalidatedGoals[0].second;
    }
    
    PointXY tentativeGoal(getX(angleToMaxPath,distanceAlongMaxPath),getY(angleToMaxPath,distanceAlongMaxPath));
    Object tentativeRobotPose(tentativeGoal.getX(),tentativeGoal.getY(),getX(angleToMaxPath,distanceAlongMaxPath+500),getY(angleToMaxPath,distanceAlongMaxPath+500));
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToMaxPath,distanceAlongMaxPath); 
    
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/BeforeAvoiding-", viewNumber, ".png");
//    if(invalidatedExitGoals.size() > 0)
//        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
//    if(viewNumber == 42)
//    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    plotObjects(viewFileName,robotWholePath,currentView);
    
//    cout<<"Test Angle: "<<robotPose.getAngleWithLine(Object(0,0,currentView.back().X1(),currentView.back().Y1(),1))<<endl;


    Destination verifiedDestination;
//    verifiedDestination = avoidObstacle(invalidatedGoals,currentView,1);
    verifiedDestination = verifyDestination(invalidatedExitGoals,invalidatedGoals,currentView);
    angle = verifiedDestination.getAngle();
    dist = verifiedDestination.getDist();      
    //storing all exits for future 
    for(unsigned int i = 0; i<invalidatedExitGoals.size();i++) {
        otherExitLocations.push_back(invalidatedExitGoals[i]);
    }
    cout<<"Verified Destination: "<<angle<<" dist "<<dist<<endl;
   
    
    //just for printing
    PointXY newGoal(getX(angle,dist),getY(angle,dist));
    tentativeRobotPose.set(newGoal.getX(),newGoal.getY(),getX(angle,dist+500),getY(angle,dist+500),1);
//    tentativeRobot= myrobot.inMFIS(tentativeRobotPose,robotPose,1);
    robotWholePath = myrobot.robotPathForNextDest(angle,dist);  
//    if(invalidatedExitGoals.size() > 0)
//        robotWholePath.push_back(destinationExitsInCV[0]);//exit
    robotWholePath.push_back(robotPose);//current robot pose
    robotWholePath.push_back(tentativeRobotPose);//robot pose at next destination
    sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/AfterAvoiding-", viewNumber,".png");
//    if(viewNumber == 42)
//    plotObjectsOf3Kinds(viewFileName,robotWholePath,destinationExitsInCV,currentView);
    plotObjects(viewFileName,robotWholePath,currentView);
}


Destination verifyDestination(vector<pair<double, double> > invalidatedExitGoals, vector<pair<double, double> > invalidatedGoals, vector<Object> currentView) {
    cout << "\n\033[1;34m               verifying NextDestination             \033[0m" << endl << endl;
    int attempts = 0;
    Destination verifiedDestination;
    double worstCaseAngle = 0;

    if (invalidatedExitGoals.size() > 0) {
//        if (abs(invalidatedExitGoals[0].first) > 60) {//saving the first target for worst case to find free path
//            if (invalidatedExitGoals[0].first > 0)
//                worstCaseAngle = 60;
//            else
//                worstCaseAngle = -60;
//        } else if (abs(invalidatedExitGoals[0].first) < 60 && abs(invalidatedExitGoals[0].first) > 25) {
//            worstCaseAngle = invalidatedExitGoals[0].first;
//        } 
        if(abs(invalidatedExitGoals[0].first) > 25)
            worstCaseAngle = invalidatedExitGoals[0].first;
        else {
            if (invalidatedExitGoals[0].first > 0)
                worstCaseAngle = 25;
            else
                worstCaseAngle = -25;
        }
    }else  if (invalidatedGoals.size() > 0) {
//        if (abs(invalidatedGoals[0].first) > 60) {//saving the first target for worst case to find free path
//            if (invalidatedGoals[0].first > 0)
//                worstCaseAngle = 60;
//            else
//                worstCaseAngle = -60;
//        } else if (abs(invalidatedGoals[0].first) < 60 && abs(invalidatedGoals[0].first) > 25) {
//            worstCaseAngle = invalidatedGoals[0].first;
//        } 
        if (abs(invalidatedGoals[0].first) > 25)
            worstCaseAngle = invalidatedGoals[0].first;
        else {
            if (invalidatedGoals[0].first > 0)
                worstCaseAngle = 25;
            else
                worstCaseAngle = -25;
        }
    }

    if (invalidatedExitGoals.size() > 0) {
        for (unsigned int i = 0; i < invalidatedExitGoals.size(); i++) {
            cout << endl << "(Exit) Going to check invalidated goal no. " << i + 1 << endl;
            do {
                attempts++;
                cout << "Attempts Number " << attempts << endl;
                //        verifiedDestination = avoidObstacle(invalidatedGoals[0].first, invalidatedGoals[0].second, currentView, 1);
                cout << "Invalidated Exit Goal Angle:  " << invalidatedExitGoals[i].first << " distance: " << invalidatedExitGoals[i].second << endl;
//                if (abs(invalidatedExitGoals[i].first) > 60) {//to limit the turn angle
//                    if (invalidatedExitGoals[i].first > 0)
//                        invalidatedExitGoals[i].first = 60;
//                    else
//                        invalidatedExitGoals[i].first = -60;
//                }
                //        waitHere();
                verifiedDestination = avoidObstacleUsingPathPolygon(invalidatedExitGoals[i].first, invalidatedExitGoals[i].second, currentView);
                if (verifiedDestination.getType() == 2) {//means got obstacle avoided Destination
                    invalidatedExitGoals[i].first = verifiedDestination.getAngle();
                    invalidatedExitGoals[i].second = verifiedDestination.getDist();
//                    cout<<"once verified angle "<<verifiedDestination.getAngle()<<" dist "<<verifiedDestination.getDist()<<endl;
                    //attempts++;
                }
                if (verifiedDestination.getType() == 1) {
                    //                waitHere();
                    cout<<"Exit Goal is reachable:)"<<endl;
                    return verifiedDestination; //means obstacle free goal. verified.
                }
                if (verifiedDestination.getType() == 3) {
                    break; //too narrow path.not possible to go through 
                }
            } while (attempts < 3);

            attempts = 0;
        }
    }
    
    //here means there is no exit goal or exit goals are not reachable 
    if (invalidatedGoals.size() > 0) {//no else it will be only if condition

        for (unsigned int i = 0; i < invalidatedGoals.size(); i++) {
            cout << endl << "(searching) Going to check next invalidated goal no. " << i + 1 << endl;
            do {
                attempts++;
                cout << "Attempts Number " << attempts << endl;
                //        verifiedDestination = avoidObstacle(invalidatedGoals[0].first, invalidatedGoals[0].second, currentView, 1);
                cout << "Invalidated Goal Angle:  " << invalidatedGoals[i].first << " distance: " << invalidatedGoals[i].second << endl;
//                if (abs(invalidatedGoals[i].first) > 60) {//to limit the turn angle
//                    if (invalidatedGoals[i].first > 0)
//                        invalidatedGoals[i].first = 60;
//                    else
//                        invalidatedGoals[i].first = -60;
//                }
                //        waitHere();
                verifiedDestination = avoidObstacleUsingPathPolygon(invalidatedGoals[i].first, invalidatedGoals[i].second, currentView);
                if (verifiedDestination.getType() == 2) {//means got obstacle avoided Destination
                    invalidatedGoals[i].first = verifiedDestination.getAngle();
                    invalidatedGoals[i].second = verifiedDestination.getDist();
//                    cout<<"once verified angle "<<verifiedDestination.getAngle()<<" dist "<<verifiedDestination.getDist()<<endl;
                    //attempts++;
                }
                if (verifiedDestination.getType() == 1) {
                    //                waitHere();
                    return verifiedDestination; //means obstacle free goal. verified.
                }
                if (verifiedDestination.getType() == 3) {
                    break; //too narrow path.not possible to go through 
                }
            } while (attempts < 3);

            attempts = 0;
        }
    }

    //here means all type of goals are failed 
    //now if there was any exit goal then take a turn towards that or just take a turn of 30 deg to find other goals
//    cout<<"No invalidated goal left. so take a turn to go back from this deadend:)"<<endl;
    if(worstCaseAngle == 0) {
        cout<<"I couldn't find any particular goal..so taking an uncertain turn to find path:))"<<endl;
        worstCaseAngle = 0;
    }
    else
        cout<<"No reachable goals...so taking a turn towards exit/searchPath...."<<endl;
    verifiedDestination.setAngle(0);//just take a turn from this deadEnd
    verifiedDestination.setDist(0);
    cout<<"WorstCaseAngle: "<<worstCaseAngle<<endl;
//    waitHere();
    return verifiedDestination;
}

Destination avoidObstacleUsingPathPolygon(double angleToDest,double distanceToDest,vector<Object> currentView){
    cout << "\n\033[1;34m               avoiding obstacle for NextDestination using whole path polygon            \033[0m" << endl << endl;
    Object currentRobotPose(0, 0, 0, 500, 1);
    PointXY tentativeGoal(getX(angleToDest, distanceToDest), getY(angleToDest, distanceToDest));
    Object tentativeRobotPose(tentativeGoal.getX(), tentativeGoal.getY(), getX(angleToDest, distanceToDest + 500), getY(angleToDest, distanceToDest + 500));
       
    //check whether robot is going to cross an exit
    //if robot comes too close to exit then from CV it can't see exit bcz of robot direction
    //may be have to use exit info from previous view as well.
//    vector<Exit> exitsFromCV = findShortestExits(currentView);
//    if (exitsFromCV.size() > 0) {
//        vector<Object> exitsAsObjects = convertExitToObject(exitsFromCV);
//        Object nextRobotPath(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
//        double crossedExitIndex = splitToFromNewASR(exitsAsObjects, nextRobotPath);
//        if ( crossedExitIndex > -1) {
//            cout << "Going to cross a exit!!!" << endl;
//            nextRobotPath.setP2(exitsAsObjects[crossedExitIndex].mpX(),exitsAsObjects[crossedExitIndex].mpY());
//            double angle = currentRobotPose.getAngleWithLine(nextRobotPath);
//            double distance = nextRobotPath.length()-200;//if will add (100/200) with length then robot collide at back for small exit
//            Destination verifiedDestination;
//            verifiedDestination.setAngle(angle);
//            verifiedDestination.setDist(distance);
//            verifiedDestination.setType(1);//exit destination
//            cout<<"Destination Angle: "<<angle<<" distance: "<<distance<<endl;
////            waitHere();
//            return verifiedDestination;          
//        }
//    }
    
    MyRobot myrobot(0, 0);
//    vector<Object> tentativeRobot = myrobot.inMFIS(tentativeRobotPose, currentRobotPose, 1);//robot at next Destination
    vector<Object> robotWholePath = myrobot.robotPathForNextDest(angleToDest,distanceToDest);
    
    vector<Surface> robotPolygon(4);
    for (unsigned i = 0; i < 4; i++) {
        robotPolygon[i] = Surface(PointXY(robotWholePath[i].X1(), robotWholePath[i].Y1()), PointXY(robotWholePath[i].X2(), robotWholePath[i].Y2()), true);
    }
    
    Destination obsAvoidedDestination;
    obsAvoidedDestination.setType(1);//Destination is free to go if not then it will be changed later
    
    //looking for obstacles in the whole path
    Point obstacleAvoidEnd;
    bool isP1Inside, isP2Inside;
    bool needToTurnLeft = false;
    bool needToTurnRight = false;
    Object objectOriginToObstacleEnd;
    double angleToObstacleEnd;
    double p1PerpendicularDist, p2PerpendicularDist;
    vector<double> obstacleAndFSXPoint;
    double angleToAvoidForThisObs;
    double shiftAngle = 0;//when there is no obstacle then this 0 value is necessary bcz at the end it will be subtract from angleToDest
    double reduceDistance = 0;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        isP1Inside = pointInPolygon(PointXY(currentView[i].X1(), currentView[i].Y1()), robotPolygon, true);
        isP2Inside = pointInPolygon(PointXY(currentView[i].X2(), currentView[i].Y2()), robotPolygon, true);
        if(isP1Inside == true && isP2Inside == false) { //Obstacle p1 inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].X1(),currentView[i].Y1(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"P1 inside... angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }           
            obstacleAvoidEnd.set(currentView[i].X1(),currentView[i].Y1());
        } else if(isP1Inside == false && isP2Inside == true) {//Obstacle p2 inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].X2(),currentView[i].Y2(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"P2 inside... angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }  
            obstacleAvoidEnd.set(currentView[i].X2(),currentView[i].Y2());
        } else if(isP1Inside == true && isP2Inside == true) {//Obstacle's both ends inside
            objectOriginToObstacleEnd.set(0,0,currentView[i].mpX(),currentView[i].mpY(),1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                needToTurnRight = true;
            } else {
                needToTurnLeft = true;
            }  
            
            Object objectOriginToNextRobotPos(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
            p1PerpendicularDist = objectOriginToNextRobotPos.perpendicularDistOfPoint(currentView[i].X1(),currentView[i].Y1());
            p2PerpendicularDist = objectOriginToNextRobotPos.perpendicularDistOfPoint(currentView[i].X2(),currentView[i].Y2());
            if(p1PerpendicularDist < p2PerpendicularDist) {
                obstacleAvoidEnd.set(currentView[i].X1(),currentView[i].Y1());
            } else
                obstacleAvoidEnd.set(currentView[i].X2(),currentView[i].Y2());
        } else if(checkForIntersection(currentView[i], robotWholePath[1]) == 1) {//both ends are outside and crossed with front side
            obstacleAndFSXPoint = getIntersectionPoint(robotWholePath[1],currentView[i]);
            objectOriginToObstacleEnd.set(0,0,obstacleAndFSXPoint[0],obstacleAndFSXPoint[1],1);
            angleToObstacleEnd = currentRobotPose.getAngleWithLine(objectOriginToObstacleEnd);
            cout<<"Xwith FS...angleToObstacleEnd: "<<angleToObstacleEnd<<endl;
            if(angleToObstacleEnd > angleToDest) {//turn dir to avoid obstacle
                reduceDistance = robotWholePath[0].length() - robotWholePath[0].distP1ToPoint(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
                needToTurnRight = true;
            } else {
                reduceDistance = robotWholePath[1].length() - robotWholePath[1].distP1ToPoint(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
                needToTurnLeft = true;
            }  
            obstacleAvoidEnd.set(obstacleAndFSXPoint[0],obstacleAndFSXPoint[1]);
        }
        
        
        if(needToTurnRight == true && needToTurnLeft == true) {
            cout<<"Path is so narrow.....not possible to go through..."<<endl;
            obsAvoidedDestination.setType(3);//means need to change path direction
            return obsAvoidedDestination;
        }else if(needToTurnRight == true) {//amount of turn angle
            Object objectSideEndToObsEnd(robotWholePath[0].X1(),robotWholePath[0].Y1(),obstacleAvoidEnd.X(),obstacleAvoidEnd.Y(),1);
            angleToAvoidForThisObs = robotWholePath[0].getAngleWithLine(objectSideEndToObsEnd);
            if(abs(angleToAvoidForThisObs) > abs(shiftAngle))//to find max shift angle in case of multiple obstacles
                shiftAngle = angleToAvoidForThisObs;
            obsAvoidedDestination.setType(2);//obstacle avoided destination
        }else if(needToTurnLeft == true) {//amount of turn angle
//            cout<<"Need to turn left: "<<endl;
            Object objectSideEndToObsEnd(robotWholePath[2].X1(),robotWholePath[2].Y1(),obstacleAvoidEnd.X(),obstacleAvoidEnd.Y(),1);
            angleToAvoidForThisObs = robotWholePath[2].getAngleWithLine(objectSideEndToObsEnd);
            if(abs(angleToAvoidForThisObs) > abs(shiftAngle))//to find max shift angle in case of multiple obstacles
                shiftAngle = angleToAvoidForThisObs;
            obsAvoidedDestination.setType(2);//obstacle avoided destination
        }
    }    
    cout<<"Shift angle "<<shiftAngle<<endl;
    obsAvoidedDestination.setAngle(angleToDest+shiftAngle);
    obsAvoidedDestination.setDist(distanceToDest-reduceDistance);
    return obsAvoidedDestination;
}

Destination avoidObstacle(double angleToDest,double distanceToDest,vector<Object> currentView,int attempts) {
    cout << "\n\033[1;34m               avoiding for NextDestination             \033[0m" << endl << endl;
    Object currentRobotPose(0, 0, 0, 500, 1);
    PointXY tentativeGoal(getX(angleToDest, distanceToDest), getY(angleToDest, distanceToDest));
    Object tentativeRobotPose(tentativeGoal.getX(), tentativeGoal.getY(), getX(angleToDest, distanceToDest + 500), getY(angleToDest, distanceToDest + 500));

    //check whether robot is going to cross an exit
    vector<Exit> exitsFromCV = findShortestExits(currentView);
//    cout << "No of shortest exits: " << exitsFromCV.size() << endl;
    if (exitsFromCV.size() > 0) {
        vector<Object> exitsAsObjects = convertExitToObject(exitsFromCV);
        Object nextRobotPath(0, 0, tentativeGoal.getX(), tentativeGoal.getY(), 1);
        double crossedExitIndex = splitToFromNewASR(exitsAsObjects, nextRobotPath);
        if ( crossedExitIndex > -1) {
            cout << "Going to cross a exit!!!" << endl;
            nextRobotPath.setP2(exitsAsObjects[crossedExitIndex].mpX(),exitsAsObjects[crossedExitIndex].mpY());
            double angle = currentRobotPose.getAngleWithLine(nextRobotPath);
            double distance = nextRobotPath.length()+100;
            Destination verifiedDestination;
            verifiedDestination.setAngle(angle);
            verifiedDestination.setDist(distance);
            cout<<"Destination Angle: "<<angle<<" distance: "<<distance<<endl;
            return verifiedDestination;          
        }
    }
    
    
    MyRobot myrobot(0, 0);
    vector<Object> tentativeRobot = myrobot.inMFIS(tentativeRobotPose, currentRobotPose, 1);

//    cout << "Angle btw currentRP and robot's left side " << currentRobotPose.getAngleWithLine(tentativeRobot[0]) << endl;

    vector<Surface> robotPolygon(4);
    for (unsigned i = 0; i < 4; i++) {
        robotPolygon[i] = Surface(PointXY(tentativeRobot[i].X1(), tentativeRobot[i].Y1()), PointXY(tentativeRobot[i].X2(), tentativeRobot[i].Y2()), true);
    }

    //    cout<<"Current view"<<endl;
    //    displayObjects(currentView);
            
    Object originToExit,originToNonExit, originToLeftSide, originToRightSide;
    double exitAngle,nonExitAngle, leftSideAngle, rightSideAngle,angleToAvoid;
    bool obstacleFreePath = true;
    //check for left side
    angleToAvoid = 0;//when there is no obstacle then this 0 value is necessary bcz at the end it will be subtract from angleToDest
    for (unsigned int i = 0; i < currentView.size(); i++) {
//        if (currentView[i].getPEP1() == true or currentView[i].getPEP2() == true) {//guess. it will happen unless xtra-ordinary case
            double xWithLS = checkForIntersection(currentView[i], tentativeRobot[0]);
            double xWithFS = checkForIntersection(currentView[i], tentativeRobot[1]);
            double xWithRS = checkForIntersection(currentView[i], tentativeRobot[2]);
            bool isP1Inside = pointInPolygon(PointXY(currentView[i].X1(), currentView[i].Y1()), robotPolygon, true);
            bool isP2Inside = pointInPolygon(PointXY(currentView[i].X2(), currentView[i].Y2()), robotPolygon, true);
            if (xWithLS == 1 or xWithFS == 1 or xWithRS == 1 or isP1Inside == true or isP2Inside == true) {
                cout<<endl<<endl<<"Bloody obstacle on my way!!! going to avoid...."<<endl<<endl;
                obstacleFreePath = false;
                if (currentView[i].getPEP1() == false && currentView[i].getPEP2() == true) {//exit is on p2 side
                    cout<<"P2 end is an exit Point"<<endl;
                    originToExit.set(0, 0, currentView[i].X2(), currentView[i].Y2(), 1);
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    originToNonExit.set(0,0,currentView[i].X1(), currentView[i].Y1(), 1);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                } else if (currentView[i].getPEP1() == true && currentView[i].getPEP2() == false) {//exit is on p1 side
                    cout<<"P1 end is an exit Point"<<endl;
                    originToExit.set(0, 0, currentView[i].X1(), currentView[i].Y1(), 1);
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    originToNonExit.set(0,0,currentView[i].X2(), currentView[i].Y2(), 1);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    cout<<"ExitAngle: "<<exitAngle<<" nonExitAngle: "<<nonExitAngle<<endl;
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        if(rightSideAngle > 180)
                            rightSideAngle = rightSideAngle-360;
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                } else {
                    cout<<"this object doesn't contain exit. it's neighbor. search that object"<<endl;
                    for(unsigned int j=i+1;j<currentView.size();j++) {
                        if(currentView[j].getPEP1() == true or currentView[j].getPEP2() == true) {
                            if(currentView[j].getPEP1() == true) {
                                originToExit.set(0, 0, currentView[j].X1(), currentView[j].Y1(), 1);
                                originToNonExit.set(0,0,currentView[j].X2(), currentView[j].Y2(), 1);
                            } else {
                                originToExit.set(0, 0, currentView[j].X2(), currentView[j].Y2(), 1);
                                originToNonExit.set(0,0,currentView[j].X1(), currentView[j].Y1(), 1);
                            }
                            break;
                        }
                    }
                    exitAngle = currentRobotPose.getAngleWithLine(originToExit);
                    nonExitAngle = currentRobotPose.getAngleWithLine(originToNonExit);
                    cout<<"ExitAngle: "<<exitAngle<<" nonExitAngle: "<<nonExitAngle<<endl;
                    if(exitAngle < nonExitAngle) {//need to turn right
                        cout<<"Need to turn right"<<endl;
                        originToLeftSide.set(0, 0, tentativeRobot[0].X1(), tentativeRobot[0].Y1(), 1);
                        leftSideAngle = currentRobotPose.getAngleWithLine(originToLeftSide);
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "LeftSide Angle " << leftSideAngle << endl;
                        cout<<"ShiftAngle: "<<leftSideAngle-exitAngle<<endl;
                        angleToAvoid = leftSideAngle-exitAngle;
                    } else {//need to turn left
                        cout<<"Need to turn left"<<endl;
                        originToRightSide.set(0, 0, tentativeRobot[2].X1(), tentativeRobot[2].Y1(), 1);
                        rightSideAngle = currentRobotPose.getAngleWithLine(originToRightSide);
                        if(rightSideAngle > 180)
                            rightSideAngle = rightSideAngle-360;
                        cout << "ExitBoundary Angle " << exitAngle << endl;                
                        cout << "rightSide Angle " << rightSideAngle << endl;
                        cout<<"ShiftAngle: "<<rightSideAngle-exitAngle<<endl;
                        angleToAvoid = rightSideAngle-exitAngle;
                    }
                }                
            }
        //}

    }
    
    Destination verifiedDestination;
    double newAngleToDest = angleToDest-angleToAvoid;
    verifiedDestination.setAngle(newAngleToDest);
    verifiedDestination.setDist(distanceToDest);
    
    if(obstacleFreePath == false) {
        verifiedDestination.setType(2);
    }
    else
        verifiedDestination.setType(1);
    
    
    return verifiedDestination;
}

//goalType = 1 for Exit goals
//goalType = 2 for farthest path goals
vector<pair<double, double> > findInvalidatedGoals(vector<Object> currentView, int goalType) {
    vector<pair<double, double> > result;
    pair<double, double> goal; //first - angle, second- distance 
    vector<Object> sortedCurrentView;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (currentView[i].length() > 400)//this condition is actually for farthest path goal but it doesn't matter for exit goal
            sortedCurrentView.push_back(currentView[i]);//bcz all exits are longer than 400
    }
    std::sort(sortedCurrentView.begin(), sortedCurrentView.end(), sortA2MPDistanceFromOrigin);

    //calculating angle and distance for all goals
    double angleToMaxPath;
    double distanceAlongMaxPath;
    Object maxPath;
    Object robotPose(0, 0, 0, 500, 1);
    for (unsigned int i = 0; i < sortedCurrentView.size(); i++) {
        //calculating angle
        maxPath.set(0, 0, sortedCurrentView[i].mpX(), sortedCurrentView[i].mpY(), 1); //max path object
        angleToMaxPath = robotPose.getAngleWithLine(maxPath); //angle to max path

        //calculating distance
        if (maxPath.length() > 1400)
            distanceAlongMaxPath = 1000;
        else if (maxPath.length() > 400)
            distanceAlongMaxPath = maxPath.length() - 400;
        else if (goalType == 1) //so when robot close to exit then just pass the exit
            distanceAlongMaxPath = maxPath.length() + 200;
        else
            distanceAlongMaxPath = 0;//need to comment out
        
        goal.first = angleToMaxPath;
        goal.second = distanceAlongMaxPath;
        result.push_back(goal);
    }
    return result;
}

vector<Exit> findExits(vector<Object> cv) 
{
    cout << "\033[1;32m-------------------Inside findExits module---------------------\033[0m" << endl;
    //vector<Exit> result;
    //vector<Object> bObjects;//Objects on asr boundary
    double distp2top1;
    //double mindist=400;
    
    for(unsigned int i=0;i<cv.size();i++) 
    {
        cv[i].setID(i+1);
    }
    
    Exit exit;
    int no_exit;
    vector<Exit> exits;
    cout << endl << "******* Finding exits(yeap's theory) ******* " << endl;
    
//    //first side exit
//    exit.set(0, 0, cv[0].X1(), cv[0].Y1()); 
//    exit.setID(1);
//    exit.setP1ID(0);
//    exit.setP2ID(1);
//    exits.push_back(exit);
    
    int exits_counter = 2;
    
   
    for (int i = 0; i<int(cv.size()-1); i++) 
    {
            if (cv[i].getPEP2() == true) //p2 is a probable exit end(p1)
            { 
                    //cout<<"  "<<i<<" "<<cv[i].getID()<<endl;
                    no_exit = 0;
                    for (int j = i + 1; j<int(cv.size()); j++) 
                    {
                        if (cv[j].getPEP1() == true) 
                        { //p1 is a probable exit end(p2)
                                if (no_exit == 0) 
                                {
                                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                    exit.setP1ID(cv[i].getID());
                                    exit.setP2ID(cv[j].getID());
                                    no_exit++;
                                } 
                                else 
                                {
                                        distp2top1 = cv[i].distP2ToP1(cv[j]);
                                        if (exit.length() > distp2top1)
                                        { //condition to get shortest exit
                                            exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                                            exit.setP1ID(cv[i].getID());
                                            exit.setP2ID(cv[j].getID());

                                        }
                                }
                              // i = i-1;//new line for place gaps
                        }//if cv j
                     }//for j
                //bObjects.push_back(cv[i]);//boundary Object having exit vertex
                //i = exit.getP2ID() - 2; //for triming for cv

                //exit.setID(exits.back().getID()+1);
                exit.setID(exits_counter);
                exits.push_back(exit);
                exits_counter++;
                //exit.display();

                //cout<<"jj "<<jj<<" id "<<exit.getP2ID()<<endl;
                //i=j-1;
            }//if cv i
    }// for i

    //Point tmpp=cv[33].shortestDistPointFrom(cv[49].X2(),cv[49].Y2());
    //exits.back().set(cv[49].X2(),cv[49].Y2(),tmpp.X(),tmpp.Y());

    //last side exit
//    exit.set(cv[int(cv.size() - 1)].X2(), cv[int(cv.size() - 1)].Y2(), 0, 0); 
//    exit.setID(exits_counter);
//    exit.setP1ID(cv[int(cv.size() - 1)].getID());
//    exit.setP2ID(0);
//    exits.push_back(exit);
    
    //displayObjects(exits);
    //cout<<"boundary Objects "<<endl;displayObjects(bObjects);



    //result.push_back(bObjects);
    //result.push_back(exits);

    //cout<<" last Object of cv "<<endl;cv.back().display();

    //std::sort(exits.begin(),exits.end(),sortExitsA2L);
    return exits;


}

//modified version of findExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits

vector<Exit> findShortestExits(vector<Object> cv) 
{
        //vector<Exit> result;
        //vector<Object> bObjects;//Objects on asr boundary
        double distp2top1;
        //double mindist=400;
        Exit exit;
        int no_exit;
        vector<Exit> exits;
        cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
        //cout << "CV" << endl;
        //displayObjects(cv);
        int exits_counter = 1;
        for (int i = 0; i<int(cv.size()); i++) {
            if (cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
                no_exit = 0;
                for (int j = i + 1; j<int(cv.size()); j++) {
                   // if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) { //p1 is a probable exit end(p2)
                    if (no_exit == 0) {
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[j].getID());
                        no_exit++;
                    } else {
                        distp2top1 = cv[i].distP2ToP1(cv[j]);
                        if (exit.length() > distp2top1) { //condition to get shortest exit
                            exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                            exit.setP1ID(cv[i].getID());
                            exit.setP2ID(cv[j].getID());
                        }
                    }
                    //}//if cv j
                }//for j

                i = exit.getP2ID() - 2; //for triming
    //            cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
                exit.setID(exits_counter);
                //find the nearest point instead of p2
                Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
                exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

                exits.push_back(exit);
                exits_counter++;
                //exit.display();

                //i=j-1;
            }//if cv i

        }// for i
    //    cout << "shortest exits" << endl;
    //    displayExits(exits);

        vector<Exit> realExits;
        for (int i = 0; i<int(exits.size()); i++) {
            if (exits[i].length() > 500 )//&& exits[i].length() < 1200)
                realExits.push_back(exits[i]);
        }

        return realExits;
}

//modified version of findShortestExits()
//Definition of DestinationExits:  when some objects are seen through exits then those exits are called destination Exits
//in other words, some shortest exits are not destination exits bcz those are uncertain gaps on the boundary 
//but destination exits are those shortest exits which are not on the CV boundary 
//implementation: extra condition abs(exits[i].getP2ID()-exits[i].getP1ID()) > 1 is added at the end
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits

vector<Object> findDestinationExits(vector<Object> cv, vector<Object> refObjects) {

    vector<Exit> exits = findShortestExits(cv);
    vector<Exit> realExits;
    for (int i = 0; i<int(exits.size()); i++) { //real exits have specific length
        if (exits[i].length() > 600 && exits[i].length() < 1200)
            realExits.push_back(exits[i]);
    }
    cout<<"Shortest Exits from CV "<<endl;
    displayExits(realExits);
    vector<Exit> destExits;
    Object originToObject, exitAsObject;
    bool itsAnExit = true;
    for (unsigned int i = 0; i<realExits.size(); i++) {
        if (abs(realExits[i].getP2ID()-realExits[i].getP1ID()) > 1 ){//two consecutive objects means uncertain exit/gap
            exitAsObject.set(realExits[i].X1(),realExits[i].Y1(),realExits[i].X2(),realExits[i].Y2(),1);
            itsAnExit = true;
            for(unsigned int j = realExits[i].getP1ID(); j < realExits[i].getP2ID()-1;j++) {
                originToObject.set(0,0,cv[j].mpX(),cv[j].mpY(),2);
                if(checkForIntersection(exitAsObject,originToObject) != 1) {//to filter those exits who has obstacle towards origin
//                    cout<<"it's not an exit "<<i+1<<endl;
                    itsAnExit = false;
                    break;
                }
            }
            if(itsAnExit == true)
                destExits.push_back(realExits[i]);
        }
    }
    
    cout<<"Destination Exits: "<<endl;
    displayExits(destExits);
    
    
    vector<Object> destinationExits = convertExitToObject(destExits);    
    
//    plotObjectsAndPExits("MFIS/destExits.png",destinationExits,cv,realExits);
//    waitHere();
//    vector<Object> destinationExitsInMFIS;
//    for(unsigned int i = 0; i<destinationExits.size();i++) {
//        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0],refObjects[1],destinationExits[i],1,0,refObjects[0].getKP()));
//    }
//
//    cout<<"Destination Exits as Objects"<<endl;
//    displayObjects(destinationExitsInMFIS);

//    return destinationExitsInMFIS;
    return destinationExits;
}

//Difference btw this function and findDestinationExits
//findDestinationExits returns only real Exits(who has specific length)
//this function returns all exits(Gaps) whose length more than 60cm
vector<Object> findAllDestinationExits(vector<Object> cv, vector<Object> refObjects) {

    vector<Exit> exits = findShortestExits(cv);
    vector<Exit> realExits;
    for (int i = 0; i<int(exits.size()); i++) { //real exits have specific length
        if (exits[i].length() > 600 )//&& exits[i].length() < 1200)
            realExits.push_back(exits[i]);
    }
    cout<<"Shortest Exits from CV "<<endl;
    displayExits(realExits);
    vector<Exit> destExits;
    Object originToObject, exitAsObject;
    bool itsAnExit = true;
    for (unsigned int i = 0; i<realExits.size(); i++) {
        if (abs(realExits[i].getP2ID()-realExits[i].getP1ID()) > 1 ){//two consecutive objects means uncertain exit/gap
            exitAsObject.set(realExits[i].X1(),realExits[i].Y1(),realExits[i].X2(),realExits[i].Y2(),1);
            itsAnExit = true;
            for(unsigned int j = realExits[i].getP1ID(); j < realExits[i].getP2ID()-1;j++) {
                originToObject.set(0,0,cv[j].mpX(),cv[j].mpY(),2);
                if(checkForIntersection(exitAsObject,originToObject) != 1) {//to filter those exits who has obstacle towards origin
//                    cout<<"it's not an exit "<<i+1<<endl;
                    itsAnExit = false;
                    break;
                }
            }
            if(itsAnExit == true)
                destExits.push_back(realExits[i]);
        }
    }
    
    cout<<"Destination Exits: "<<endl;
    displayExits(destExits);
    
    
    vector<Object> destinationExits = convertExitToObject(destExits);    
    
//    plotObjectsAndPExits("MFIS/destExits.png",destinationExits,cv,realExits);
//    waitHere();
//    vector<Object> destinationExitsInMFIS;
//    for(unsigned int i = 0; i<destinationExits.size();i++) {
//        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0],refObjects[1],destinationExits[i],1,0,refObjects[0].getKP()));
//    }
//
//    cout<<"Destination Exits as Objects"<<endl;
//    displayObjects(destinationExitsInMFIS);

//    return destinationExitsInMFIS;
    return destinationExits;
}

vector<Object> tranformCVExitsInMFIS(vector<Object> cvExits, vector<Object> refObjects) {
    vector<Object> destinationExitsInMFIS;
    for (unsigned int i = 0; i < cvExits.size(); i++) {
        destinationExitsInMFIS.push_back(remakeLineP2(refObjects[0], refObjects[1], cvExits[i], 1, 0, refObjects[0].getKP()));
    }
    return destinationExitsInMFIS;
}

//modified version of findExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(probable exit point 2 of objects)
//		  p2 is the end point of exit(nearest point on the objects which contains the nearest probable exit point 1)
// vital condition is exit length should be between 800 to 1200 cm.
//this module is used in PartialUpdating.cpp which updates the map only before crossing the exits
//developed on Tuesday Oct 11, 2011
vector<Exit> findNewASRFormingExits(vector<Object> cv) {
    //vector<Exit> result;
    //vector<Object> bObjects;//Objects on asr boundary
    double distp2top1,probableExitLength;
    //double mindist=400;
    Point sDistPoint;
    Exit exit;
    int no_exit;
    vector<Exit> exits;
    cout << endl << "******* Finding exits(modified version yeap's theory) ******* " << endl;
    //cout << "CV" << endl;
    //displayObjects(cv);
    int exits_counter = 2;
    for (int i = 0; i<int(cv.size()); i++) {
        if (cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
            no_exit = 0;
            for (int j = i + 1; j<int(cv.size()); j++) {
                //if(cv[j].getPEP1() == true) { //p1 is a probable exit end(p2)
               /* if (no_exit == 0) {
                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    no_exit++;
                } else {
                    distp2top1 = cv[i].distP2ToP1(cv[j]);
                    if (exit.length() > distp2top1) { //condition to get shortest exit
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[j].getID());
                    }
                }*/
                //}//if cv j
                probableExitLength = cv[j].shortestDistFrom(cv[i].X2(),cv[i].Y2());
                if(probableExitLength > 800 && probableExitLength < 1200) {
                    //find the shortest point on cv[j] from cv[i] p2 
                    sDistPoint = cv[j].shortestDistPointFrom(cv[i].X2(),cv[i].Y2());
                    exit.set(cv[i].X2(), cv[i].Y2(),sDistPoint.X(),sDistPoint.Y());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    exit.setID(exits_counter);
                    exits.push_back(exit);
                    exits_counter++;
                }
                
            }//for j

//            i = exit.getP2ID() - 2; //for triming
//            cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
//            
//            //find the nearest point instead of p2
//            Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
//            exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

            
           
            //exit.display();

            //i=j-1;
        }//if cv i

    }// for i
//   
    return exits;
}

//modified version of findShortestExits()
//finds exits e(p1,p2)
// where, p1 is the first point of exit(p2 of any objects)
//		  p2 is the end point of exit(nearest point on any other objects except objects which are in the same group
//			of p1 containing object)
//this module is used in Algorithm3.cpp to compute ASR(Algorithm3 split ASR when robot crosses a exit of this kind

vector<Exit> findShortestExitsForASR(vector<Object> cv) {

    double distp2top1, shortestDist;
    Exit exit;
    int no_exit, nextPEP1=0;
    vector<Exit> exits;
    int exits_counter = 2;

    for (int i = 0; i<int(cv.size()); i++) {
        //if(cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
        for (int k = i + 1; k<int(cv.size()); k++)
            if (cv[k].getPEP1() == true)
                nextPEP1 = k;
        no_exit = 0;
        for (int j = nextPEP1; j<int(cv.size()); j++) {
            shortestDist = cv[j].shortestDistFrom(cv[i].X2(), cv[i].Y2());
            if (shortestDist > 600 && shortestDist < 1200) {//p1 is a probable exit end(p2)
                //if(cv[j].getPEP1() == true) { //p1 is a probable exit end(p2)
                if (no_exit == 0) {
                    exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                    no_exit++;
                } else {
                    distp2top1 = cv[i].distP2ToP1(cv[j]);
                    if (exit.length() > distp2top1) { //condition to get shortest exit
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[j].X1(), cv[j].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[j].getID());
                    }
                }
                //}//if cv j
            }//for j

            i = exit.getP2ID() - 2; //for triming
            cout << "got one exit " << exit.getP1ID() << " " << exit.getP2ID() << endl;
            exit.setID(exits_counter);
            //find the nearest point instead of p2
            Point tmpp = cv[exit.getP2ID() - 1].shortestDistPointFrom(exit.X1(), exit.Y1());
            exit.set(exit.X1(), exit.Y1(), tmpp.X(), tmpp.Y());

            exits.push_back(exit);
            exits_counter++;
        }//if cv i

    }// for i
    cout << "shortest exits" << endl;
    displayExits(exits);

    return exits;
}

vector<Exit> findGateways(vector<Object> cv) {
	cout<<endl<<"Inside gateway finding module "<<endl;
	//cout<<"cv size "<<cv.size()<<endl;
	//displayObjects(cv);

	double distp2top1;

	Exit exit;
	int no_exit;
	vector<Exit> exits;
	cout<<endl<<"******* Finding Gateways******* "<<endl;
	int exits_counter=1;
	for(int i=0;i<int(cv.size()-1);i++) {
		if(cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
			no_exit=0;
			for(int j=i+1;j<int(cv.size());j++) {
				if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) {//&& cv[i].distP2ToP1(cv[j]) > 600){p1 is a probable exit end(p2)
					if(no_exit == 0) {
						Object tmp;
						tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
						bool obs=false;
						for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
							if(checkForIntersection(tmp,cv[k]) == 1) {
								obs=true;
							}
						}
						if(obs == true) {
							exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[i+1].getID());
						}
						else {
							exit.set(tmp);
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[j].getID());
						}//
						no_exit++;
					}
					else {
						distp2top1=cv[i].distP2ToP1(cv[j]);
						double dist_from_robot=cv[j].distP1ToPoint(0,0);
						//condition to get shortest exit
						if(exit.length() > distp2top1 && exit.distP2ToPoint(0,0) > dist_from_robot) {
							Object tmp;
							tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
							bool obs=false;
							for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
								if(checkForIntersection(tmp,cv[k]) == 1) {
									obs=true;
								}
							}
							if(obs == true) {
								exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
								exit.setP1ID(cv[i].getID());
								exit.setP2ID(cv[i+1].getID());
							}
							else {
								exit.set(tmp);
								exit.setP1ID(cv[i].getID());
								exit.setP2ID(cv[j].getID());
							}//
						}
					}
				}//if cv j
			}//for j

			if(no_exit > 0) {
				i=exit.getP2ID()-2;//for triming
				//cout<<"i "<<i<<endl;

				exit.setID(exits_counter);
				exits.push_back(exit);
				exits_counter++;
			}

		}//if cv i

	}// for i

	//setting angle with robot direction
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		//Object exit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		Object exit(0,0,exits[i].X1(),exits[i].Y1(),1);
		exits[i].setAngle(abs(exit.getAngleWithLine(edirection)));//rdirection.getAngleWithLine(edirection));
//		cout<<"angle with x axis "<<exit.getAngleWithXaxis()<<endl;
		//exits[i].display();

	}

	//cout<<"exits size "<<exits.size()<<endl;
	//displayExits(exits);
	std::sort(exits.begin(),exits.end(),sortExitsA2A);
	displayExits(exits);
	//char go;
	//cin>>go;
	return exits;
}

vector<Exit> findGatewaysNew(vector<Object> cv) {
	cout<<endl<<"Inside New gateway finding module "<<endl;
	//cout<<"cv size "<<cv.size()<<endl;
	//displayObjects(cv);

	//double distp2top1;

	vector<Object> rsides;
	Object rside;
	rside.set(-200,0,-200,500,1);
	rsides.push_back(rside);
	rside.set(200,0,200,500,2);
	rsides.push_back(rside);



	Exit exit;
	//int no_exit;
	vector<Exit> exits, tmpexits;
	cout<<endl<<"******* Finding Gateways******* "<<endl;
	int exits_counter=1;
	for(int i=0;i<int(cv.size()-1);i++) {
		if(cv[i].getPEP2() == true) { //p2 is a probable exit end(it's p1 for exit)
			//tmpexits.clear();
			for(int j=i+1;j<int(cv.size());j++) {//looking for p2 end(p1 of objects) for this exit
				if(cv[j].getPEP1() == true && cv[j].getP1OS() == 1 ) {//&& cv[i].distP2ToP1(cv[j]) > 600) {//){p1 is a probable exit end(p2)
					//if(no_exit == 0) {
						Object tmp;
						tmp.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),1);
						rsides[0].setP2(tmp.mpX()-200,tmp.mpY()+500);//robot side 1
						rsides[1].setP2(tmp.mpX()+200,tmp.mpY()+500);//robot side 2
						bool obs=false;
						for(int k=cv[i].getID();k<cv[j].getID()-1;k++) {
							if(checkForIntersection(tmp,cv[k]) == 1) {
								obs=true;
							}
							else if(checkForIntersection(rsides[0],cv[k]) == 1) {
								obs=true;
							}
							else if(checkForIntersection(rsides[1],cv[k]) == 1) {
								obs=true;
							}
						}
						if(obs == false) {

							exit.set(tmp);
							exit.setP1ID(cv[i].getID());
							exit.setP2ID(cv[j].getID());
							exit.setP2DistFromRobot(exit.distP2ToPoint(0,0));
							tmpexits.push_back(exit);
						}

				}//if cv j
			}//for j
			int tmpsize=tmpexits.size();
			if(tmpsize == 0){ //if no p2 for this p1
				exit.set(cv[i].X2(),cv[i].Y2(),cv[i+1].X1(),cv[i+1].Y1());
				if(exit.length() > 600) { //don't push if not wide enough
					exit.setP1ID(cv[i].getID());
					exit.setP2ID(cv[i+1].getID());
					exits.push_back(exit);
					exits.back().setID(exits_counter);
					exits_counter++;
				}
				i=cv[i+1].getID()-2;//for trimming
			}
			else if(tmpsize == 1){//if only one for this p1
				if(tmpexits.back().length() > 600) {//don't push if not wide enough
					exits.push_back(tmpexits.back());
					exits.back().setID(exits_counter);
					exits_counter++;
				}
				tmpexits.clear();
				i=tmpexits[0].getP2ID()-2;//trimming
			}
			else { //if more than one for this p1
				Exit tmpe1;//tmp exit 1
				tmpe1.set(tmpexits[0]);
				double e1length=tmpe1.length();
				double e1dist=tmpe1.getP2DistFromRobot();
				double ellength,eldist;
				for(int l=1;l<int(tmpexits.size());l++) {
					ellength=tmpexits[l].length();
					eldist=tmpexits[l].getP2DistFromRobot();
					if(e1dist > eldist) {//e1length >  ellength &&
						tmpe1.set(tmpexits[l]);
						e1length=ellength;
						e1dist=eldist;
					}
				}
				if(tmpe1.length() > 600) {//don't push if not wide enough
					exits.push_back(tmpe1);
					exits.back().setID(exits_counter);
					exits_counter++;
					cout<<"only once "<<endl;
				}
				tmpexits.clear();
				i=tmpe1.getP2ID()-2;//trimming
			}

			//i=exits.back().getP2ID()-2;//trimming
			//if(exits.back().length() < 600) {

		}//if cv i

	}// for i

	//setting angle with robot direction
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		Object exit(0,0,exits[i].X1(),exits[i].Y1(),1);
		exits[i].setAngle(abs(exit.getAngleWithLine(edirection)));//rdirection.getAngleWithLine(edirection));
	}

	std::sort(exits.begin(),exits.end(),sortExitsA2A);
	displayExits(exits);

	return exits;
}


//new module(called for every steps) to find  next destination(have to use findGatewaysNew first).
Destination findNextDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides) {
	cout<<endl<<"Inside find Next destination module "<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	Destination destination;
	double pdist;

	if(exits.size() == 0) {
		bool notclear=false;
		robsides[7].setP2(getX(0,2000)-200,getY(0,2000));
		for(int i=0;i<int(cv.size());i++) {
			if(checkForIntersection(robsides[7],cv[i]) == 1)
				notclear=true;
		}
		if(notclear == false) {
			destination.setAngle(-45);
			destination.setDist(1000);
		}
		else {
			notclear=false;
			robsides[7].setP2(getX(-45,1000)-200,getY(-45,1000));
			for(int i=0;i<int(cv.size());i++) {
				pdist=robsides[7].perpendicularDistOfPoint(cv[i].X2(),cv[i].Y2());
				if(pdist < 100) {
					notclear=true;
				}
			}
			if(notclear == false) {
				destination.setAngle(-45);
				destination.setDist(1000);
			}
		}

	}//first if
	else {
		Object rdirection(0,0,0,300,1);
		Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
		double angle=rdirection.getAngleWithLine(edirection);

		cout<<"Exits "<<endl;
		displayExits(exits);

		robsides[7].setP2(getX(angle,1000)-200,getY(angle,1000));
		robsides[8].setP2(getX(angle,1000)+200,getY(angle,1000));

		pdist=robsides[7].perpendicularDistOfPoint(exits[0].X1(),exits[0].Y1());
		cout<<"pdist "<<pdist<<endl;

		//plotObjectsAndPExits("MFIS/Nextdestination.png",robsides,cv,exits);



		if(pdist > 200) {
			destination.setAngle(angle);
			destination.setDist(1000);
		}
		else {
			double rdist1=exits[0].distP1ToPoint(0,0);
			double rdist2=exits[0].distP2ToPoint(0,0);

			cout<<"rdist1 "<<rdist1<<" rdist2 "<<rdist2<<endl;
			//which point have to avoid
			if(rdist1 < rdist2) { //true means have to avoid first exit point
				destination.setAngle(-15);
				if(rdist1 < 900)
					destination.setDist(rdist1+100);
				else
					destination.setDist(1000);
			}
			else {
				destination.setAngle(15);
				if(rdist2 < 900)
					destination.setDist(rdist2+100);
				else
					destination.setDist(1000);
			}
		}
	}
	cout<<"Destination: angle "<<destination.getAngle()<<" dist: "<<destination.getDist()<<endl;
	return destination;
}

//new module(called for every steps) to find  next destination(have to use findGatewaysNew first).
Destination findDestToMaxPath(vector<Object> cv, vector<Object> robsides, int viewno) {
	cout<<endl<<"Inside find destination to max path module for view "<<viewno<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	Destination destination;

	//finding max path
	double mpdist=0;//cv[0].distMPToPoint(0,0);
	int maxindex=0;
	for(int i=0;i<int(cv.size());i++) {
		double tmpdist=cv[i].distMPToPoint(0,0);
		if(mpdist < tmpdist && cv[i].length() > 400) {
			maxindex=i;
			mpdist=tmpdist;
		}
	}

	//calculating direction to max path
	Object rdirection(0,0,0,300,1);
	Object edirection(0,0,cv[maxindex].mpX(),cv[maxindex].mpY(),1);//max path object
	double angle=rdirection.getAngleWithLine(edirection);//angle to max path

	//if(abs(angle) > 50) {
	if(abs(angle) > 50) {//limitting to recognize objects
		if(angle < 0)
			angle=-50;
		else
			angle = 50;
	}


	double destlength=1100;
	robsides[6].setP2(getX(angle,destlength),getY(angle,destlength));
	robsides[7].setP2(getX(angle,destlength)-350,getY(angle,destlength));//rob sides to max path direction
	robsides[8].setP2(getX(angle,destlength)+350,getY(angle,destlength));

	cout<<"direction angle "<<angle<<endl;

	/*cout<<"robsides "<<endl;
	robsides[6].display();
	robsides[7].display();
	robsides[8].display();
	*/
	//looking for any hindrance
	vector<double> interdist;
	bool notclear=false;
	vector<Obstacle> obstacles;
//	cout<<"pdist "<<robsides[7].perpendicularDistOfPoint(cv[0].X2(),cv[0].Y2());
	for(int i=0;i<int(cv.size());i++) {
		//cout<<"checking"<<endl;
		double x7=checkForIntersection(robsides[7],cv[i]);
		double x6=checkForIntersection(robsides[6],cv[i]);
		double x8=checkForIntersection(robsides[8],cv[i]);
		/*double ppd7=robsides[7].perpendicularDistOfPoint(cv[i].X2(),cv[i].Y2());
		double ppd8=robsides[8].perpendicularDistOfPoint(cv[i].X1(),cv[i].Y1());
		double rd7=robsides[7].distP2ToPoint(cv[i].X2(),cv[i].Y2());
		double rd8=robsides[8].distP2ToPoint(cv[i].X1(),cv[i].Y1());*/
		//if(i < 3)
			//cout<<" 7: "<<x7<<" 8: "<<x8<<" x6 "<<x6<<endl;


		//if(x7 ==0 && 8x ==0) {


		if( (x7 == 1 || x6 == 1) && x8 == 0) {//have to turn right
			vector<double> interpoint=getIntersectionPoint(robsides[7],cv[i]);
			//cout<<"inter sect with "<<cv[i].getID()<<" inter point "<<interpoint[0]<<" "<<interpoint[1]<<endl;

			Object tmp(robsides[7].X1(),robsides[7].Y1(),cv[i].X2(),interpoint[1],1);
			double od=rdirection.distP1ToP2(tmp);
			double oa=robsides[7].getAngleWithLine(tmp);
			Obstacle obs(1,od,oa+angle);
			obstacles.push_back(obs);
			notclear=true;
		}


		if( (x8 == 1 || x6 == 1) && x7 == 0) {//have to turn left
			vector<double> interpoint=getIntersectionPoint(robsides[8],cv[i]);
			//cout<<"inter sect with "<<cv[i].getID()<<" inter point "<<interpoint[0]<<" "<<interpoint[1]<<endl;
			Object tmp(robsides[8].X1(),robsides[8].Y1(),cv[i].X1(),interpoint[1],1);
			double od=rdirection.distP1ToP2(tmp);

			double oa=robsides[8].getAngleWithLine(tmp);
			Obstacle obs(2,od,oa+angle);
			obstacles.push_back(obs);

			notclear=true;
		}

		double dp1=rdirection.distP1ToP1(cv[i]);
		double dp2=rdirection.distP1ToP2(cv[i]);
		if( dp1 < 1500 ||  dp2 < 1500) {
			bool isp1inside=pointInPolygon(robsides[7],robsides[6],cv[i].X1(),cv[i].Y1());
			bool isp2inside=pointInPolygon(robsides[7],robsides[6],cv[i].X2(),cv[i].Y2());
			if(isp1inside == true || isp2inside == true) {
				double od;
				Object tmp;
				double px,py;
					if(isp1inside == true && isp2inside == false) {
						tmp.set(robsides[7].X1(),robsides[7].Y1(),cv[i].X1(),cv[i].Y1(),1);
						od=robsides[6].distP1ToPoint(cv[i].X1(),cv[i].Y1());
					}
					else if(isp1inside == false && isp2inside == true) {
						tmp.set(robsides[7].X1(),robsides[7].Y1(),cv[i].X2(),cv[i].Y2(),1);
						od=robsides[6].distP1ToPoint(cv[i].X2(),cv[i].Y2());
					}
					else {
						if(cv[i].X1() > cv[i].X2())
							px=cv[i].X1();
						else
							px=cv[i].X2();
						if(cv[i].Y1() < cv[i].Y2()){
							tmp.set(robsides[7].X1(),robsides[7].Y1(),px,cv[i].Y1(),1);		//take turn upto mid direct
							py=cv[i].Y1();
						}
						else {
							tmp.set(robsides[7].X1(),robsides[7].Y1(),px,cv[i].Y2(),1);
							py=cv[i].Y2();
						}
						od=robsides[6].distP1ToPoint(px,py);
					}
					robsides.push_back(tmp);

				double oa=robsides[7].getAngleWithLine(tmp);
				Obstacle obs(3,od,oa+angle);
				obstacles.push_back(obs);
				notclear=true;
				cout<<"Got LEFT inside obstacle.id-"<<cv[i].getID()<<endl;
			}
			else {
				isp1inside=pointInPolygon(robsides[6],robsides[8],cv[i].X1(),cv[i].Y1());
				isp2inside=pointInPolygon(robsides[6],robsides[8],cv[i].X2(),cv[i].Y2());
				if(isp1inside == true || isp2inside == true) {
					double od;
					Object tmp;
					double px,py;
					if(isp1inside == true && isp2inside == false) {
						tmp.set(robsides[8].X1(),robsides[8].Y1(),cv[i].X1(),cv[i].Y1(),1);
						od=robsides[6].distP1ToPoint(cv[i].X1(),cv[i].Y1());
					}
					else if(isp1inside == false && isp2inside == true) {
						tmp.set(robsides[8].X1(),robsides[8].Y1(),cv[i].X2(),cv[i].Y2(),1);
						od=robsides[6].distP1ToPoint(cv[i].X2(),cv[i].Y2());
					}
					else {
						if(cv[i].X1() < cv[i].X2())
							px=cv[i].X1();
						else
							px=cv[i].X2();
						if(cv[i].Y1() < cv[i].Y2()){
							tmp.set(robsides[8].X1(),robsides[8].Y1(),px,cv[i].Y1(),1);		//take turn upto mid direct
							py=cv[i].Y1();
						}
						else {
							tmp.set(robsides[8].X1(),robsides[8].Y1(),px,cv[i].Y2(),1);
							py=cv[i].Y2();
						}
						od=robsides[6].distP1ToPoint(px,py);
					}
					robsides.push_back(tmp);

					double oa=robsides[8].getAngleWithLine(tmp);
					cout<<"oa "<<oa<<endl;
					Obstacle obs(3,od,oa+angle);
					obstacles.push_back(obs);
					notclear=true;
					cout<<"Got RIGHT inside obstacle.id-"<<cv[i].getID()<<endl;
				}
			}
			//cout<<" oa "<<oa<<endl;
		}//for inside obstacle

	}

	if(notclear == false) { //means clear

		destination.setAngle(angle);
		destination.setDist(1000);
		destination.setType(1);
	}
	else {
		std::sort(obstacles.begin(),obstacles.end(),sortObsA2D);
		displayObstacles(obstacles);
		double oangle=obstacles[0].getAngle();
		double odist=obstacles[0].getDist();
		if(odist > 1000)//
			odist=1000;
		//else
			//odist=odist+100;

		cout<<"Max path isn't clear "<<endl;
		/*if(obstacles[0].getID() == 1 ) {//have to take right
			if(abs(oangle) < 15) {
				destination.setAngle(-15);
				destination.setDist(odist);
			}
			else {
				destination.setAngle(-abs(oangle));
				destination.setDist(odist);
			}
		}
		else if(obstacles[0].getID() == 2) {
			if(abs(oangle) < 15) {//have to take left
				destination.setAngle(15);
				destination.setDist(odist);
			}
			else {
				destination.setAngle(abs(oangle));
				destination.setDist(odist);
			}
		}
		else {//when obs inside rectangle
		*/	cout<<" angle to turn "<<oangle<<endl;
			//if(abs(oangle) < 15) {
			/*
				if(oangle < 0)
					destination.setAngle(-15);
				else
					destination.setAngle(15);*/
			//}
			//else
				destination.setAngle(oangle);
			destination.setDist(odist);
		//}
		destination.setType(2);

	}
	//cout<<"Destination: angle "<<destination.getAngle()<<" dist: "<<destination.getDist()<<endl;


	//plotting the current view
			char vname[50];
			sprintf(vname, "%s%d%s", "MFIS/view-",viewno,".png");
			plotObjects(vname,robsides,cv);

	//if(robsides.size() > 9) {
	//char wait;
	//cin>>wait;
	//}

	return destination;
}


//new module to find destination(have to use findGatewaysNew first)
Destination findDestination(vector<Object> cv, vector<Exit> exits, vector<Object> robsides) {
	cout<<endl<<"Inside find destination module "<<endl;
	//cout<<"current view "<<endl;
	//displayObjects(cv);

	robsides[7].setP2(-200,exits[0].mpY());
	robsides[8].setP2(200,exits[0].mpY());

	plotObjectsAndPExits("MFIS/destination-c.png",robsides,exits,cv);

	Destination destination;
	Object rdirection(0,0,0,300,1);

	//cout<<"angle "<<exits[0].getAngle()<<endl;
	if(exits.size() == 0) {	//if there's no gateways
		destination.setAngle(0);
		destination.setDist(500);
	}
	else if(abs(exits[0].getAngle()) < 2) { //means doubtful/oblique/opaque/shady gateway
		std::sort(exits.begin(),exits.end(),sortExitsA2L);
		displayExits(exits);

		//turning to the longest gateway
		Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
		//destination.setAngle(rdirection.getAngleWithLine(edirection));
		double angle=rdirection.getAngleWithLine(edirection);
		cout<<"Actual turning angle for doubtful gateway "<<angle<<endl;
		if(angle < 0 && angle > -15)
			destination.setAngle(-15);
		else if(angle > 0 && angle < 15)
			destination.setAngle(15);
		else
			destination.setAngle(angle);
		destination.setDist(0);

	}
	else {
		double pdist=robsides[7].perpendicularDistOfPoint(exits[0].X1(),exits[0].Y1());
		cout<<"pdist "<<pdist<<endl;

		if(pdist < 100) {//set secondary destination point
			double rdist1=exits[0].distP1ToPoint(0,0);
			double rdist2=exits[0].distP2ToPoint(0,0);

			cout<<"rdist1 "<<rdist1<<" rdist2 "<<rdist2<<endl;
			//which point have to avoid
			if(rdist1 < rdist2) { //true means have to avoid first exit point
				Object edirection(0,0,exits[0].X2(),exits[0].Y2(),1);
				double angle=rdirection.getAngleWithLine(edirection);
				cout<<"Actual turning angle to avoid first exit point "<<angle<<endl;
				if(abs(angle) < 15)
					destination.setAngle(-15);
				else
					destination.setAngle(-abs(angle));//sometimes it becomes +ve(e.g. @result/33

				destination.setDist(rdist1+100);
			}
			else { //have to avoid second exit point
				Object edirection(0,0,exits[0].X1(),exits[0].Y1(),1);
				//destination.setAngle(rdirection.getAngleWithLine(edirection));
				double angle=rdirection.getAngleWithLine(edirection);
				cout<<"Actual turning angle to avoid second exit point "<<angle<<endl;
				if(abs(angle) < 15)
					destination.setAngle(15);
				else
					destination.setAngle(abs(angle));
				destination.setDist(rdist2+100);
			}
			//just for plotting to view the destination point
				double dx=destination.getDist()*sin(-(PI/180)*destination.getAngle());
				double dy=destination.getDist()*cos(-(PI/180)*destination.getAngle());

				cout<<"Obstacle on the robot path "<<endl;

				Object tmp;
				tmp.set(0,0,dx,dy,1);
				robsides.push_back(tmp);
				plotObjectsAndPExits("MFIS/dview.png",robsides,exits,cv);
		}//if pdist
		else { //all clear. there's no hindrance
			Object edirection(0,0,exits[0].mpX(),exits[0].mpY(),1);
			//destination.setAngle(rdirection.getAngleWithLine(edirection));
			double angle=rdirection.getAngleWithLine(edirection);
			cout<<"Actual turning angle for no hindrance exit "<<angle<<endl;
			if(angle < 0 && angle > -15)
				destination.setAngle(-15);
			else if(angle > 0 && angle < 15)
				destination.setAngle(15);
			else
				destination.setAngle(angle);
			destination.setDist(edirection.distP2ToPoint(0,0));
		}
	}//if getAngle < 2

	cout<<"destination "<<destination.getAngle()<<" "<<destination.getDist()<<endl;
	//because of robot's mechanical error
	if(destination.getDist() > 4000) {
		destination.setDist(4000);
	}

	return destination;
}


Destination findDestinationPoint(vector<Object> cv, vector<Exit> exits) {
	//vector<double> result;
	Destination destination;
	Object rdirection(0,0,0,300,1);
	for(int i=0;i<int(exits.size());i++) {
		//cout<<exits[i].getID()<<endl;
		bool clearpath=true;
		Object exit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		//exit.display();
		Object edirection(0,0,exits[i].mpX(),exits[i].mpY(),i+1);
		cout<<"Angle with robot direction "<<rdirection.getAngleWithLine(edirection)<<" dist from rp: "<<edirection.distP2ToPoint(0,0)<<endl;
		for(int j=exits[i].getP1ID();j<exits[i].getP2ID()-1;j++) {
			Object object(0,0,cv[j].mpX(),cv[j].mpY(),cv[j].getID());
			double isintersect=checkForIntersection(object,exit);
			//intersected means beyond the exit

			if(isintersect == 0) {
				//cout<<cv[j].getID()<<" intersected"<<endl;
			//	cout<<"obstacle exist"<<endl;
				clearpath=false;
			}
			//cv[j].display();
		}
		// clear path is false when any objects in front of exit
		if(clearpath == true ) {
			cout<<"clear path to exit no "<<exits[i].getID()<<endl;

			destination.setAngle(rdirection.getAngleWithLine(edirection));
			destination.setDist(edirection.distP2ToPoint(0,0));
			break;
		}
		//

	}
	return destination;
}

vector<double> findMovementDirection(vector<Object> pexits) {
	vector<double> result;
	std::sort(pexits.begin(),pexits.end(),sortA2L);

	cout<<"Sorted PExits"<<endl;
	displayObjects(pexits);



	return result;
}

vector<Object> exitsInMFIS(vector<Exit> exits,Object rmfis, Object rcv,int refpoint) {
	vector<Object> result;
	for(int i=0;i<int(exits.size());i++) {
		Object tmpexit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object
		Object exit=remakeLineP2(rmfis,rcv,tmpexit,1,0, refpoint);
		result.push_back(exit);
	}

	return result;
}

//converts vector of exits to vector of objects
vector<Object> convertExitToObject(vector<Exit> exits) {
	vector<Object> result;
	for(int i=0;i<int(exits.size());i++) {
		Object tmpexit(exits[i].X1(),exits[i].Y1(),exits[i].X2(),exits[i].Y2(),exits[i].getID());//cnvrt from exit to object

		result.push_back(tmpexit);
	}
	return result;
}


Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno) {
    vector<Surface> surfaces(s.size());

    for (unsigned i = 0; i < s.size(); i++) {
        surfaces[i] = Surface(PointXY(s[i].X1(), s[i].Y1()), PointXY(s[i].X2(), s[i].Y2()), true);
    }

    return DestinationToGo(myPathPlanner, surfaces, viewno);
}

Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno) {
    pair<double, double> mydest = myPathPlanner->getNextDestination(s);

    return Destination(mydest.first, mydest.second);
}

////will be used to reach a goal
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno,PointXY goal) {
//    vector<Surface> surfaces(s.size());
//
//    for (unsigned i = 0; i < s.size(); i++) {
//        surfaces[i] = Surface(PointXY(s[i].X1(), s[i].Y1()), PointXY(s[i].X2(), s[i].Y2()), true);
//    }
//
//    return DestinationToGo(myPathPlanner, surfaces, viewno,goal);
//}
//
//Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno,PointXY goal) {
//    pair<double, double> mydest = myPathPlanner->getNextDestination(s,goal);
//
//    return Destination(mydest.first, mydest.second);
//}
            


//goal.first is the anlge goal.second is the distance of goal
vector<Object> findExitToReachGoal(vector<Object> currentView, pair<double,double> goal, vector<Object> referenceObjects, int viewNumber) {
    cout<<"Goal angle and Distance: ";
//    cin>>goal.first>>goal.second;
    goal.first = -45;
    goal.second = 9000;
    MyRobot myrobot(0, 0);
    vector<Object> currentRobotPosition = myrobot.getRobot();
    vector<Object> destinationExitsInCV;
    destinationExitsInCV = findAllDestinationExits(currentView, referenceObjects);
//    vector<Exit> tempExits = findExits(currentView);
//    destinationExitsInCV = convertExitToObject(tempExits);
    Object robotLocationToExitMP;
    double destinationExitDirection;//angle
    double goalDirection = goal.first;//angle
    double goalX = goal.second * sin(-goal.first);//angle in radian
    double goalY = goal.second * cos(-goal.first);
    double exitToGoalDistance;
    
    Object destinationExit;
    
    if (destinationExitsInCV.size() > 1) {
        vector<Object> exitsOnLeftSide, exitsOnRightSide;
        //sort destination exits according to their orientation angle(angle btw -x axis and rp to exit midPoint object)
        for (unsigned int i = 0; i < destinationExitsInCV.size(); i++) {
            robotLocationToExitMP.set(0, 0, destinationExitsInCV[i].mpX(), destinationExitsInCV[i].mpY(), i);
            destinationExitDirection = currentRobotPosition[8].getAngleWithLine(robotLocationToExitMP);
            destinationExitsInCV[i].setOrt(destinationExitDirection);//setting angle
            destinationExitsInCV[i].setDistance(destinationExitsInCV[i].distMPToPoint(goalX, goalY));
            if(destinationExitDirection > -90) 
                exitsOnLeftSide.push_back(destinationExitsInCV[i]);
            else
                exitsOnRightSide.push_back(destinationExitsInCV[i]);
            cout << "Angle with -x Axis: " << destinationExitDirection << endl;
        }

        if (goalDirection < -90)//true means goal is on right side
            std::sort(destinationExitsInCV.begin(), destinationExitsInCV.end(), sortA2OrtAngleR2L); //sorting from right to left
        else//means goal is on left side
            std::sort(destinationExitsInCV.begin(), destinationExitsInCV.end(), sortA2OrtAngleL2R); //sorting from left to right

        cout << "Sorted Destination Exits" << endl;
        displayObjects(destinationExitsInCV);        
        cout<<"Exits on LEFT"<<endl;
        displayObjects(exitsOnLeftSide);
        cout<<"Exits on RIGHT"<<endl;
        displayObjects(exitsOnRightSide);

        //find the closest exit to reach this goal        
        
        if(goalDirection > 0) {//true means goal is on left side
            if(exitsOnLeftSide.size() > 0) {//true means there is at least one exit on left side
                std::sort(exitsOnLeftSide.begin(),exitsOnLeftSide.end(),sortA2Distance);
                destinationExit = exitsOnLeftSide[0];
            }
            else {//true means there is no exits on left side , so pick the first one from right side 
                std::sort(exitsOnRightSide.begin(),exitsOnRightSide.end(),sortA2OrtAngleL2R);
                destinationExit = exitsOnRightSide[0];
            }
        }
        else {//true means goal is on right side
            if(exitsOnRightSide.size() > 0) {//true means these is at least one exit on right side, pick the closet to goal
                std::sort(exitsOnRightSide.begin(),exitsOnRightSide.end(),sortA2Distance);
                destinationExit = exitsOnRightSide[0];
            }
            else {//true means there is no exits on right side, so pick the first one from left side
                std::sort(exitsOnLeftSide.begin(),exitsOnLeftSide.end(),sortA2OrtAngleR2L);
                destinationExit = exitsOnLeftSide[0];
            }
        }
    } else if(destinationExitsInCV.size() == 1){//true means there is only one exit
        destinationExit = destinationExitsInCV[0];
    }
    
    Object temp;
    temp = makeLineAtPointWithObject(goal.first,goal.second,500,currentRobotPosition[6]);
    currentRobotPosition.push_back(temp);
    currentRobotPosition.push_back(destinationExit);

    char viewFileName[80];
    sprintf(viewFileName, "%s%d%s", "Maps/view-", viewNumber, ".png");
    plotObjectsOf3Kinds(viewFileName,destinationExitsInCV,currentRobotPosition,currentView);
//    waitHere();
    return destinationExitsInCV;
}

vector<Object> findGapsForGoalExit(vector<Object> currentView, 
                                                        vector<Object> referenceObjects, 
                                                        Object goalExit,
                                                        int viewNumber) {
    cout <<endl<< "Finding gaps to reach the Goal" << endl;
    vector<Object> goalGaps;

    vector<Object> finalDestination;
    finalDestination.push_back(goalExit);
    finalDestination.push_back(makeLineAtPointWithObject(-90, 0, 500, Object(goalExit.mpX(), goalExit.mpY(), goalExit.X2(), goalExit.Y2(), 1)));
    Object originToGoal = Object(0, 0, finalDestination[1].X2(), finalDestination[1].Y2());
    
    //finding destination exits    
    vector<Object> destinationExitsInCV;// = convertExitToObject(findShortestExits(currentView));
    destinationExitsInCV = findAllDestinationExits(currentView,referenceObjects);

    if (destinationExitsInCV.size() > 0) {
        cout<<"Finding nearest gap"<<endl;
        double gapToGoalDist = destinationExitsInCV[0].distMPToPoint(finalDestination[1].X2(),finalDestination[1].Y2());
        Object gapToGo = destinationExitsInCV[0];
        for (unsigned int i = 0; i < destinationExitsInCV.size(); i++) {
            if(destinationExitsInCV[i].length() > 1000)
            if(gapToGoalDist > destinationExitsInCV[i].distMPToPoint(finalDestination[1].X2(),finalDestination[1].Y2()))
                gapToGo = destinationExitsInCV[i];
        }
        //its a 90degree line on the gapToGoal
        Object goalObject = makeLineAtPointWithObject(-90, 0, 500, Object(gapToGo.mpX(), gapToGo.mpY(), gapToGo.X2(), gapToGo.Y2(), 1));
        goalObject.reverse();
        goalGaps.push_back(goalObject);
    }
    
    char viewFileName[100];
    sprintf(viewFileName, "%s%d%s", "Maps/nextDestination-", viewNumber, ".png");
    //plotObjectsOf4Kinds(viewFileName, currentView, destinationExitsInCV,goalGaps, finalDestination);

    return goalGaps;
}

pair<double,double> findCurrentGoal(vector<Object> goalGap) {
    cout<<"Finding next GapGoal"<<endl;
    MyRobot robot(0, 0);
    pair<double,double> goal;
    cout<<"angle: "<<goal.first<<" Dist: "<<goal.second<<" goalGap Size: "<<goalGap.size()<<endl;
    Object temp(0,0,goalGap[0].mpX(),goalGap[0].mpY(),1);
    
    goal.first = robot.getRobot()[6].getAngleWithLine(temp);
    goal.second = temp.length();
    
    return goal;
}

bool isThePathClear(vector<Object> currentView, Object goalExit) {
    
    bool clear = true;
    vector<Object> finalDestination;
    finalDestination.push_back(goalExit);
    finalDestination.push_back(makeLineAtPointWithObject(-90, 0, 500, Object(goalExit.mpX(), goalExit.mpY(), goalExit.X2(), goalExit.Y2(), 1)));
    Object originToGoal = Object(0, 0, finalDestination[1].X2(), finalDestination[1].Y2());
    cout<<"HERE"<<endl;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (checkForIntersection(originToGoal, currentView[i]) == 1) {
            cout << "There is obstacle(s) on the path." << endl;
            clear = false;
        }
    }
    
    return clear;
}

Object recognizeGoalExit(vector<Object> exits, Object goalExit) {
    double mpTompDistance;
    Object recognizedExitInCV;
    for(unsigned int i=0; i<exits.size(); i++) {
        mpTompDistance = goalExit.distMPToPoint(exits[i].mpX(),exits[i].mpY());
        if(mpTompDistance < 1000) {
            recognizedExitInCV = exits[i];
            recognizedExitInCV.setID(100);
            cout<<"One Exit from CV has been recognized as goalExit"<<endl;
        }
    }
    
    return recognizedExitInCV;
}

vector<Exit> findShortestGap(vector<Object> cv) {
    cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
    for (unsigned int i = 0; i < cv.size(); i++) {
        cv[i].setID(i + 1);
    }

    Exit exit;
    vector<Exit> exits;


    Point gapPoint2;
    int exits_counter = 1;
    for (int i = 0; i<int(cv.size()); i++) {
        //if (cv[i].distP2ToP1(cv[i+1]) > 600) {
        if (cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
            exit.set(cv[i].X2(), cv[i].Y2(), cv[i + 1].X1(), cv[i + 1].Y1());
            for (int j = i+1; j<int(cv.size()); j++) {
               
                gapPoint2 = cv[j].shortestDistPointFrom(cv[i].X2(), cv[i].Y2());
                if (cv[i].distP2ToPoint(gapPoint2.X(), gapPoint2.Y()) < exit.length()) {
                    exit.set(cv[i].X2(), cv[i].Y2(), gapPoint2.X(), gapPoint2.Y());
                    exit.setP1ID(cv[i].getID());
                    exit.setP2ID(cv[j].getID());
                }          

            }//for j
            if (exit.length() > 600) {//filter out small gaps
                exit.setID(exits_counter);
                exits.push_back(exit);
                exits_counter++;
            }
        }//if cv i
//        if (cv[i].getPEP1() == true) { //p2 is a probable exit end(p1)
//            exit.set(cv[i].X1(), cv[i].Y1(), cv[i + 1].X1(), cv[i + 1].Y1());
//            for (int j = i + 1; j<int(cv.size()); j++) {
//                gapPoint2 = cv[j].shortestDistPointFrom(cv[i].X2(), cv[i].Y2());
//                if (cv[i].distP2ToPoint(gapPoint2.X(), gapPoint2.Y()) < exit.length()) {
//                    exit.set(cv[i].X2(), cv[i].Y2(), gapPoint2.X(), gapPoint2.Y());
//                    exit.setP1ID(cv[i].getID());
//                    exit.setP2ID(cv[j].getID());
//                }
//
//            }//for j
//            exit.setID(exits_counter);
//            exits.push_back(exit);
//            exits_counter++;
//        }//if cv i
    }// for i


    //    cout << "shortest exits" << endl;
    //    displayExits(exits);
    return exits;
}

vector<Point> findPathToReachGoal(vector<ASR> places) {
    cout << endl << endl << "Finding Route to Reach The Exit Goal" << endl;
    vector<Point> path;
    MyRobot myRobot(0, 0);
    vector<Object> leftSideObjects, rightSideObjects;
    vector<Object> leftBoundary, rightBoundary;
    vector<Object> gapOnLeftBoundary, gapOnRightBoundary;
    vector<Object> exit;
    vector<Object> allObjects;
    vector<Object> gaps;
    vector<Object> allParalleLines;
    vector<Point> shortestPath;
    vector<Object> route;
    vector<Object> initialRoute;
    
    Object tempGap;

    char viewFileName[100];
    Object gap;
    Point gapPoint;
    for (unsigned int p = 0; p < 5; p++) {
        allObjects = places[p].getASRObjects();
        
        exit = makeSquare(places[p].getASRExit1());

        for (unsigned int i = 0; i < allObjects.size(); i++) {
            if (allObjects[i].getPos() == -1) {
                leftSideObjects.push_back(allObjects[i]);
            } else
                rightSideObjects.push_back(allObjects[i]);
        }       
        
        
        
        //two vectors(left and right) for boundary        
        //leftside
        //exit1 to p1 of first object
        leftBoundary.push_back(Object(exit[1].mpX(), exit[1].mpY(), leftSideObjects[0].X1(), leftSideObjects[0].Y1()));
        
        for (unsigned int k = 0; k < leftSideObjects.size() - 1; k++) {
            leftBoundary.push_back(leftSideObjects[k]);
            tempGap = Object(leftSideObjects[k].X2(), leftSideObjects[k].Y2(),leftSideObjects[k + 1].X1(), leftSideObjects[k + 1].Y1());
//            for(unsigned int k2=k+1;k2<leftSideObjects.size();k2++) {
//                gapPoint = leftSideObjects[k2].shortestDistPointFrom(tempGap.X1(),tempGap.Y1());
//                if( tempGap.distP1ToPoint(gapPoint.X(),gapPoint.Y())< tempGap.length()) {
//                    tempGap.setP2(gapPoint.X(),gapPoint.Y());
//                }
//            }
            
            if(tempGap.length() > 600) {      
                tempGap = Object(leftSideObjects[k].X2(), leftSideObjects[k].Y2(),leftSideObjects[k + 1].X1(), leftSideObjects[k + 1].Y1());
                gapOnLeftBoundary.push_back(tempGap);
            }
            else
            leftBoundary.push_back(tempGap);
        }
        leftBoundary.push_back(leftSideObjects[leftSideObjects.size() - 1]);
        
        //rightside
        for (unsigned int k = 0; k < rightSideObjects.size() - 1; k++) {
            rightBoundary.push_back(rightSideObjects[k]);
            tempGap = Object(rightSideObjects[k].X2(), rightSideObjects[k].Y2(),rightSideObjects[k + 1].X1(), rightSideObjects[k + 1].Y1());
//            for(unsigned int k2=k+1;k2<rightSideObjects.size();k2++) {
//                gapPoint = rightSideObjects[k2].shortestDistPointFrom(tempGap.X1(),tempGap.Y1());
//                if( tempGap.distP1ToPoint(gapPoint.X(),gapPoint.Y())< tempGap.length()) {
//                    tempGap.setP2(gapPoint.X(),gapPoint.Y());
//                }
//            }
            
            if(tempGap.length() > 600) {       
                tempGap = Object(rightSideObjects[k].X2(), rightSideObjects[k].Y2(),rightSideObjects[k + 1].X1(), rightSideObjects[k + 1].Y1());
                gapOnRightBoundary.push_back(tempGap);
            }
            else
            rightBoundary.push_back(tempGap);
        }
        rightBoundary.push_back(rightSideObjects[rightSideObjects.size() - 1]);
        
        //mpoint of last object to exit1
        rightBoundary.push_back(Object(rightSideObjects[rightSideObjects.size() - 1].X2(), rightSideObjects[rightSideObjects.size() - 1].Y2(), exit[3].mpX(), exit[3].mpY()));
        
        
        //shortest route/path following left boundary
        for(unsigned int i=0;i<leftSideObjects.size();i++) {
        allParalleLines.push_back(makeParallelObject(leftSideObjects[i],600,'right'));
        allParalleLines.back().setID(i+1);
        }
        for(unsigned int i=0;i<allParalleLines.size();i++) {
            
        }
        
        vector<Object> allObstacles;
        allObstacles = addTwoVectorsOfObjects(allObstacles,leftBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,rightBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,gapOnLeftBoundary);
        allObstacles = addTwoVectorsOfObjects(allObstacles,gapOnRightBoundary);
        
        Point lastPoint,tempPoint,nextPoint;
        shortestPath.push_back(Point(places[p].getASRExit1().mpX(),places[p].getASRExit1().mpY()));
        Object tempPath;
        int clear =1;
        //waitHere();
        lastPoint.set(exit[2].mpX(),exit[2].mpY());
//        tempPoint.set(allParalleLines[0].X1(),allParalleLines[0].Y1());
//        tempPath.set(lastPoint.X(),lastPoint.Y(),tempPoint.X(),tempPoint.Y(),1);
        shortestPath.push_back(lastPoint);
        
        //while(checkForIntersection(places[p].getASRExit2(),route.back()) != 1) {
        int forBug;
        
        
        tempPath.set(lastPoint.X(), lastPoint.Y(),places[p].getASRExit2().mpX(),places[p].getASRExit2().mpY(),1);
        clear = 1;
                for (unsigned int j = 0; j < allObstacles.size(); j++) {
                    if (checkForIntersection(tempPath, allObstacles[j]) == 1) {
                        clear = 2;
                        break;
                    }
                }
        if(clear ==1) {//true means exit is visible. 
            route.push_back(Object(lastPoint.X(),lastPoint.Y(),places[p].getASRExit2().mpX(),places[p].getASRExit2().mpY()));
        }
        else { //need to follow boundry to go to exit
        route.push_back(Object(lastPoint.X(),lastPoint.Y(),lastPoint.X(),lastPoint.Y()));
        while(checkForIntersection(route.back(),places[p].getASRExit2()) != 1 and places[p].getASRExit2().distMPToPoint(lastPoint.X(),lastPoint.Y()) > 2000) {
        //for(int ii=0;ii<10;ii++) {
            for (unsigned int i = 0; i < allParalleLines.size(); i++) {
                tempPoint.set(allParalleLines[i].X1(), allParalleLines[i].Y1());
                tempPath.set(lastPoint.X(), lastPoint.Y(), tempPoint.X(), tempPoint.Y(), 1);
                clear = 1;
                for (unsigned int j = 0; j < allObstacles.size(); j++) {
                    if (checkForIntersection(tempPath, allObstacles[j]) == 1) {
                        //lastPoint.set(allParalleLines[i-1].X1(),allParalleLines[i-1].Y1());
                        //shortestPath.push_back(lastPoint);
                        clear = 2;
                        break;
                    }
                }
                
                if (clear == 1 ) {
                    cout << " " << i;
                    nextPoint.set(allParalleLines[i].X1(), allParalleLines[i].Y1());
                    forBug = i;
                }
                

            }
            if(forBug == 55)
                nextPoint.set(allParalleLines[51].X1(), allParalleLines[51].Y1());
            route.push_back(Object(lastPoint.X(), lastPoint.Y(), nextPoint.X(), nextPoint.Y()));
            lastPoint = nextPoint;
//            cout<<"p: "<<p<<endl;
//        waitHere();
        }
        }
       
//        for(unsigned int i=0;i<shortestPath.size()-1;i++) {
//            route.push_back(Object(shortestPath[i].X(),shortestPath[i].Y(),shortestPath[i+1].X(),shortestPath[i+1].Y()));
//        }
        
        
        sprintf(viewFileName, "%s%d%s", "Maps/placeWithBoundaryAndGaps-", p, ".png");
        plotObjectsOf3Kinds(viewFileName, route, leftBoundary,rightBoundary);
        // plotObjectsOf4Kinds(viewFileName, allParalleLines,route, leftBoundary,rightBoundary);
         
        
        leftBoundary.clear();
        rightBoundary.clear();
        gapOnLeftBoundary.clear();
        gapOnRightBoundary.clear();
        shortestPath.clear();
        allParalleLines.clear();
        route.clear();
        shortestPath.clear();

        //single vector of boundary
        for (unsigned int i = 0; i < leftSideObjects.size(); i++) {
            //if(leftSideObjects[i].getPEP2() == true) {
            gap.set(leftSideObjects[i].X2(), leftSideObjects[i].Y2(), rightSideObjects[0].X1(), rightSideObjects[0].Y1(), 1);
            for (unsigned int j = 0; j < rightSideObjects.size(); j++) {
                gapPoint = rightSideObjects[j].shortestDistPointFrom(leftSideObjects[i].X2(), leftSideObjects[i].Y2());
                if (leftSideObjects[i].distP2ToPoint(gapPoint.X(), gapPoint.Y()) < gap.length()) {
                    gap.setP2(gapPoint.X(), gapPoint.Y());
                }
            }
            
//            for(unsigned int k=0;k<leftSideObjects.size();k++) {
//                gapPoint = leftSideObjects[k].shortestDistPointFrom(gap.X2(),gap.Y2());
//                if(gap.distP2ToPoint(gapPoint.X(),gapPoint.Y()))
//            }

            if (gaps.size() > 0) {
                if (gaps.back().distMPToPoint(gap.mpX(), gap.mpY()) > 3000) //filter out of close gaps
                    gaps.push_back(gap);
            } else
                gaps.push_back(gap);
            //}
        }
        
        
        sprintf(viewFileName, "%s%d%s", "Maps/placeWithGaps-", p, ".png");
        plotObjects(viewFileName, gaps, allObjects);
        gaps.clear();
        leftSideObjects.clear();
        rightSideObjects.clear();
               
    }



    //find the wall

    return path;
}


vector<Exit> findGapasExits(vector<Object> cv) 
{
    Exit exit;
    vector<Exit> exits;
    double threshold = 1500;
    double dist_PtP = 0;
    
    cout << endl << "******* Finding exits(modified version of Yeap's theory) ******* " << endl;
   for (unsigned int i = 0; i < cv.size(); i++) 
    {   
        cv[i].setID(i + 1);  
    }
 
    for (int i = 0; i < cv.size(); i++) 
    {
            if(i != cv.size() - 1)
            {
                    dist_PtP = sqrt((cv[i].X2() - cv[i+1].X1()) * (cv[i].X2() - cv[i+1].X1()) + (cv[i].Y2() - cv[i+1].Y1()) * (cv[i].Y2() - cv[i+1].Y1()));
                   //dist_PtP = cv[i].distP1ToP2(cv[i+1]);
                    if(dist_PtP >= threshold) 
                    {
                        exit.set(cv[i].X2(), cv[i].Y2(), cv[i+1].X1(), cv[i+1].Y1());
                        exit.setP1ID(cv[i].getID());
                        exit.setP2ID(cv[i+1].getID());

                        exits.push_back(exit);
                    }
            }
    }
 
    
    return exits;
}

vector<Object> BoundaryByExits(vector<Object> CurrentView)
{
            Exit exit;
            vector<Exit> exits;
            
            Object temp;
            vector<Object> temp_view;
            
            double threshold = 1500;
            double dist_PtP = 0;
            
            cout << endl << "******* Finding exits and Rebuild view with boundary ******* " << endl;
            for (unsigned int i = 0; i < CurrentView.size(); i++) 
             {   
                 CurrentView[i].setID(i + 1);  
             }
            
            for (int i = 0; i < CurrentView.size(); i++) 
            {
                    if(i != CurrentView.size() - 1)
                    {
                            dist_PtP = sqrt((CurrentView[i].X2() - CurrentView[i+1].X1()) * (CurrentView[i].X2() - CurrentView[i+1].X1()) + (CurrentView[i].Y2() - CurrentView[i+1].Y1()) * (CurrentView[i].Y2() - CurrentView[i+1].Y1()));

                            if(dist_PtP >= threshold) 
                            {
                                exit.set(CurrentView[i].X2(), CurrentView[i].Y2(), CurrentView[i+1].X1(), CurrentView[i+1].Y1());
                                exit.setP1ID(CurrentView[i].getID());
                                exit.setP2ID(CurrentView[i+1].getID());

                                exits.push_back(exit);
                            }
                    }
                 
            }
            
            for(int i = 0; i < exits.size(); i ++)
            {
                    if(i == 0 )
                        temp.set(CurrentView[0].X1(), CurrentView[0].Y1(), exits[i].X1(), exits[i].Y1(), i);
                    else
                    {
                        if(i == exits.size() - 1)
                        {
                                    temp.set(exits[i].X2(), exits[i].Y2(), CurrentView[CurrentView.size() - 1].X2(), CurrentView[CurrentView.size() - 1].Y2(), i);     
                                    temp_view.push_back(temp);
                                    temp.set(exits[i-1].X2(), exits[i-1].Y2(), exits[i].X1(), exits[i].Y1(), i);  
                                    temp_view.push_back(temp);
                                    
                        }
                        else
                            temp.set(exits[i-1].X2(), exits[i-1].Y2(), exits[i].X1(), exits[i].Y1(), i);  
                    }
                    
                    if( i != exits.size() - 1)
                         temp_view.push_back(temp);
            }
            
            
            return temp_view;
}




