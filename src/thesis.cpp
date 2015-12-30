#include <vector>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "Object.H"
#include "GeometricOp.H"

#define PI 3.14159265

using namespace std;

//it reads a laser file and print in .png showing laser points, rays, and robot position

void printASingleScan() {

    MyRobot myrobot(0, 0);
    vector<Point> laserPoints = readPoints("inputData/laser-1");
    vector<Object> rays;
    for (unsigned int i = 0; i < laserPoints.size(); i++) {
        rays.push_back(Object(0, 0, laserPoints[i].X(), laserPoints[i].Y()));
    }
    plotObjectsAndPoints("Maps/AScan.png", myrobot.getRobot(), laserPoints, rays);
}

//it computes a global map using only odometric information
void odometricErrorMap(vector<Object> & errorMap, vector<Object> currentView,
        double traveledDistance, double robotFacing) {
    
    //finding robot position first
    Point robotPosition;
    double angle = robotFacing + 0.2;
    angle = angle * (PI/180); //angle in radian
    double rpx = (traveledDistance - 50) * sin(-angle);
    double rpy = (traveledDistance - 50)* cos(-angle);
    robotPosition.set(rpx, rpy);
    
    //transforming global map in to cv then adding with cv
    //plotObjects("Maps/before.png",errorMap,currentView);
    vector<Object> xGlobalMapOnCV = xformPVIntoCV(errorMap,robotPosition,angle);
    errorMap = addTwoVectorsOfObjects(xGlobalMapOnCV,currentView);
    //plotObjects("Maps/after.png",errorMap);
    //waitHere();
}

void globalMapByXformation() {
    
}
