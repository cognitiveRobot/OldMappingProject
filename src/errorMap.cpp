//============================================================================
// Name        : Algorithm3.cpp
// Author      : Hossain
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================
#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "GeometricOp.H"

#include "Point.H"
#include "Object.H"
#include "asr.H"
#include "mfisOp.H"
#include "asrOp.H"
#include "PathPlanning.H"
#include "Transporter.H"

#include "Minfo.H"
#include "Map.H"
#include "CompareASR.H"
#include "Mapping.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "PolygonChoice.h"
#include "PerceptualMapping.H"
#include "thesis.H"

#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"




#define PI 3.14159265


using namespace std;
vector<double> moveToTheDestination(ArRobot&, double distance, double angle, int viewNumber);

int main(int argc, char **argv) {

    //    if (argc < 7) {
    //        cout << endl << "Command: AutonomousXplorationAndMapping -rp /dev/ttyUSB0 -lp /dev/ttyUSB1 -li half" << endl << endl;
    //        return 0;
    //    }
    srand(getpid());

    cout << "argc: " << argc << " argv: " << argv << endl;

    Aria::init();
    ArSimpleConnector connector(&argc, argv);
    ArRobot robot;
    ArSick sick;
    if (!connector.parseArgs() || argc > 1) {
        Aria::logOptions();
        Aria::shutdown();
        Aria::exit(1);
    }
    /*
            ArKeyHandler keyHandler;
            Aria::setKeyHandler(&keyHandler);
            robot.attachKeyHandler(&keyHandler);
     */
    robot.addRangeDevice(&sick);
    // Try to connect, if we fail exit
    if (!connector.connectRobot(&robot)) {
        cout << "Could not connect to robot... exiting" << endl;
        Aria::shutdown();
        return 1;
    }
    // Turn on the motors, turn off amigobot sounds
    robot.runAsync(true);
    robot.comInt(ArCommands::ENABLE, 1);
    robot.comInt(ArCommands::SOUNDTOG, 0);
    robot.lock();
    robot.clearDirectMotion();
    robot.unlock();
    // Set up the laser
    connector.setupLaser(&sick);
    sick.runAsync();
    if (!sick.blockingConnect()) {
        cout << "Could not connect to SICK laser... exiting" << endl;
        robot.disconnect();
        Aria::shutdown();
        return 1;
    }
    ArUtil::sleep(1500);
    cout << "----------------Connected to robot and laser-------------------" << endl;

    int v; 
    v = 1;

    bool computeASR = false;

    //variables
    Transporter recognizedTargetObjects, computedOutput;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInPV, targetObjectsInCV;
    vector<Object> allTargetObjectsInPV;
    vector<Object> referenceObjects, odometricReferenceObject;
    ;
    vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
    vector<Object> previousRobotPositionInMFIS;
    vector<Object> routeMap;
    vector<Object> routeMapForOneASR;
    vector<Object> routeMapConnLP; //route map simply connecting limiting points
    vector<vector<Object> > routeMapForallASR;
    Object lastRouteMapNode, tempLastRouteMapNode;
    vector <Object> allRobotPositions;
    vector<Object> robotPositionsAtLimitingPoints;
    Transporter objectForPlaceRecognition;
    Transporter loopClosingInfo;
    Transporter lastStepInfo;
    vector<Object> refObjectForLoopClosing;
    string environmentType = "unknown";
    vector<int> lostPoints, limitingPoints, exitPoints;
    Object lineOfSitePoint;
    limitingPoints.push_back(1);
    Object lastLocomotion;
    vector<Object> wholeRoute;
    double traveledDistance = 0;
    double angleError, distanceError;

    vector<Exit> exitsFromCV;
    vector<Object> exitsFromCVInMFIS, allExitsInMFIS, crossedExit;

    vector<Object> objectOfCurrentASR;
    ASR currentASR;
    ASRNetwork perceptualMap;
    int ASRNumber = 1;

    vector<ASR> places;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;

    //for routeMap
    previousRobotPositionInMFIS = currentRobotPositionInMFIS;
    lastRouteMapNode = currentRobotPositionInMFIS[6];
    tempLastRouteMapNode = lastRouteMapNode;
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);

    //save coordinate transformation info
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);
    char coordTransFileName[100];
    char viewFileName[80], mfisFileName[80];
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", v);
    writeASCII(distang, 2, coordTransFileName);

    //scanning for the current view
    vector <Object> currentView = scanAndSaveView(sick, v);

    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);

    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

    for (unsigned int i = 0; i < currentView.size(); i++) {
        currentView[i].setASRNo(1);
    }

    //initializing MFIS
    vector<Object> MFIS = currentView;
    cout << "..........MFIS.........." << endl;
    displayObjects(MFIS);
    //waitHere();

    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(Object(-500, 0, 500, 0));
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);

	//printing first view with laser rays
    printASingleScan();

    sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
    plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);
    sprintf(viewFileName, "%s", "Maps/CurrentView.png");
    plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);

    bool exitCrossed = false;
    crossedExit.push_back(currentRobotPositionInMFIS[6]); //consider home as a exit

    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    char n = 'y';
    double angleToMove;
    double distanceToMove;
    double totalDist = 0, totalAngle = 0;
    while (n != 'n' && n != 'N') {
        v++;

        cout << endl << "How much to turn? ";
        cin >> angleToMove;
        cout << "How much to move? ";
        cin >> distanceToMove;

	totalDist = totalDist + distanceToMove;
	totalAngle = totalAngle + angleToMove;
        cout << "Now @ step " << v << endl;
        coordTransInfo = moveToTheDestination(robot, distanceToMove, angleToMove, v);
        traveledDistance = traveledDistance + coordTransInfo[0];

        //scanning for the current view
        currentView = scanAndSaveView(sick, v);
	//finding target objects
        targetObjectsInCV = findTargetObjects(currentView);

        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;


        //saving current view
        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);
        sprintf(viewFileName, "%s", "Maps/CurrentView.png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);

	odometricErrorMap(MFIS, currentView, distanceToMove, angleToMove);

        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
        plotObjects(mfisFileName, currentRobotPositionInMFIS, MFIS);
        sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
        plotObjects(mfisFileName, allRobotPositions, MFIS);

        cout << "Take another step? (y/n) ";
        cin >> n;
    }

    cout << "Traveled Dist: " << traveledDistance << endl;
    if (n == 'n' || n == 'N') {
        robot.disconnect();
        Aria::shutdown();
    }
    return 0;
}

vector<double> moveToTheDestination(ArRobot& robot, double distance, double angle, int viewNumber) {

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //save obometer and orientation angle before moving       
    robot.lock();
    double oldDistance = robot.getOdometerDistance();
    double oldAngle = robot.getTh();
    robot.unlock();

    //movement execution
    setHeading(robot, angle);
    moveDistance(robot, distance);
    angle = 0;

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //get the actual distance traveled for this step
    robot.lock();
    double traveledDistance = robot.getOdometerDistance() - oldDistance;
    double turnedAngle = robot.getTh() - oldAngle;
    robot.unlock();

    vector<double> distang;
    distang.push_back(traveledDistance); //rdist);
    distang.push_back(turnedAngle); //rangle);
    char coordTransFileName[100];
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", viewNumber);
    writeASCII(distang, 2, coordTransFileName);

    cout << "Traveled Dist: " << traveledDistance << " turned Angle: " << turnedAngle << endl;
    return distang;
}
