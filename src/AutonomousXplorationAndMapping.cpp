
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

#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"

#include "convexPathPlanner.h"
#include "PolygonChoice.h"

#define PI 3.14159265
using namespace std;

vector<double> moveToTheDestination(ArRobot&,double distance, double angle, int viewNumber);

int main(int argc, char **argv) {

    if (argc < 7) {
        cout << endl << "Command: AutonomousXplorationAndMapping -rp /dev/ttyUSB0 -lp /dev/ttyUSB1 -li half" << endl << endl;
        return 0;
    }
    srand(getpid());

    cout<<"argc: "<<argc<<" argv: "<<argv<<endl;
    
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
    
    int level,v;
    v=1;
    level =1;

    //variables
    Transporter recognizedTargetObjects, computedOutput;
    vector <Object> currentView;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInPV, targetObjectsInCV;
    vector<Object> referenceObjects;
    vector<Object> currentRobotPositionInMFIS;
    vector <Object> allRobotPositions;
    Transporter objectForPlaceRecognition;
    vector<int> lostPoints, limitingPoints, exitPoints;
    Object lineOfSitePoint;
    limitingPoints.push_back(1);
    Object lastLocomotion;
    
    vector<Exit> exitsFromCV;
    vector<Object> exitsFromCVInMFIS,allExitsInMFIS,crossedExit;
       
    vector<Object> objectOfCurrentASR;
    ASR currentASR;
    ASRNetwork perceptualMap;
    int ASRNumber = 1;

    MyRobot myrobot(0, 0);
    currentRobotPositionInMFIS = myrobot.getRobot();
    allRobotPositions = currentRobotPositionInMFIS;
    
    //scan for the current view
    currentView = scanAndSaveView(sick,v);
    
    //save coordinate transformation info
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);
    char coordTransFileName[100];
    sprintf(coordTransFileName, "%s%d", "bin/coordTrans-", v);
    writeASCII(distang, 2, coordTransFileName);

    char viewFileName[80], mfisFileName[80];
//    //reading the first view
//    cout << "........Reading " << viewFileName << endl;
//    currentView = readASCII(viewFileName);

    
    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
        referenceObjects.push_back(targetObjectsInPV[0]);//bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);


    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, 114);

    //initializing MFIS
    vector<Object> MFIS = currentView;
    
    //initializing Perceptual Map
    objectOfCurrentASR = currentView;
    currentASR.setASRObjects(objectOfCurrentASR);
    currentASR.setASRExit1(currentRobotPositionInMFIS[6]);
    currentASR.setASRID(1);
    lineOfSitePoint = currentRobotPositionInMFIS[6];
    currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
    perceptualMap.setCurrentASR(currentASR);

    plotObjects("Maps-Autonomous/MFIS-1.png", allRobotPositions, MFIS);
    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    
//    ConvexRobotPathPlanner myPathPlanner(5, 400);//2nd arg.
//    PointXY goal(0,1200);
    Destination nextDestination(0,0);
    double angle=0;
    double distance=0;
    char m = 'y';
    
    vector<Object> destinationExitsInCV, destinationExitsInMFIS;
    vector<pair<double,double> > invalidatedExitGoals,invalidatedGoals;
    pair<double,double> goal;
    
    while(true) {        
        
        
        //finding destination exits
        destinationExitsInCV = findDestinationExits(currentView, referenceObjects);
        if (destinationExitsInCV.size() > 0) {
            destinationExitsInMFIS = tranformCVExitsInMFIS(destinationExitsInCV, referenceObjects);
             invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV,1);//1 bcz these are exit goals
            goal = invalidatedExitGoals[0];
        }
        else {
            invalidatedGoals = findInvalidatedGoals(currentView, 2); //2 bcz these are farthest path goals
            goal = invalidatedGoals[0];
        }
        
        //find next destination
        if (angle == 0 && distance == 0) {
//            nextDestination.findNextDestination(currentView, referenceObjects,v);
            nextDestination.findNextStepLocation(currentView,referenceObjects,goal,v);
//            nextDestination = DestinationToGo(&myPathPlanner,currentView,v);
            angle = nextDestination.getAngle();
            distance = nextDestination.getDist();
            if(angle == 0 && distance == 0) {//means i'm at dead end. i'm going to other exits
//                nextDestination.findNextDestination(currentView, referenceObjects,v);
                waitHere();
            }
            
            cout<<"(Next Destination) Angle: "<<angle<<" Dist: "<<distance<<endl;
        }               
        
        //call same find nextStepLocation function to make sure path is obstacle free

        v++;
        //execute
        cout <<endl<<endl<< "I'm going at Angle: " << angle << " Distance: " << distance << endl;
        cout << "Enter 'n' to overwrite goal:(" << endl;
        cout<<"         Enter 'q' to quit exploration::(("<<endl;
        cin >> m;
        if (m == 'n') {
            cout << "Angle ";
            cin >> angle;
            cout << " distance ";
            cin >> distance;
        }
        if(m == 'q') {
            robot.disconnect();
            Aria::shutdown();
            break;
        }

        if (abs(angle) > 60) {//distance will not be more than 1000..never..
            if(angle > 0) {
                coordTransInfo = moveToTheDestination(robot, 0, 60, v);
                angle = angle - 60;
            } else {
                coordTransInfo = moveToTheDestination(robot, 0, - 60, v);
                angle = angle + 60;
            }
            
        } else if (abs(angle) < 60 && abs(angle) > 10){
            coordTransInfo = moveToTheDestination(robot, 0, angle, v);
            angle = 0;
        }
        else {
            coordTransInfo = moveToTheDestination(robot, distance, angle, v);
            angle = 0;
            distance = 0;
        }

        //scanning for the current view
        currentView = scanAndSaveView(sick,v);

        //reading current view and coordinate transformation info
//        cout << endl << endl << "........Reading " << viewFileName << endl;
//        currentView = readASCII(viewFileName);
//        coordTransInfo = readCoordTrans(ctFileName);
        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;
        
        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo,v);

        cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
        cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;

        //if it's a lost situation then use odometry info
        if (recognizedTargetObjects.getReferenceObjects().size() == 0) {
            Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6],1);
            aLine.setKP(1);
            referenceObjects.clear();
            referenceObjects.push_back(aLine);
            referenceObjects.push_back(allRobotPositions[6]);
            lostPoints.push_back(v);//just for printing at the end
        } else
            referenceObjects = recognizedTargetObjects.getReferenceObjects();

        //localization 
        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
        lastLocomotion.set(allRobotPositions.back().X1(),allRobotPositions.back().Y1(),currentRobotPositionInMFIS.back().X1(),currentRobotPositionInMFIS.back().Y1(),1);
        allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
        
        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 3) {
            cout << "updating situation " << endl;
            computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            MFIS = computedOutput.getView();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v);//limiting/updating points just for printing at the end
            
            computedOutput = computeMap(objectOfCurrentASR, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            objectOfCurrentASR = computedOutput.getView();
            currentASR.addLimitingPoint(currentRobotPositionInMFIS[6]);
        } else
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
        //for line of site points
        if(lineOfSitePoint.distP1ToP1(currentRobotPositionInMFIS[6]) > 3000) {
            currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
            lineOfSitePoint = currentRobotPositionInMFIS[6];
        }
        currentASR.setASRObjects(objectOfCurrentASR);
        currentASR.updateRoute(lastLocomotion);
        perceptualMap.setCurrentASR(currentASR);
        
        
        //split current ASR if necessary
        exitsFromCVInMFIS = exitsInMFIS(exitsFromCV, referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
        allExitsInMFIS = addTwoVectorsOfObjects(allExitsInMFIS,exitsFromCVInMFIS);
        if (splitToFromNewASR(allExitsInMFIS,lastLocomotion) > -1 ) {//&& v != 239 && v!= 163) {
            crossedExit.push_back(allExitsInMFIS[splitToFromNewASR(allExitsInMFIS,lastLocomotion)]);
            perceptualMap = splitCurrentASR(perceptualMap, crossedExit);
            
            objectOfCurrentASR = perceptualMap.getCurrentASR().getASRObjects();
            currentASR = perceptualMap.getCurrentASR();
            currentASR.clearRoute();
            allExitsInMFIS.clear();
            ASRNumber++;
            exitPoints.push_back(v);//just for printing at the end
            currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
            lineOfSitePoint = currentRobotPositionInMFIS[6];
        }

        if(v == 92) {//have to use a module which will find objects from old ASRs in Current view(but limited y-axis range to reduce wrong recognition)
            objectForPlaceRecognition = findSameObjectsFromTwoASRs(perceptualMap,perceptualMap.getAllASRs()[0],perceptualMap.getCurrentASR(),currentRobotPositionInMFIS[6],Point(1,2));
        }
        cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
//      if (v == 100) {
//            vector<Object> tmpMFIS;
//            for (int i = 0; i<int(MFIS.size()); i++)
//                if (MFIS[i].getOoPV() == true)
//                    tmpMFIS.push_back(MFIS[i]);
//            MFIS = tmpMFIS;
//            allRobotPositions = myrobot.getRobot();
//        }
        
//        if (v > 100)
//            plotObjects(mfisFileName, allRobotPositions, MFIS);
                sprintf(viewFileName, "%s%d%s", "Maps-Autonomous/view-", v, ".png");
                plotObjects(viewFileName,myrobot.getRobot(),currentView);
//                }
                sprintf(mfisFileName, "%s%d%s", "Maps-Autonomous/MFIS-", v, ".png");
                plotObjects(mfisFileName, allRobotPositions, MFIS);
        //MFIS.push_back(lastLocomotion);
        
    }  //  while(n == 'y' || n == 'Y');//
    plotObjects("Maps-Autonomous/MFIS.png", allRobotPositions, MFIS);
     
    //keyInfoOfCompuetedPM(ASRNumber,exitPoints,lostPoints,limitingPoints);
//    plotSingleASR("MFIS/ASR-15.png",perceptualMap.getCurrentASR());
//    makeFinalPMUsingOldASRs(perceptualMap);
//    testingFunction(perceptualMap);
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
    sprintf(coordTransFileName, "%s%d", "bin/coordTrans-", viewNumber);
    writeASCII(distang, 2, coordTransFileName);
    
    cout<<"Traveled Dist: "<<traveledDistance<<" turned Angle: "<<turnedAngle<<endl;
    return distang; 
}