
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

//#include "Aria.h"
//#include "RobotFuncs.H"
//#include "Laser2Surface.H"




#define PI 3.14159265


using namespace std;


int main() {
    

    int v, w, level, set;
    //for offline
    cout << "Which level you would like to map" << endl
            << "'0' for groudlevel" << endl
            << "'1' for firstlevel " << endl
            << "set 1 -- 999 for Guided xploration" << endl
            << "set  1001 --  for autonomous xploration" << endl << endl;
    level = 1; //cin >> level;
    cout << "Which Dataset ??? ";
    cin >> set;
    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
    cin >> v;
    cin >> w;

    bool computeASR = false;

    //variables
    Transporter recognizedTargetObjects, computedOutput;
    vector <double> coordTransInfo;
    vector<Object> targetObjectsInPV, targetObjectsInCV;
    vector<Object> allTargetObjectsInPV;
    vector<Object> referenceObjects;
    vector<Object> currentRobotPositionInMFIS;
    vector<Object> previousRobotPositionInMFIS;
    vector<Object> routeMap;
    vector<Object> routeMapForOneASR;
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

    const char* levelName = "bin/level";
    const char* surfaceName = "/surfaces-";
    char viewFileName[80], mfisFileName[80], ctFileName[80];
    sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
    //    cout<<viewFileName<<endl;
    //    waitHere();
    //    sprintf(viewFileName, "%s%d", "/media/DATA/GobesoNa/UbuntuWorkSpace/CognitiveMapping/surfaces-", v);

    vector <Object> currentView;
    
//    //for autonomous
//        //scan for the current view
//    currentView = scanAndSaveView(sick,v);
//        //save coordinate transformation info
//    vector<double> distang;
//    distang.push_back(0); //rdist);
//    distang.push_back(0); //rangle);
//    char coordTransFileName[100];
//    sprintf(coordTransFileName, "%s%d", "bin/coordTrans-", v);
//    writeASCII(distang, 2, coordTransFileName);
    
    //for offline mapping
    //reading the first view
    cout << "........Reading " << viewFileName << endl;
    currentView = readASCII(viewFileName);
    if (currentView.size() == 0) {
        cout << "Need to change the file name" << endl;
        surfaceName = "/surface-";
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        currentView = readASCII(viewFileName);
    }

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

    //    ConvexRobotPathPlanner myPathPlanner(5, 400);//2nd arg.
    //    PointXY goal(0,1200);
    Destination nextDestination;
    double angle = 0;
    double distance = 0;

    sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
    
    //for summer project test
    for(unsigned int i=0;i<currentView.size();i++){
    allRobotPositions.push_back(Object(0,0,currentView[i].X2(),currentView[i].Y2()));
    cout<<i<<" Dist: "<<Object(0,0,currentView[i].X2(),currentView[i].Y2()).length()<<endl;
    }
    cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;
    //    tmpMA.set(0,0,currentView[currentView.size()-1].X1(),currentView[currentView.size()-1].Y1(),1);
    //    allRobotPositions.push_back(tmpMA);
    plotObjects(mfisFileName, allRobotPositions, MFIS);
       waitHere();

    int exitCounter = 0;
    bool exitCrossed = false;
    crossedExit.push_back(Object(-500, 0, 500, 0)); //consider home as a exit

    vector<Object> destinationExitsInCV, destinationExitsInMFIS, alldestinationExitsInMFIS, exitInCVToReachGoal;
    vector<pair<double, double> > invalidatedExitGoals, invalidatedGoals;
    pair<double, double> goal;
    Object goalExit,goalExitInCV;
    goalExit = makeLineAtPointWithObject(-17.5073,15110.9,1000,currentRobotPositionInMFIS[6]);
    string goalType;
    vector<Object> gapGoal;
    double gapToFinalGoalDistance;
    char m;
    bool IsGoalExitSpotted = false;
    
    
    
    
//    Angle: -17.5073 Distance: 15110.9
//Angle: 349.892 Distance: 7871.59
//Angle: 2.5031 Distance: 9369.95
//Angle: 225.071 Distance: 23512.1
//Angle: 26.9252 Distance: 4130.78

    //plotObjectsOf3Kinds("Maps/view-0.png",currentView,currentRobotPositionInMFIS,makeSquare(goalExit));
    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    do {

       if(IsGoalExitSpotted == true && angle == 0 && distance == 0) {
            cout<<endl<<endl<<"I reached the goal. I'm going to cross this exit."<<endl<<endl<<endl;
        }

       cout<<"ref point: "<<referenceObjects[0].getKP()<<endl;
        goalExitInCV = remakeLineP2(referenceObjects[1], referenceObjects[0], goalExit, 1, 0, referenceObjects[0].getKP());
        cout<<"HERE"<<endl;
        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        //plotObjectsOf4Kinds(viewFileName,currentView,myrobot.getRobot(),makeRectangle(goalExitInCV),convertExitToObject(exitsFromCV));
        
//        if (IsGoalExitSpotted == false && isThePathClear(currentView, goalExitInCV) == true) {
//            goal = findCurrentGoal(gapGoal);
//            angle = goal.first;
//            distance = goal.second;
//
//            IsGoalExitSpotted = true;
//            cout<<"Path to reach our goal is clear."<<endl;
//        }
//        if (IsGoalExitSpotted == false) {
//            if (angle == 0 && distance == 0) {
//                cout<<"Find Gaps"<<endl;
//                gapGoal = findGapsForGoalExit(currentView, referenceObjects, goalExitInCV, v);
//                if (gapGoal.size() == 0) {//go to max path
//                    cout << "There is no destination exits. Need to move towards a gap or somewhere else." << endl;
//                    invalidatedGoals = findInvalidatedGoals(currentView, 2); //2 bcz these are farthest path goals
//                    angle = invalidatedGoals[0].first;
//                    distance = invalidatedGoals[0].second;
//                } else {
//                    goal = findCurrentGoal(gapGoal);
//                    angle = goal.first;
//                    distance = goal.second;
//
//                    gapToFinalGoalDistance = gapGoal[0].distMPToPoint(goalExitInCV.mpX(), goalExitInCV.mpY());
//                }
//            } else {//check whether these is new gapGoal close to Final Goal than previous one
//                gapGoal = findGapsForGoalExit(currentView, referenceObjects, goalExitInCV, v);
//                if (gapGoal.size() > 0) {
//                    if (gapToFinalGoalDistance > gapGoal[0].distMPToPoint(goalExitInCV.mpX(), goalExitInCV.mpY())) {//new gapGoal is better. Follow this one.
//                        goal = findCurrentGoal(gapGoal);
//                        angle = goal.first;
//                        distance = goal.second;
//
//                        gapToFinalGoalDistance = gapGoal[0].distMPToPoint(goalExitInCV.mpX(), goalExitInCV.mpY());
//                        cout<<"Found better gap to reach the GOAL:::)"<<endl;
//                    }
//                }
//            }
//        }
        //waitHere();
        v++;
//        //execute
//        cout <<endl<<endl<< "I'm going at Angle: " << angle << " Distance: " << distance << endl;
//        cout << "Enter 'n' to overwrite goal:(" << endl;
//        cout<<"         Enter 'q' to quit exploration::(("<<endl;
//        cin >> m;
//        if (m == 'n') {
//            cout << "Angle ";
//            cin >> angle;
//            cout << " distance ";
//            cin >> distance;
//        }
//        if(m == 'q') {
//            robot.disconnect();
//            Aria::shutdown();
//            break;
//        }
//
//        if (abs(angle) > 60) {//distance will not be more than 1000..never..
//            if(angle > 0) {
//                coordTransInfo = moveToTheDestination(robot, 0, 60, v);
//                angle = angle - 60;
//            } else {
//                coordTransInfo = moveToTheDestination(robot, 0, - 60, v);
//                angle = angle + 60;
//            }
//            
//        } else if (abs(angle) < 60 && abs(angle) > 10){
//            coordTransInfo = moveToTheDestination(robot, 0, angle, v);
//            angle = 0;
//        }
//        else {
//            coordTransInfo = moveToTheDestination(robot, 1000, angle, v);
//            angle = 0;
//            distance = distance - 1000;
//        }
        
        
//        //scanning for the current view
//        currentView = scanAndSaveView(sick,v);
        
        
        //for exp3
//        if (v == 112 && set == 1) {
//            v = 12;
//            set = 1018;
//            w =137;
//        }
        
        //
        
//        //for exp2
//        if (v == 107) {
//            v = 128;
//            //set = 1018;
//            w =200;
//        }
//        if (v == 198) {
//            v = 53;
//            set = 21;
//            w = 75;
//        }
        
        
        if(v == 118 && set == 1) {
            v = 1;
            set = 2222;
            w = 122;
        }
        
        if(v == 22 && set == 2222) {
            v = 40;
            w=122;
        }

        //for offline
        cout << "@ step " << v << endl;
        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
        coordTransInfo = readCoordTrans(ctFileName);
        traveledDistance = traveledDistance + coordTransInfo[0];

        //changing the filenames 
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        //reading current view and coordinate transformation info
        cout << endl << endl << "........Reading " << viewFileName << endl;
        currentView = readASCII(viewFileName);
        if (currentView.size() == 0) {
            cout << "Need to change the file name" << endl;
            surfaceName = "/surface-";
            sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
            currentView = readASCII(viewFileName);
        }
        //recognizeViews(MFIS,currentView);

        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;

        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);
        
//        cout<<"Current View"<<endl;
//        displayObjects(currentView);
//        waitHere();

//                sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
                   //if(v > 105)
//                plotObjectsAndPExits(viewFileName,myrobot.getRobot(),currentView,findExits(currentView));

                //waitHere();

        if (v == 128) {//to ignor last two objects of this current view bcz replacing steps isnt perfect. overlaps with other lines. need to change.
            vector<Object> tmpcv;
            for (unsigned int ci = 0; ci < (currentView.size() - 2); ci++) {
                tmpcv.push_back(currentView[ci]);
            }
            currentView = tmpcv;
        }

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);

        cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
        cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;

        if (environmentType == "knowfaafan") {
            cout << "i'm lost at @" << v << ". But i know the ASR number" << endl;
            updateMapAtLastStep(lastStepInfo);
            waitHere();
        }

        //if it's a lost situation then use odometry info
        if (recognizedTargetObjects.getReferenceObjects().size() == 0) {
            //currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
            aLine.setKP(1);
            referenceObjects.clear();
            referenceObjects.push_back(aLine);
            referenceObjects.push_back(allRobotPositions[6]);
            lostPoints.push_back(v); //just for printing at the end
        } else
            referenceObjects = recognizedTargetObjects.getReferenceObjects();

        //        if(v > 100) 
        //            ASRNumber = 3;

        //localization 
        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
        lastLocomotion.set(allRobotPositions.back().X1(), allRobotPositions.back().Y1(),
                currentRobotPositionInMFIS.back().X1(), currentRobotPositionInMFIS.back().Y1(), 1);

        wholeRoute.push_back(lastLocomotion);
        allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
        cout << "Current robot position " << endl;
        currentRobotPositionInMFIS[7].display();


        //check whether exit is crossed
        exitsFromCVInMFIS = exitsInMFIS(exitsFromCV, referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
        allExitsInMFIS = addTwoVectorsOfObjects(allExitsInMFIS, exitsFromCVInMFIS);

        if(v == 114)
            allExitsInMFIS.clear();

        if ((splitToFromNewASR(allExitsInMFIS, lastLocomotion) > -1 )){//or v == 197) && v != 53 && v != 100 && v != 114 && v != 165 && v != 166 && v != 191 && v != 192 && v != 193) {
            exitCrossed = true;
            crossedExit.push_back(allExitsInMFIS[splitToFromNewASR(allExitsInMFIS, lastLocomotion)]);
            ASRNumber++;
            exitPoints.push_back(v); //just for printing at the end
            allExitsInMFIS.clear();
        }

        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 3 or exitCrossed == true or v == 108) {
            cout << "updating situation " << endl;

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);

            computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS, allRobotPositions,referenceObjects,
                    v, ASRNumber, exitCrossed, crossedExit);
            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            MFIS = computedOutput.getView();
            places = computedOutput.getASRs();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end

            exitCrossed = false;

        } else
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();

        cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;

        cout << endl << "Computing route for ASR" << endl << endl;
//        if (v == 10 or v == 27 or v == 46 or v == 56 or v == 99 or v == 106 or v == 166 or v == 190 or v == 197) { //for level1set1 loop1
//            // if(v == 99 or v == 106 or v == 10 or v == 27 or v == 46 or v == 56) {
//            lastRouteMapNode.setP2(crossedExit.back().mpX(), crossedExit.back().mpY());
//
//            routeMap.push_back(lastRouteMapNode);
//            routeMapForOneASR.push_back(lastRouteMapNode);
//            routeMapForallASR.push_back(routeMapForOneASR);
//            routeMapForOneASR.clear();
//            lastRouteMapNode = currentRobotPositionInMFIS[6];
//
//            if (v == 56 or v == 166) {//special case for ASR 5 and ASR 8 of set19
//                Object tempObj;
//                tempObj = makeLineAtPointWithObject(90, 0, 500, crossedExit.back()); //left side line       
//                tempObj.reverse();
//                tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
//                //tempObj.setP1(tempObj.mpX(),tempObj.mpY());
//                //tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj);
//                lastRouteMapNode.setP1(tempObj.mpX(), tempObj.mpY());
//            } else
//                lastRouteMapNode.setP1(crossedExit.back().mpX(), crossedExit.back().mpY());
//
//
//            exitCounter++;
//        } else {
//            //for route map
//            lastRouteMapNode.setP2(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1());
//            if (abs(lastRouteMapNode.getAngleWithLine(currentRobotPositionInMFIS[6])) > 40 && lastRouteMapNode.distP1ToP1(currentRobotPositionInMFIS[6]) > 3000) {
//
//                routeMap.push_back(lastRouteMapNode);
//                routeMapForOneASR.push_back(lastRouteMapNode);
//                lastRouteMapNode = currentRobotPositionInMFIS[6];
//            }
//        }

        //loopClosing using shape of ASR
     //   if (v == 64 && set == 21) {
        if(v == 111 && set == 1) {
        //if(v == 197) {
            perceptualMap.setASRs(places);
            loopClosingInfo = recognizeThisPlace(perceptualMap, MFIS, currentView, currentRobotPositionInMFIS, referenceObjects, v);
            refObjectForLoopClosing = loopClosingInfo.getReferenceObjectsForLoopClosing();
            //targetObjectsInPV = loopClosingInfo.getTargetObjects();
            //MFIS = loopClosingInfo.getView();
            //plotObjects("Maps/MFISafterLoopClosing.png",MFIS,projectingTheView(perceptualMap.getASRs()[0].getASRObjects(),refObjectForLoopClosing[1],refObjectForLoopClosing[0],1));
            environmentType = "known";
            //waitHere();
        }

        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(loopClosingInfo.getView());
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setReferenceObjects(referenceObjects);

//                if (v > 80) {
//                    sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
//                    plotRobotView(mfisFileName, currentRobotPositionInMFIS, MFIS);
//                }


    } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//
    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++) {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }
    plotPerceptualMap("Maps/1PerceptualMap.png", allRobotPositions, MFIS, ASRNumber);
    //waitHere();
    //routeMap for current ASR
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);
    routeMapForallASR.push_back(routeMapForOneASR);
    //    plotObjects("Maps/currentView.png", myrobot.getRobot(), currentView);
    //    plotObjects("Maps/1MFIS.png", robotPositionsAtLimitingPoints, MFIS);
    //    plotObjects("Maps/2MFISwithoutRP.png", firstAndLastRP, MFIS);
        plotObjects("Maps/3PM.png", crossedExit, MFIS);
    //    plotObjects("Maps/4RouteMap.png", routeMap, MFIS);
    //    plotRobotView("Maps/MapAsTheViews.png", firstAndLastRP, MFIS);

    //overley knowledge
   // if (environmentType == "knownnn")
        //useTopoMap(perceptualMap, MFIS, crossedExit, refObjectForLoopClosing);

    cout << "Traveled Dist: " << traveledDistance << endl;

    perceptualMap.setMFIS(MFIS);
    // if (computeASR == true) {
    // abstractASRs(places,MFIS,routeMapForallASR,refObjectForLoopClosing);
     //boundaryOfThePlace(places,MFIS,routeMapForallASR,refObjectForLoopClosing,crossedExit);
     findPathToReachGoal(places);
    //}
    //  pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);
   // keyInfoOfCompuetedPM(ASRNumber, exitPoints, lostPoints, limitingPoints);
    return 0;
}


