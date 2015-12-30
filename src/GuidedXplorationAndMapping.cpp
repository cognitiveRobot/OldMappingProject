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




#define PI 3.14159265


using namespace std;

int main() {

    int v, w, level, set;
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
    
    bool computeASR = true;
    
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
    Object lastRouteMapNode,tempLastRouteMapNode;
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

    //reading the first view
    cout << "........Reading " << viewFileName << endl;
    vector <Object> currentView = readASCII(viewFileName);
    if(currentView.size() == 0) {
        cout<<"Need to change the file name"<<endl;
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
    
    for(unsigned int i=0;i<currentView.size();i++) {
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
    //    nextDestination.findNextDestination(currentView,v);
    //    cout<<"Angle: "<<nextDestination.getAngle()<<" Distance: "<<nextDestination.getDist()<<endl;


    //        coordTransInfo = readCoordTrans(ctFileName);
    //        cout<<"Size of CoordTransInfo: "<<coordTransInfo.size()<<endl;
    //        vector<double> distang;
    //        char coordTransFileName[100];
    //        for(int co=1;co<52;co++) {
    //            sprintf(viewFileName, "%s%d", "bin/Outdoor-AI-Students/step4/surfaces-", co);
    ////            currentView = readASCII(viewFileName);
    //       sprintf(ctFileName, "%s%d", "bin/Outdoor-AI-Students/step4/coordTrans-", co);     
    //     distang = readCoordTrans(ctFileName);
    //    
    //    sprintf(coordTransFileName, "%s%d", "bin/Outdoor-AI-Students/all/coordTrans-", co+37);
    //    writeASCII(distang, 2, coordTransFileName);
    //    distang.clear();
    //        }
    //    Object toMeasureAngle,tmpMA;
    //    toMeasureAngle.set(0,0,0,2000,1);
    //    
    //    vector<Object> compass,null;
    //    for(unsigned int i = 1;i<91;i++) {
    //    compass.push_back(makeLineAtPointWithObject(i,0,2000,toMeasureAngle));
    //    }
    //    
    //    plotObjects("Maps/compass.png",compass,null);
    //                waitHere();
    sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");


    cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;
    //    tmpMA.set(0,0,currentView[currentView.size()-1].X1(),currentView[currentView.size()-1].Y1(),1);
    //    allRobotPositions.push_back(tmpMA);
    plotObjects(mfisFileName, allRobotPositions, MFIS);
    //    waitHere();
    
    int exitCounter = 0;

    vector<Object> destinationExitsInCV, destinationExitsInMFIS, alldestinationExitsInMFIS, exitInCVToReachGoal;
    vector<pair<double, double> > invalidatedExitGoals, invalidatedGoals;
    pair<double, double> goal;
    string goalType;
    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    do {
        //Finding exit to reach a goal
        //        do{
        //        exitInCVToReachGoal = findExitToReachGoal(currentView, goal, referenceObjects,v);
        //        } while(v < 10);

        //        //finding destination exits
        //        if (goal.second == 0) {//need to find exit or longest path to go along
        //            destinationExitsInCV = findDestinationExits(currentView, referenceObjects);
        //            if (destinationExitsInCV.size() > 0) {
        //                destinationExitsInMFIS = tranformCVExitsInMFIS(destinationExitsInCV, referenceObjects);
        //                alldestinationExitsInMFIS = addTwoVectorsOfObjects(alldestinationExitsInMFIS, destinationExitsInMFIS);
        //                invalidatedExitGoals = findInvalidatedGoals(destinationExitsInCV, 1); //1 bcz these are exit goals
        //                goal = invalidatedExitGoals[0];
        //                goalType = "EXIT_GOAL";
        //            } else {
        //                invalidatedGoals = findInvalidatedGoals(currentView, 2); //2 bcz these are farthest path goals
        //                goal = invalidatedGoals[0];
        //                goalType = "NONEXIT_GOAL";
        //            }
        //        }
        //        Object tempLine = makeLineAtPointWithObject(goal.first,goal.second,myrobot.getRobot()[6]);
        //        vector<Object> tempObjects = myrobot.getRobot();
        //        //tempObjects.push_back(tempLine);
        //        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        //        plotObjectsOf3Kinds(viewFileName,tempObjects,currentView,findBoundaryLinesOfCV(currentView));
        //waitHere();
        ////        
        ////        //Avoid Obstacle
        ////        if (angle == 0 && distance == 0) {
        ////            nextDestination.findNextDestination(currentView, referenceObjects,v);
        //            nextDestination.findNextStepLocation(currentView,referenceObjects,goal,v);
        ////            nextDestination = DestinationToGo(&myPathPlanner,currentView,v);
        //            angle = nextDestination.getAngle();
        //            distance = nextDestination.getDist();
        //            if(angle == 0 && distance == 0) {//means i'm at dead end. i'm going to other exits
        ////                nextDestination.findNextDestination(currentView, referenceObjects,v);
        //                plotObjects("Maps-Autonomous/previousExits.png",alldestinationExitsInMFIS,MFIS);
        //                waitHere();                
        //            }           
        ////        }

        cout << "Destination Exits size: " << nextDestination.getOtherExitLocations().size();
        //                 waitHere();
        v++;
               if(v ==112 && set == 1) {
                    v=12;
                    set = 1018;
                    w = 140;
                }
//        if(v == 198) {
//            v = 53;
//            set =21;
//            w = 75;
//        }

        cout << "@ step " << v << endl;
        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);
        coordTransInfo = readCoordTrans(ctFileName);
        traveledDistance = traveledDistance + coordTransInfo[0];
        cout << endl << endl << "Enter 'n' to overwrite goal:(" << endl;
        cout << "                          Enter 'q' to quit exploration::((" << endl;
        //        if ( v > 42)
        //        waitHere();
        //execute
        if (abs(angle) < 10) {
            //            coordTransInfo = moveToTheDestination(robot, distance, angle, v);
            angle = 0;
            distance = 0;
        } else {
            //            coordTransInfo = moveToTheDestination(robot, 0, angle, v);
            angle = 0;
        }
        cout << "Left Angle: " << angle << " distance: " << distance << endl;

        //changing the filenames 
        sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
        //reading current view and coordinate transformation info
        cout << endl << endl << "........Reading " << viewFileName << endl;
        currentView = readASCII(viewFileName);
         if(currentView.size() == 0) {
        cout<<"Need to change the file name"<<endl;
        surfaceName = "/surfaces-";
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
        
//        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
////           if(v > 105)
//        plotObjectsAndPExits(viewFileName,myrobot.getRobot(),currentView,exitsFromCV);



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

        if(environmentType == "knowfaafan") {
            cout<<"i'm lost at @"<< v<<". But i know the ASR number"<<endl;
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
        
       

        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 3) {
            cout << "updating situation " << endl;

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints,currentRobotPositionInMFIS);

            computedOutput = updatePerceptualMap(MFIS, currentView, currentRobotPositionInMFIS, referenceObjects, v, ASRNumber);
            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            MFIS = computedOutput.getView();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end

//                        if (v > 105) {
////                                        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
////                                        plotObjects(viewFileName,myrobot.getRobot(),currentView);
//            
            sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
             plotObjects(mfisFileName, currentRobotPositionInMFIS, MFIS);
////            sprintf(mfisFileName, "%s%d%s", "Maps/MapAsTheViews-", v, ".png");
////           //  plotRobotView(mfisFileName,currentRobotPositionInMFIS,MFIS);
//                        }

            if (computeASR == true) {
                //update Current ASR
                computedOutput = updatePerceptualMap(objectOfCurrentASR, currentView, currentRobotPositionInMFIS, referenceObjects, v,ASRNumber);
                //  computedOutput = computeMap(objectOfCurrentASR, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
                objectOfCurrentASR = computedOutput.getView();
                currentASR.addLimitingPoint(currentRobotPositionInMFIS[6]);
            }
        } else
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();

        if (computeASR == true) {
            //for line of site points               
            if (lineOfSitePoint.distP1ToP1(currentRobotPositionInMFIS[6]) > 3000) {
                currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
                lineOfSitePoint = currentRobotPositionInMFIS[6];
            }
            currentASR.setASRObjects(objectOfCurrentASR);
            currentASR.updateRoute(lastLocomotion);
            perceptualMap.setCurrentASR(currentASR);



            //split current ASR if necessary
            exitsFromCVInMFIS = exitsInMFIS(exitsFromCV, referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            allExitsInMFIS = addTwoVectorsOfObjects(allExitsInMFIS, exitsFromCVInMFIS);
            if(v == 1)
                allExitsInMFIS.clear();
            if ((splitToFromNewASR(allExitsInMFIS, lastLocomotion) > -1 && v != 114 && v != 19) ){//or v == 105 or v == 158) {//&& v != 239 && v!= 163) {
                //waitHere();
//                if (v == 105) {
//                    //                cout<<"current view Size: "<<currentView.size()<<endl;
//                    crossedExit.push_back(allExitsInMFIS[0]);
//                    for (unsigned int e = 0; e < allExitsInMFIS.size(); e++) {
//                        if (checkForIntersection(lastLocomotion, allExitsInMFIS[e]) == 1)
//                            crossedExit.push_back(allExitsInMFIS[e]);
//                    }
//                } else
                    crossedExit.push_back(allExitsInMFIS[splitToFromNewASR(allExitsInMFIS, lastLocomotion)]);
                //            plotObjects("MFIS/test.png",perceptualMap.getCurrentASR().getASRObjects(),crossedExit);
                perceptualMap = splitCurrentASR(perceptualMap, crossedExit);
                
//                //route for abstraction
//                lastRouteMapNode.setP2(allExitsInMFIS[splitToFromNewASR(allExitsInMFIS, lastLocomotion)].mpX(),allExitsInMFIS[splitToFromNewASR(allExitsInMFIS, lastLocomotion)].mpX());
//                 routeMap.push_back(lastRouteMapNode);
//                lastRouteMapNode = currentRobotPositionInMFIS[6];
//                lastRouteMapNode.setP1(crossedExit.back().mpX(),crossedExit.back().mpX());

                objectOfCurrentASR = perceptualMap.getCurrentASR().getASRObjects();
                currentASR = perceptualMap.getCurrentASR();
                currentASR.clearRoute();
                allExitsInMFIS.clear();
                ASRNumber++;
                exitPoints.push_back(v); //just for printing at the end
                currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
                lineOfSitePoint = currentRobotPositionInMFIS[6];
                
                
            }
            
        }

        //relocalization using Godefroy's algorithm
        //        if(v == 113) {//have to use a module which will find objects from old ASRs in Current view(but limited y-axis range to reduce wrong recognition)
        //            cout<<"ViewNumber: "<<v<<endl;
        //            objectForPlaceRecognition = findSameObjectsFromTwoASRs(perceptualMap,perceptualMap.getAllASRs()[0],perceptualMap.getCurrentASR(),currentRobotPositionInMFIS[6],Point(1,2));
        //            waitHere();
        //        }
        cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;

        //        waitHere();
        //        if(v == 70)
        //            ASRNumber = 2;
        //        if (v == 111)
        //            v=2; 
      // if (v == 10) {
        
        cout<<endl<<"Computing route for ASR"<<endl<<endl;
//       if (v == 10 or v == 27 or v == 46 or v == 56 or v == 99 or v == 106 or v == 166 or v == 190 or v == 197) { //for level1set1 loop1
//       // if(v == 99 or v == 106 or v == 10 or v == 27 or v == 46 or v == 56) {
//            
//           //routeMap.push_back(lastRouteMapNode);
//            
//            lastRouteMapNode.setP2(crossedExit[exitCounter].mpX(), crossedExit[exitCounter].mpY());
//            
//            routeMap.push_back(lastRouteMapNode);
//            routeMapForOneASR.push_back(lastRouteMapNode);
//            routeMapForallASR.push_back(routeMapForOneASR);
//            routeMapForOneASR.clear();
//            lastRouteMapNode = currentRobotPositionInMFIS[6];
//
//            if (v == 56 or v == 166) {//special case for ASR 5 and ASR 8 of set19
//                Object tempObj;
//                tempObj = makeLineAtPointWithObject(90, 0, 500, crossedExit[exitCounter]); //left side line       
//                tempObj.reverse();
//                tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
//                lastRouteMapNode.setP1(tempObj.mpX(), tempObj.mpY());
//            } else
//                lastRouteMapNode.setP1(crossedExit[exitCounter].mpX(), crossedExit[exitCounter].mpY());
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

        if (goalType == "NONEXIT_GOAL") {
            goal.second = 0;
        } else {
            goal.second = 0;
        }

        //loopClosing using shape of ASR
        if (v == 111 && set == 1) {
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
        
//                        if ( v > 130) {
//                    sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
//                    plotObjects(mfisFileName, currentRobotPositionInMFIS, MFIS);
//                }
//        

    } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//
    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++) {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }
    //pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);
    //routeMap for current ASR
    routeMap.push_back(lastRouteMapNode);
    routeMapForOneASR.push_back(lastRouteMapNode);
    routeMapForallASR.push_back(routeMapForOneASR);
    plotObjects("Maps/currentView.png", myrobot.getRobot(), currentView);
    plotObjects("Maps/1MFIS.png", robotPositionsAtLimitingPoints, MFIS);
    plotObjects("Maps/2MFISwithoutRP.png", firstAndLastRP, MFIS);
    plotObjects("Maps/3PM.png",crossedExit, MFIS);
    plotObjects("Maps/4RouteMap.png",routeMap, MFIS);
    plotRobotView("Maps/MapAsTheViews.png", firstAndLastRP, MFIS);
    
    //overley knowledge
    //useTopoMap(perceptualMap,MFIS,crossedExit,refObjectForLoopClosing);
    
    
   // computeRouteMap(perceptualMap,routeMap,crossedExit);


    //plotObjects("Maps/ASR1.png",allExitsInMFIS,perceptualMap.getCurrentASR().getASRObjects());


    // plotSingleASR("Maps/currentASR.png",perceptualMap.getCurrentASR());
    //  makeFinalPMUsingOldASRs(perceptualMap, firstAndLastRP);
    

    cout << "Traveled Dist: " << traveledDistance << endl;
    //    plotObjects("MFIS/lastView.png",myrobot.getRobot(),currentView);


    // plotAllASR(perceptualMap.getAllASRs(),currentRobotPositionInMFIS);
    perceptualMap.setMFIS(MFIS);
//    if (computeASR == true) {
//        abstractASRs(perceptualMap.getAllASRs(),MFIS,routeMapForallASR,refObjectForLoopClosing);
//    }
     
    
   //testingFunction(perceptualMap,MFIS,currentView,currentRobotPositionInMFIS,v);
    cout<<"RouteMap size: "<<routeMap.size()<<endl;
    //keyInfoOfCompuetedPM(ASRNumber, exitPoints, lostPoints, limitingPoints);
    return 0;
}
