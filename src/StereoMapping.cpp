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

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;


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
    vector<Object> referenceObjects;
    vector<Object> currentRobotPositionInMFIS;
    vector <Object> allRobotPositions;
    vector<Object> robotPositionsAtLimitingPoints;
    Transporter objectForPlaceRecognition;
    vector<int> lostPoints, limitingPoints, exitPoints;
    Object lineOfSitePoint;
    limitingPoints.push_back(1);
    Object lastLocomotion;
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

    const char* levelName = "bin/level";
    //const char* surfaceName = "/surfaces-";
    const char* surfaceName = "/points-";
    char viewFileName[80], mfisFileName[80], ctFileName[80];
    sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
    //    cout<<viewFileName<<endl;
    //    waitHere();
    //    sprintf(viewFileName, "%s%d", "/media/DATA/GobesoNa/UbuntuWorkSpace/CognitiveMapping/surfaces-", v);

    //reading the first view
    cout << "........Reading " << viewFileName << endl;
    vector <Object> currentView;// = readASCII(viewFileName);
    //vector <Point> points;
    currentView = makeViewFromPoints(set,v);
    
    //Hough line detector
   Mat src = imread("Maps/Points-1.png", 0);
   imshow("source", src);
   Mat dst, cdst;
 Canny(src, dst, 50, 200, 3);
 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
 #else
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point2f(l[0], l[1]), Point2f(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
 #endif
 cvtColor(dst, cdst, CV_GRAY2BGR);
 imshow("detected lines", cdst);
   waitKey(0);
    
    //waitHere();

    //finding target objects
    targetObjectsInPV = findTargetObjects(currentView);
    referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
    referenceObjects.push_back(targetObjectsInPV[0]);
    
    //finding shortest Exits in first View
    vector<Exit> shortestExitsInPV = findShortestExits(currentView);
    
    
    vector<Object> exitMFIS = convertExitToObject(shortestExitsInPV);
    vector<Object> targetExitsInPV = exitMFIS;
    
    vector<Object> targetExitsInCV;
    
    
    //tagging side and view number
    currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

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
    
    vector<Object> localizedRPUsingExits;
    


    //    ConvexRobotPathPlanner myPathPlanner(5, 400);//2nd arg.
    //    PointXY goal(0,1200);
    Destination nextDestination;
    double angle = 0;
    double distance = 0;
    
    sprintf(mfisFileName, "%s%d%s", "Maps/view-", v, ".png");

    vector<Object> exitAsReference;
    exitAsReference.push_back(convertExitToObject(shortestExitsInPV)[0]);
    exitAsReference.back().setKP(1);

    cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;
    //    tmpMA.set(0,0,currentView[currentView.size()-1].X1(),currentView[currentView.size()-1].Y1(),1);
    //    allRobotPositions.push_back(tmpMA);
    //plotObjectsAndPExits(mfisFileName, allRobotPositions, MFIS,shortestExitsInPV);
    plotObjects(mfisFileName,myrobot.getRobot(),currentView);
      waitHere();

    vector<Object> destinationExitsInCV, destinationExitsInMFIS, alldestinationExitsInMFIS, exitInCVToReachGoal;
    vector<pair<double, double> > invalidatedExitGoals, invalidatedGoals;
    pair<double, double> goal;
    string goalType;
    cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
    do {
       

        cout << "Destination Exits size: " << nextDestination.getOtherExitLocations().size();
        //                 waitHere();
        v ++;
//               if(v == 57 && set == 13) {
//                    v=16;
//                    set = 14;
//                    w = 90;
//                }

        cout << "@ step " << v << endl;
        sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", 1);
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
        //currentView = readASCII(viewFileName);
        currentView = makeViewFromPoints(set,v);
        //points = readPoints(viewFileName);
        //recognizeViews(MFIS,currentView);

        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;


        
        //vector<Object> newRP = updateUsingAveragePoint(MFIS,currentView);
        //waitHere();

        
        
        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        
        targetExitsInCV = convertExitToObject(exitsFromCV);
        //targetExitsInCV = currentView;
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);
        
        exitAsReference.push_back(convertExitToObject(exitsFromCV)[0]);
        
        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        //plotObjectsAndPoints(viewFileName,myrobot.getRobot(),currentView,points);



        if (v == 128) {//to ignor last two objects of this current view bcz replacing steps isnt perfect. overlaps with other lines. need to change.
            vector<Object> tmpcv;
            for (unsigned int ci = 0; ci < (currentView.size() - 2); ci++) {
                tmpcv.push_back(currentView[ci]);
            }
            currentView = tmpcv;
        }

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetExits(exitMFIS,targetExitsInPV,targetExitsInCV,coordTransInfo,v);
        //recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);

        cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
        cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;

        //if it's a lost situation then use odometry info
        if (recognizedTargetObjects.getReferenceObjects().size() < 10) {
            cout<<"LostSituation"<<endl<<endl<<endl;
            //currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
            aLine.setKP(1);
            referenceObjects.clear();
            referenceObjects.push_back(aLine);
            referenceObjects.push_back(allRobotPositions[6]);
            lostPoints.push_back(v); //just for printing at the end
        } else
            referenceObjects = recognizedTargetObjects.getReferenceObjects();
        referenceObjects = exitAsReference;
       
        //        if(v > 100) 
        //            ASRNumber = 3;
        
        if(recognizedTargetObjects.getReferenceObjects().size() != 0)
        localizedRPUsingExits = addTwoVectorsOfObjects(localizedRPUsingExits,myrobot.inMFIS(recognizedTargetObjects.getReferenceObjects()[0],recognizedTargetObjects.getReferenceObjects()[1],recognizedTargetObjects.getReferenceObjects()[0].getKP()));

        //localization 
        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
        lastLocomotion.set(allRobotPositions.back().X1(), allRobotPositions.back().Y1(),
                currentRobotPositionInMFIS.back().X1(), currentRobotPositionInMFIS.back().Y1(), 1);
        allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);
        cout << "Current robot position " << endl;
        currentRobotPositionInMFIS[7].display();
        
        plotObjectsOf4Kinds("Maps/FinalMap.png",MFIS,allRobotPositions,exitMFIS,localizedRPUsingExits);
     
        //waitHere();

        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 1) {
            cout << "updating situation " << endl;

            robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints,currentRobotPositionInMFIS);

            computedOutput = updatePerceptualMap(MFIS, currentView, currentRobotPositionInMFIS, referenceObjects, v, ASRNumber);
            //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
            MFIS = computedOutput.getView();
            targetObjectsInPV = computedOutput.getTargetObjects();
            limitingPoints.push_back(v); //limiting/updating points just for printing at the end

            //            if (v > 120) {
            //                            sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
            //                            plotObjects(viewFileName,myrobot.getRobot(),currentView);
            //
            sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
            // plotObjects(mfisFileName, currentRobotPositionInMFIS, MFIS);
            sprintf(mfisFileName, "%s%d%s", "Maps/MapAsTheViews-", v, ".png");
           //  plotRobotView(mfisFileName,currentRobotPositionInMFIS,MFIS);
            //            }

            if (computeASR == true) {
                //update Current ASR
                computedOutput = updatePerceptualMap(objectOfCurrentASR, currentView, currentRobotPositionInMFIS, referenceObjects,v, ASRNumber);
                //  computedOutput = computeMap(objectOfCurrentASR, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
                objectOfCurrentASR = computedOutput.getView();
                currentASR.addLimitingPoint(currentRobotPositionInMFIS[6]);
            }
        } else {
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
            targetExitsInPV = recognizedTargetObjects.getTargetObjects();
            //targetExitsInPV = currentView;
        }

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
            if ((splitToFromNewASR(allExitsInMFIS, lastLocomotion) > -1 && v != 114) or v == 105 or v == 158) {//&& v != 239 && v!= 163) {
                //waitHere();
                if (v == 105) {
                    //                cout<<"current view Size: "<<currentView.size()<<endl;
                    crossedExit.push_back(allExitsInMFIS[0]);
                    for (unsigned int e = 0; e < allExitsInMFIS.size(); e++) {
                        if (checkForIntersection(lastLocomotion, allExitsInMFIS[e]) == 1)
                            crossedExit.push_back(allExitsInMFIS[e]);
                    }
                } else
                    crossedExit.push_back(allExitsInMFIS[splitToFromNewASR(allExitsInMFIS, lastLocomotion)]);
                //            plotObjects("MFIS/test.png",perceptualMap.getCurrentASR().getASRObjects(),crossedExit);
                perceptualMap = splitCurrentASR(perceptualMap, crossedExit);

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


//                if (v > 205) {
                    sprintf(mfisFileName, "%s%d%s", "Maps/MFIS-", v, ".png");
                    plotObjectsOf3Kinds(mfisFileName, allRobotPositions, MFIS,projectingTheView(currentView,referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP()));
//                }

                waitHere();
        //        if(v == 70)
        //            ASRNumber = 2;
        //        if (v == 111)
        //            v=2;

        if (goalType == "NONEXIT_GOAL") {
            goal.second = 0;
        } else {
            goal.second = 0;
        }

    } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//
    cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

    //localization 
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

    vector<Object> firstAndLastRP = myrobot.getRobot();
    for (int i = 0; i < currentRobotPositionInMFIS.size(); i++) {
        firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
    }

    plotObjects("Maps/currentView.png", myrobot.getRobot(), currentView);
    plotObjects("Maps/1MFIS.png", robotPositionsAtLimitingPoints, MFIS);
    plotObjects("Maps/2MFISwithoutRP.png", firstAndLastRP, MFIS);
    plotRobotView("Maps/MapAsTheViews.png", firstAndLastRP, MFIS);


    //plotObjects("Maps/ASR1.png",allExitsInMFIS,perceptualMap.getCurrentASR().getASRObjects());


    // plotSingleASR("Maps/currentASR.png",perceptualMap.getCurrentASR());
    //  makeFinalPMUsingOldASRs(perceptualMap, firstAndLastRP);
    //testingFunction(perceptualMap,currentView);

    cout << "Traveled Dist: " << traveledDistance << endl;
    //    plotObjects("MFIS/lastView.png",myrobot.getRobot(),currentView);


    // plotAllASR(perceptualMap.getAllASRs(),currentRobotPositionInMFIS);
    if (computeASR == true) {
        //abstractASRs(perceptualMap.getAllASRs(),MFIS);
    }
    //  pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);

    keyInfoOfCompuetedPM(ASRNumber, exitPoints, lostPoints, limitingPoints);
    return 0;
}

