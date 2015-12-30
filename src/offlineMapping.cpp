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
	ifstream inputFile ("mapSettings.txt", ios::in);
    //int v = 0;
    int startpoint;
    int endpoint;
    //assign those variables to two integer values
    inputFile >> startpoint; 
    inputFile >> endpoint;
    inputFile.close();
	//cout << startpoint << endl << endpoint << endl;

    int v = startpoint; //, w, level, set;
    //    cout << "Which level you would like to map" << endl
    //            << "'0' for groudlevel" << endl
    //            << "'1' for firstlevel " << endl
    //            << "set 1 -- 999 for Guided xploration" << endl
    //            << "set  1001 --  for autonomous xploration" << endl << endl;
    //    level = 1; //cin >> level;
    //    cout << "Which Dataset ??? ";
    //    cin >> set;
    //    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
    //    cin >> v;
    //    cin >> w;
    //v = 3;

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
    limitingPoints.push_back(v);
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
    
    //reading odometery
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", v);
    coordTransInfo = readCoordTrans(coordTransFileName);


    //reading surfaces
    sprintf(viewFileName, "%s%d", "inputData/surfaces-", v);
    vector <Object> currentView = readASCII(viewFileName);

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
    while (v < endpoint) {
        v++;

        cout << "Now @ step " << v << endl;
        //reading odometery
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", v);
    coordTransInfo = readCoordTrans(coordTransFileName);


    //reading surfaces
    sprintf(viewFileName, "%s%d", "inputData/surfaces-", v);
    vector <Object> currentView = readASCII(viewFileName);

        cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;

        //finding Exits
        exitsFromCV = findShortestExits(currentView);
        //finding target objects
        targetObjectsInCV = findTargetObjects(currentView);
        //tagging sides and view number
        currentView = tagObjectsAsSideAndViewNumber(currentView, v);

        //saving current view
        sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);
        sprintf(viewFileName, "%s", "Maps/CurrentView.png");
        plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);

        //recognizing target Objects
        recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
        referenceObjects = recognizedTargetObjects.getReferenceObjects();

        cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
        cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;

        //localization using odometer
        Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
        aLine.setKP(1);
        odometricReferenceObject.clear();
        odometricReferenceObject.push_back(aLine);
        odometricReferenceObject.push_back(allRobotPositions[6]);
        odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());

        if (referenceObjects.size() > 0) {
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            //localization error checking
            angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(currentRobotPositionInMFIS[6]));
            distanceError = odometricCRPositionInMFIS[6].distP1ToP1(currentRobotPositionInMFIS[6]);
            if (distanceError > 400.0 or angleError > 5.0) {
                referenceObjects = odometricReferenceObject;
                currentRobotPositionInMFIS = odometricCRPositionInMFIS;
            }
        } else {
            referenceObjects = odometricReferenceObject;
            currentRobotPositionInMFIS = odometricCRPositionInMFIS;
        }
        allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

        //update MFIS and ASR if necessary
        if (recognizedTargetObjects.getTargetObjects().size() < 3) {
            cout << endl << "Updating situation " << endl;
            if ((v - limitingPoints.back()) > 1) { //update at last step //retriving info to update at last step
                cout << "Updating at last step. i.e. @ " <<v-1<< endl << endl;
                //MFIS = lastStepInfo.getMFIS();
                //currentView = lastStepInfo.getView();
                //referenceObjects = lastStepInfo.getReferenceObjects();
                //currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                v = v - 1;

                //updating at last step
                computedOutput = updatePerceptualMapATPlace(places, lastStepInfo.getMFIS(), lastStepInfo.getView(),
                        lastStepInfo.getRobotPosition(), lastStepInfo.getAllRobotPositions(), lastStepInfo.getReferenceObjects(),
                        v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                MFIS = computedOutput.getView();
                places = computedOutput.getASRs();
                targetObjectsInPV = computedOutput.getTargetObjects();
                allRobotPositions = lastStepInfo.getAllRobotPositions();
                limitingPoints.push_back(v); //limiting/updating points just for printing at the end

                sprintf(mfisFileName, "%s%d%s", "Maps/PM-u-", v, ".png");
                plotObjects(mfisFileName, allRobotPositions, MFIS);
                sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
                plotObjects(mfisFileName, allRobotPositions, MFIS);

                v++;
                //now processing current view
                //recognizing target Objects
                recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
                referenceObjects = recognizedTargetObjects.getReferenceObjects();

                if (referenceObjects.size() > 0) {
                    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
                    //localization error checking
                    angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(currentRobotPositionInMFIS[6]));
                    distanceError = odometricCRPositionInMFIS[6].distP1ToP1(currentRobotPositionInMFIS[6]);
                    if (distanceError > 400.0 or angleError > 5.0) {
                        referenceObjects = odometricReferenceObject;
                        currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                    }
                } else {
                    referenceObjects = odometricReferenceObject;
                    currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                }
                allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

                if (recognizedTargetObjects.getTargetObjects().size() < 3) {
                    //updating at current step
                    computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                            allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                    MFIS = computedOutput.getView();
                    places = computedOutput.getASRs();
                    targetObjectsInPV = computedOutput.getTargetObjects();
                    limitingPoints.push_back(v); //limiting/updating points just for printing at the end
                } else {
                    targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
                }

            } else if ((v - limitingPoints.back()) == 1) {//update at this step

                //updating at current step
                computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                MFIS = computedOutput.getView();
                places = computedOutput.getASRs();
                targetObjectsInPV = computedOutput.getTargetObjects();
                limitingPoints.push_back(v); //limiting/updating points just for printing at the end
            }
        } else
            targetObjectsInPV = recognizedTargetObjects.getTargetObjects();

        cout << "\n\033[1;34m******************PM is computed at step" << v << "********************\033[0m" << endl << endl;

        //save lastStep information to update at next step(in case)
        lastStepInfo.setMFIS(MFIS);
        lastStepInfo.setView(currentView);
        lastStepInfo.setTargetObjects(targetObjectsInCV);
        lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
        lastStepInfo.setAllRobotPositions(allRobotPositions);
        lastStepInfo.setReferenceObjects(referenceObjects);
        lastStepInfo.setExits(exitsFromCV);

        sprintf(mfisFileName, "%s%d%s", "Maps/RC-", v, ".png");
        plotObjects(mfisFileName, allRobotPositions, MFIS);
        sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
        plotObjects(mfisFileName, allRobotPositions, MFIS);

        //cout << "Take another step? (y/n) ";
        //cin >> n;
    }

    cout << "Traveled Dist: " << traveledDistance << endl;
    currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
    pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);
    //keyInfoOfCompuetedPM(ASRNumber, exitPoints, lostPoints, limitingPoints);
    return 0;
}


