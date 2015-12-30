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
#include <string>

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
#include "Comparison.h"

#include "GeometryFuncs.H"
#include "GeometricOp.H"

#include "thesis.H"

#include <cstdlib>
#include <ctime>

#define PI 3.14159265
using namespace std;

const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80], mfisFileName[80], ctFileName[80], LocalMFileName[80];
char Flag_LM;

vector<Object> NewLM; // a new local map
vector<Object> PreLM;  // previous local map
vector<Surface> currentPG;  // the polygon of current local map

vector<Object> LandMarks;

double crx = 0;  // robot position in local map coordinate
double cry = 0;  // 
double dist_currentLM = 0; // accumulate distance in a local map
double ang_currentLM = 0; // accumulate angle in a local map
double angl = 0 ; // accumualating angle

unsigned char PinLM_Flag = 0; // the flag of previous position of robot is in local map 0 -- not 1 -- yes


vector< vector<Object> > allLEs;
vector < vector<Object> > Memories;


int main() 
{
         int v, w, level, set;

         cout << "Which level?? ";
         cin >> level;
         cout << "Which dataset?? ";
         cin >> set;
         cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
         cin >> v;
         cin >> w;

         int referenceNumberTH = 0;
         char mappingInfoFileName[100];


         sprintf(mappingInfoFileName, "%s%d%s", "Maps/Offline/MappingInfo-refTh-", referenceNumberTH, ".txt");
         //variables
         Transporter recognizedTargetObjects, computedOutput,computedLocalMapOutput;
         vector <double> coordTransInfo;
         vector<Object> targetObjectsInPV, targetObjectsInCV, targetObjectsInMFIS;
         vector<Object> allTargetObjectsInPV;
         vector<Object> referenceObjects, odometricReferenceObject;
         vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
         vector<Object> previousRobotPositionInMFIS;
         vector<Object> recognizedTargetObjectInPV;
         vector<Object> routeMap;
         vector<Object> routeMapForOneASR;
         vector<Object> routeMapConnLP; //route map simply connecting limiting points
         Object pathFromLastLimitingPoint;
         vector<vector<Object> > routeMapForallASR;
         Object lastRouteMapNode, tempLastRouteMapNode;
         vector <Object> allRobotPositions, allOdometricRPosition;
         vector< vector<Object> > allRPoseAtOneStep;
         vector<Object> robotPositionsAtLimitingPoints;
         Transporter objectForPlaceRecognition;
         Transporter loopClosingInfo;
         Transporter lastStepInfo;
         vector<Object> refObjectForLoopClosing;
         string environmentType = "unknown";
         vector<int> lostPoints, limitingPoints, exitPoints, badLocalization;
         Object lineOfSitePoint;
         limitingPoints.push_back(1);
         Object lastLocomotion;
         vector<Object> wholeRoute;

         double traveledDistance = 0;
         double robotFacing = 0; //in degree

         double exitx=0;
         double exity=0;
       //  int odoLocalizationUsed = 0;
         ofstream outFile("Maps/Offline/LocalizationError.txt", ios::out);

         vector<Exit> exitsFromCV;
         vector<Object> crossedExit;

         vector<Object> objectOfCurrentASR;
         ASR currentASR;
         ASRNetwork perceptualMap;
         int ASRNumber = 1;
         vector<Object> globalMap;
         vector<Object>globalMaps;
         vector<ASR> places;

         MyRobot myrobot(0, 0);
         currentRobotPositionInMFIS = myrobot.getRobot();
         allRobotPositions = currentRobotPositionInMFIS;
         allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
         odometricCRPositionInMFIS = currentRobotPositionInMFIS; //just to see odometric robot position
         robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;
         //Arthur's variables;
         int objInCurrentMFIS = 0;
//         int objInPreviousMFIS = 0;
         vector<Object> LocalMap;
//         char LocalMapFileName[80];
         vector<Object> previousView;
         //end
         //for routeMap
         previousRobotPositionInMFIS = currentRobotPositionInMFIS;
         lastRouteMapNode = currentRobotPositionInMFIS[6];
         tempLastRouteMapNode = lastRouteMapNode;
         routeMap.push_back(lastRouteMapNode);
         routeMapForOneASR.push_back(lastRouteMapNode);
         routeMapConnLP.push_back(Object(0, 0, 0, 0, 1));

         sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);

         //reading the first view
         //cout << "........Reading " << viewFileName << endl;
         vector <Object> currentView = readASCII(viewFileName);
         globalMap = currentView;

         if (currentView.size() == 0) {
             //cout << "Need to change the file name" << endl;
             surfaceName = "/surface-";
             sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
             currentView = readASCII(viewFileName);
         }
         // previousView = currentView;
         //finding target objects
         targetObjectsInPV = findTargetObjects(currentView);
         referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
         referenceObjects.push_back(targetObjectsInPV[0]);

         //tagging side and view number
         currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

         for (unsigned int i = 0; i < currentView.size(); i++) {
             currentView[i].setASRNo(1);
             currentView[i].setLimitingPoint(1);
             currentView[i].setLocalEnvID(1);
         }

         //initializing MFIS
         vector<Object> MFIS = currentView;
         //cout << "..........MFIS.........." << endl;
         displayObjects(MFIS);
         PreLM = NewLM = currentView;          // first local map
         currentPG = makePolygonOfCV(NewLM); // first local to generate polygon
         Memories.push_back(NewLM); //store the first local map as first memory
         
         sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
         plotObjects(viewFileName, myrobot.getRobot(), NewLM);    
         
         //waitHere();

         //initializing Perceptual Map
         objectOfCurrentASR = currentView;
         currentASR.setASRObjects(objectOfCurrentASR);
         currentASR.setASRExit1(Object(-500, 0, 500, 0));
         currentASR.setASRID(1);
         lineOfSitePoint = currentRobotPositionInMFIS[6];
         currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
         perceptualMap.setCurrentASR(currentASR);


         sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
         sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
         plotObjectsOf3Kinds(viewFileName, allRobotPositions, targetObjectsInPV, currentView);
         // waitHere();
         plotObjects(mfisFileName, allRobotPositions, MFIS);
         targetObjectsInMFIS = findTargetObjectsFromMFIS(MFIS);
         objInCurrentMFIS = targetObjectsInMFIS.size();
         sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
         plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV,recognizedTargetObjects.getTargetObjects());
         

         
         // plotObjects(viewFileName, myrobot.getRobot(), currentView);
         //cout << "home: " << currentRobotPositionInMFIS[6].distP1ToP1(currentView[4]) << endl;

 
         vector<Object> errorMap = currentView; //odometric error map initialization


         vector<Point> updatingPoints;
         updatingPoints.push_back(Point(0, 0));

         bool exitCrossed = false;
         crossedExit.push_back(currentRobotPositionInMFIS[5]); //consider home as a exit
         crossedExit.back().setID(v);

         vector<int> failedToRecognizeRef;
         ofstream outFileForRefError("Maps/Offline/Localization", ios::out);
         outFileForRefError << v << " " << 1 << endl;
         //cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;


         do 
         {
                v++;
               //cout << "@ step " << v << endl;
               //Read coordTrans Data
               sprintf(ctFileName, "%s%d%s%d%s%d", levelName, level, "set", set, "/coordTrans-", v);

               coordTransInfo = readCoordTrans(ctFileName);
               /*
                       for(int i=0;i<coordTransInfo.size()&&v==5;i++){
                   cout <<  "coordTransInfo[" << i << "]:" <<coordTransInfo[i] << endl;
               }
               return 0;
                */
            traveledDistance = traveledDistance + coordTransInfo[0]; //The first element.
            robotFacing = robotFacing + coordTransInfo[1];                 //The second element.
            // errorMap = addTwoVectorsOfObjects(errorMap,xformPObjectsIntoCV())

            //changing the filenames 
            sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
            //reading current view and coordinate transformation info
            //cout << endl << endl << "........Reading " << viewFileName << endl;
            currentView = readASCII(viewFileName);
            if (currentView.size() == 0) {
                //cout << "Need to change the file name" << endl;
                surfaceName = "/surface-";
                sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
                currentView = readASCII(viewFileName);
            }
            //added begin by arthur
            Point rpos;
            double angle = (coordTransInfo[1] / 180) * PI; //angle in radian
            double rpx = coordTransInfo[0] * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
            double rpy = coordTransInfo[0] * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
            rpos.set(rpx, rpy);
            exitx = exitx + rpx;
            exity = exity + rpy;
           
            
            if(PinLM_Flag == 0)
            {
                    crx = coordTransInfo[0] * sin(-angle); 
                    cry = coordTransInfo[0] * cos(-angle);
            }
            else
            {
                    angl = (ang_currentLM / 180) * PI;
                    crx = dist_currentLM * sin(-angl);
                    cry = dist_currentLM * cos(-angl);
            }
            
            vector<Object> cv2pv;
            cout<< "angle" << angle << "x:" << rpx << "y:" <<rpy << endl;
            if(v%4!=0){
                cv2pv =  xformPObjectsIntoCV(globalMap,rpos,angle);
                globalMap = addTwoVectorsOfObjects(cv2pv,currentView); 
            }else{
                 sprintf(viewFileName, "%s%d%s", "Maps/Offline/global-", v, ".png");
                 plotObjects(viewFileName, myrobot.getRobot(), globalMap);
                 globalMap=currentView;
            }
          //  previousView = currentView;
            sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");

            odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
            //finding Exits
            exitsFromCV = findShortestExits(currentView);
            //finding target objects
            targetObjectsInCV = findTargetObjects(currentView);

            //recognizing target Objects
            recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
            
            plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV,recognizedTargetObjects.getTargetObjects());

            currentView = tagObjectsAsSideAndViewNumber(currentView, v);
            referenceObjects = recognizedTargetObjects.getReferenceObjects();
            if (set == 507 && v == 302)
            {
                referenceObjects.clear(); //there is a bug. 
            }

            if ((recognizedTargetObjects.getTargetObjects().size() < 3)) 
            {
                // if ((recognizedTargetObjects.getTargetObjects().size() < referenceNumberTH)) {
                //cout << endl << "Updating situation " << endl;
                if ((v - limitingPoints.back()) > 1) { //update at last step //retriving info to update at last step
                    MFIS = lastStepInfo.getMFIS();
              //        LocalMap = computedLocalMapOutput.getView();
                    currentView = lastStepInfo.getView();
                    referenceObjects = lastStepInfo.getReferenceObjects();
                    currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                    if (referenceObjects.size() == 0) {
                        //cout << "trying to update at " << v - 1 << " but no ref: " << endl;
                        waitHere();
                    }
                    v = v - 1;
                } else if ((v - limitingPoints.back()) == 1) {//update at this step

                     if (referenceObjects.size() == 0) 
                     {
                            //localization using odometer
                            Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
                            aLine.setKP(1);
                            odometricReferenceObject.clear();
                            odometricReferenceObject.push_back(aLine);
                            odometricReferenceObject.push_back(allRobotPositions[6]);
                            odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());
                            referenceObjects = odometricReferenceObject;
                            lostPoints.push_back(v); //just for printing at the end
                      }
                }

                  //localization 
                  currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
                  allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

                  //routeMap
                  pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                        currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
                  routeMapConnLP.push_back(pathFromLastLimitingPoint);

                  robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);
                  updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                  computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                  //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
                  computedLocalMapOutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                  MFIS = computedOutput.getView();
                //  LocalMap = computedLocalMapOutput.getView();
                  places = computedOutput.getASRs();
                  targetObjectsInPV = computedOutput.getTargetObjects();
                  limitingPoints.push_back(v); //limiting/updating points just for printing at the end


                  //to print local and global map

                  sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
                  plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);
                
                  sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
                  plotObjects(viewFileName, myrobot.getRobot(), currentView);
                  
                  // the position of robot in current local map/boundary
                  //PointXY robotP(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1());
                  
                  PointXY robotP(crx, cry); // robot position is going to be detected in a particular local map
                  
                  Flag_LM = pointInPolygon(robotP, currentPG, 0); // Whether the robot position in local map
                  
                  if(!Flag_LM)
                  {
                      
                        //  sprintf(viewFileName, "%s%d%s", "Maps/Offline/currentPG-", v-1, ".png");
                          // vector<Object> tmp =  xformPObjectsIntoCV(NewLM,rpos,angle);
                       //     plotObjects(viewFileName, currentPG, myrobot.getRobot(robotP));
                           currentPG.clear();
                           NewLM = lastStepInfo.getView();
                           LandMarks= findTargetObjects(NewLM);
                           currentPG = makePolygonOfCV(NewLM); // make a new polygon/boundary of local environment
                           Memories.push_back(NewLM);                  // all local environments are memories
                           //lastStepInfo.getRobotPosition()
                           sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v-1, ".png");
                           //plotObjects(viewFileName, myrobot.getRobot(), NewLM);         
                             MyRobot currentrobot = MyRobot(exitx, exity);
                      //       MyRobot zero(0,0);
                             cout<< "exitx:" << exitx << "       exity:"<< exity << endl;
                             
                     //      plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), xformObjectsbyXY(currentrobot.getRobot(),exitx,exity),NewLM);
                           dist_currentLM = 0;
                           ang_currentLM = 0;
                           exitx = 0;
                           exity = 0;
                           PinLM_Flag = 0;
                  }
                  else
                  {
                           PinLM_Flag = 1;
                           dist_currentLM += coordTransInfo[0];
                           ang_currentLM += coordTransInfo[1];
                  }
                    
                  
                  recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();
            } else
                targetObjectsInPV = recognizedTargetObjects.getTargetObjects();

            //cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;

            //save lastStep information to update at next step(in case)
            lastStepInfo.setMFIS(MFIS);
            lastStepInfo.setView(currentView);
            lastStepInfo.setTargetObjects(targetObjectsInCV);
            lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
            lastStepInfo.setReferenceObjects(referenceObjects);
            lastStepInfo.setExits(exitsFromCV);
        } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//
        cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

        outFile.close();
        outFileForRefError.close();
        //localization 
        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

        vector<Object> firstAndLastRP = myrobot.getRobot();
        for (unsigned int i = 0; i < currentRobotPositionInMFIS.size(); i++) {
            firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
        }

        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", referenceNumberTH, ".png");
        plotObjects(mfisFileName, robotPositionsAtLimitingPoints, MFIS);
        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
        plotObjectsOf3Kinds(mfisFileName, firstAndLastRP, crossedExit, MFIS);

        //cout << "Traveled Dist: " << traveledDistance << endl;
        writeASCII(convertObjectToSurface(MFIS), "Maps/Offline/pm"); //for thesis
        //plotObjects("Maps/errorMap.png", errorMap, errorMap); //for thesis
        perceptualMap.setMFIS(MFIS);
        // if (computeASR == true) {
        //abstractASRs(places,MFIS,routeMapForallASR,refObjectForLoopClosing);
        //}

        abstractRouteMap(MFIS, robotPositionsAtLimitingPoints, updatingPoints, currentRobotPositionInMFIS);
        keyInfoOfCompuetedPM(mappingInfoFileName, ASRNumber, exitPoints, lostPoints, limitingPoints,
                badLocalization, failedToRecognizeRef, referenceNumberTH, level, set);
        //cout << levelName << level << " set " << set << endl;
        //cout << "odo " << odoLocalizationUsed << endl;


        return 0;
}












