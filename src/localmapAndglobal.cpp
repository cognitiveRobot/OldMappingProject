/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

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
//#include <bits/stl_algobase.h>
//#include <bits/mathcalls.h>
#include <math.h>

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
//***//
void Init_variables();
double mult(Point a, Point b, Point c);
bool intersect(vector<Object> CurrentV, Point Pos);


bool between(double a, double X0, double X1);
bool detectIntersect(vector<Object> CurrentV, Point Pos); // detect the intersection between path and surfaces
bool TwoAdjacentPos(Point A, Point B); // detect nearby position and delete these

// combine all local maps in to one global map
void CombineLocalMaps(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); //transform and plot
void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num); // only transform

// detect the intersection between a new path segment and old path segments
bool TwopathSegment(vector<Point> AllPositions);

// calculate the angle between most recent two adjacent paths
bool TwoPathAngle(vector<Point> AllPositions);

// Transform current position onto old coordinate system
Point TransPointToOldCrd(double Transf_angle, Point Transf_point, Point coord);

void TwoPathsCrossed(int num);

vector<Object> TransPositions(vector<Object> positions, double TransAngle, Point coord);

const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80], mfisFileName[80], ctFileName[80], LocalMFileName[80];
char exitsFileName[80];

char testFileName[80]; // this file char array is of test programme and plot

char Flag_LM;

vector<Object> NewLM; // a new local map
vector<Object> PreLM;  // previous local map
vector<Surface> currentPG;  // the polygon of current local map

vector<Object> LandMarks;

double crx = 0;  // robot position in local map coordinate
double cry = 0;  // 
double dist_currentLM = 0; // accumulate distance in a local map
double ang_currentLM = 0; // accumulate angle in a local map
double Pangle = 0 ; // accumulating angle

double dist_rb = 0;

vector<double> distFromorign;
vector<double> distFromrbot ;

unsigned char PinLM_Flag = 0; // the flag of previous position of robot is in local map 0 -- not 1 -- yes
vector<double> odome;


vector< vector<Object> > allLEs; // all local environments 
vector< vector<Object> > Memories; // all local maps as memories
vector< vector<Object> > ListOfPositions; // for storing currentCRPInMFIS variable 

vector<ASR> pla; // for computing local maps
//vector<Object> TwoLines;
vector<Point> RobotPs;
Point rb; // in current coordinate robot position
Point Prerb; // in previous coordinate robot position where it moves out

Point PathSegment1P1; // path segment 1 point 1
Point PathSegment1P2; // path segment 1 point 2
Point PathSegment2P1; // path segment 2 point 1
Point PathSegment2P2; // path segment 2 point 2

//**********************************//
vector<Object> GlobalLM;
vector<ASR> placesOfLM;
Transporter computeRe, Cmoutput;
Transporter lastLMInfo;

//Transporter CombineLMs;
vector<Object> MapofLMs;

vector<Object> Refs;
vector<Object> positions; // position in current local map
vector<Object> AP;

Object xAxisAtCRP; // x axis angle current robot position in global coordinate
Object xAxisIncurrentCrd; // x axis angle current robot position in current coordinate
Object Pathfromlimiting;
vector<Object> routeMapCo;

vector<Object> PreLandmarks;
vector<Exit> exitsFromcurrentLM;
vector<Object>  odomRef;
vector<Object> odometricPosition;
vector<Object> LandMarkCV;
vector<Object> robotPositionsInLMs;

vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;

vector<Object> currentTransLM;

unsigned char firstloop_Flag = 0;
unsigned char inter_Flag = 0;
unsigned char clear_Flag = 0;
unsigned char pathCrossed_Flag = 0;
unsigned char OutOfASR_Flag = 0;
unsigned char NextOfOutASR_Flag = 0;

int CrossedP1 = 0; // when two paths are crossed, return the old segment point1
int CountOfOutASR = 0;

MyRobot robot(0,0); 

int main() 
{
         int v, w, r,level, set, saveFrom, saveTo;

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
         //vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
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
         limitingPoints.push_back(v);
         Object lastLocomotion;
         vector<Object> wholeRoute;

         double traveledDistance = 0;
         double robotFacing = 0; //in degree


         int odoLocalizationUsed = 0;
         ofstream outFile("Maps/Offline/LocalizationError.txt", ios::out);

         vector<Exit> exitsFromCV;
         vector<Object> crossedExit;

         vector<Object> objectOfCurrentASR;
         ASR currentASR;
         ASRNetwork perceptualMap;
         int ASRNumber = 1;

         vector<ASR> places;

         MyRobot myrobot(0, 0);
         currentRobotPositionInMFIS = myrobot.getRobot();
         allRobotPositions = currentRobotPositionInMFIS;
         allOdometricRPosition = currentRobotPositionInMFIS; //just to see odometric robot position
         odometricCRPositionInMFIS = currentRobotPositionInMFIS; //just to see odometric robot position
         robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;
         
         ListOfPositions.push_back(currentRobotPositionInMFIS); // push the first position into a list
         
         RobotPs.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
         
         
         AP = positions = robot.getRobot(); // initial the positions.
         odometricPosition = positions;
         //robotPositionsInLMs = robot.getRobot();
                 
         //Arthur's variables;
         int objInCurrentMFIS = 0;
         int objInPreviousMFIS = 0;
         vector<Object> LocalMap;
         char LocalMapFileName[80];
         vector<Object> preMFIS;
         vector<Object> LocalMapLists;
         vector<Object> previousView;
         //end
         //for routeMap
         previousRobotPositionInMFIS = currentRobotPositionInMFIS;
         lastRouteMapNode = currentRobotPositionInMFIS[6];
         tempLastRouteMapNode = lastRouteMapNode;
         routeMap.push_back(lastRouteMapNode);
         routeMapForOneASR.push_back(lastRouteMapNode);
         routeMapConnLP.push_back(Object(0, 0, 0, 0, 1));
         
         routeMapCo.push_back(Object(0, 0, 0, 0, 1));
         sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);

         //reading the first view
         //cout << "........Reading " << viewFileName << endl;
         vector <Object> currentView = readASCII(viewFileName);

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
         
         PreLandmarks = targetObjectsInPV;
         Refs.push_back(PreLandmarks[0]);
         Refs.push_back(PreLandmarks[0]);
          
         //tagging side and view number
         currentView = tagObjectsAsSideAndViewNumber(currentView, v);

         for (unsigned int i = 0; i < currentView.size(); i++) 
         {
             currentView[i].setASRNo(v);
             currentView[i].setLimitingPoint(v);
             currentView[i].setLocalEnvID(v);
         }

         //initializing MFIS
         vector<Object> MFIS = currentView;
         //cout << "..........MFIS.........." << endl;
         displayObjects(MFIS);
         NewLM = currentView;          // first local map
         GlobalLM = currentView;
         
         MapofLMs = NewLM; // the first local map in global
         sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");            
         plotObjects(LocalMFileName, robot.getRobot(), MapofLMs);
         
         //currentPG = makePolygonOfCV(NewLM); // first local to generate polygon
         Memories.push_back(NewLM); //store the first local map as first memory
         currentTransLM = NewLM; // the first one does not to transform
         
         exitsFromcurrentLM = findGapasExits(NewLM); // initial exits in first view/local environment
         
         sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
         plotObjects(viewFileName, myrobot.getRobot(), NewLM);    
         
         for(int i = 0; i < exitsFromcurrentLM.size(); i++)
                exitsFromcurrentLM[i].display();
         
         sprintf(exitsFileName, "%s%d%s", "Maps/Offline/Exits-", v, ".png");
         plotObjectsOf3KindswithExits(exitsFileName, myrobot.getRobot(), currentView, exitsFromcurrentLM);
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
      //   plotObjectsOf3Kinds(viewFileName, allRobotPositions, targetObjectsInPV, currentView);
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
                
                // a^2 = b^2 + c^2 -2*b*c*cos(A);
               
                
                vector<Object> cv2pv;
                cout<< "angle" << angle << "x:" << rpx << "y:" <<rpy << endl;
                if(previousView.size()!=0)
                {
                    cv2pv =  xformPObjectsIntoCV(previousView,rpos,angle);
                    LocalMap = addTwoVectorsOfObjects(LocalMap,cv2pv);

                }
                previousView = currentView;


                odometricErrorMap(errorMap, currentView, coordTransInfo[0], coordTransInfo[1]); //computing error map
                //finding Exits
                exitsFromCV = findShortestExits(currentView);
                //exitsFromCV = findExits(currentView);
                
                
                //finding target objects
                targetObjectsInCV = findTargetObjects(currentView);
                
                LandMarkCV = findTargetObjects(currentView);

                //recognizing target Objects
                recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v, limitingPoints.back());
                
                 sprintf(viewFileName, "%s%d%s", "Maps/Offline/View-", v, ".png");
                // plotObjectsOf4Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV,recognizedTargetObjects.getTargetObjects());

                //plot exit and boundary
                //sprintf(exitsFileName, "%s%d%s", "Maps/Offline/exitAndboundary-", v, ".png");
                //plotExitsandBoundary(exitsFileName, exitsFromCV) ;
                
                currentView = tagObjectsAsSideAndViewNumber(currentView, v);
                referenceObjects = recognizedTargetObjects.getReferenceObjects();
                
                // for computing local maps
                //computeRe = recognizeTargetObjects(GlobalLM, PreLandmarks, currentView, coordTransInfo, v);
                computeRe = recognizeTargetObjects(GlobalLM, PreLandmarks, LandMarkCV, coordTransInfo, v, limitingPoints.size());
                Refs = computeRe.getReferenceObjects();
                
                
                if (set == 507 && v == 302)
                {
                     referenceObjects.clear(); //there is a bug. 
                }

                if ((recognizedTargetObjects.getTargetObjects().size() < 3)) 
                {
                        // if ((recognizedTargetObjects.getTargetObjects().size() < referenceNumberTH)) {
                        //cout << endl << "Updating situation " << endl;
                        if ((v - limitingPoints.back()) > 1) //update at last step //retriving info to update at last step
                        { 
                                MFIS = lastStepInfo.getMFIS();
                                LocalMap = computedLocalMapOutput.getView();
                                currentView = lastStepInfo.getView();
                                referenceObjects = lastStepInfo.getReferenceObjects();
                                currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();

                                Refs = lastLMInfo.getReferenceObjects();
                                positions = lastLMInfo.getRobotPosition();

                                if (referenceObjects.size() == 0)
                                {
                                    //cout << "trying to update at " << v - 1 << " but no ref: " << endl;
                                    waitHere();
                                }
                                v = v - 1;
                         } 
                         else 
                                if ((v - limitingPoints.back()) == 1) //update at this step
                                {

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

                                               Object ThLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, positions[6], 1);
                                               ThLine.setKP(1);
                                               odomRef.clear();
                                               odomRef.push_back(ThLine);
                                               odomRef.push_back(AP[6]);
                                               odometricPosition = robot.inMFIS(odomRef[0], odomRef[1], odomRef[0].getKP());
                                               Refs = odomRef;
                                         }
                                }

                      //localization 
                      currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
                      allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

                      //localization in current local map
                      positions = robot.inMFIS(Refs[0], Refs[1], Refs[0].getKP());
                      AP =  addTwoVectorsOfObjects(AP, positions);


                      //rb.set(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1());
                      rb.set(positions[6].X1(), positions[6].Y1());
                      xAxisAtCRP = currentRobotPositionInMFIS[7];
                      
                      if(detectIntersect(NewLM, rb)) // If the path between starting and current positions is intersected with any surface
                      {
                                cout << "intersection flag: " << (int)inter_Flag << endl;
                                cout << "number of the view v: " << v << endl;
                                
                                clear_Flag = 1;
                                OutOfASR_Flag = 0;
                      }
                      else // should add a special case. When the robot moves out from the behind of starting position.
                      {
                                cout << "No any intersection " << endl << endl;
                                cout << "It is going to check whether the position is under the X axis" << endl << endl;

                                if(rb.Y() < 0) // If the position appears under X axis, this position has moved out this area
                                {
                                                clear_Flag = 1;
                                                OutOfASR_Flag = 1;
                                }

                      }

                      /*----------------  Compute for Local Maps ------------------- */
                      if(clear_Flag == 0)
                      {

                                Pathfromlimiting.set(routeMapCo.back().X2(), routeMapCo.back().Y2(),
                                      positions[6].X1(), positions[6].Y1(), v);

                                routeMapCo.push_back(Pathfromlimiting);
                                if(Refs.size() == 0)
                                    waitHere();
                                Cmoutput = updatePerceptualMapATPlace(pla, GlobalLM, currentView, positions,
                                      AP, Refs, v, ASRNumber, exitCrossed, crossedExit, limitingPoints,routeMapCo);
                                //Cmoutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                            //allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);

                                GlobalLM = Cmoutput.getView();
                                pla = Cmoutput.getASRs();
                                PreLandmarks = Cmoutput.getTargetObjects();
                      }
                      else 
                           if(clear_Flag ==1)
                           {
                                if(OutOfASR_Flag == 0)
                                {
                                      sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                     // plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);
                                      
                                      PreLM = NewLM; // store as previous local map
                                      NewLM = currentView; // new local map, new coordinate system
                                      Prerb = rb; // save it as old position
                                      xAxisIncurrentCrd = positions[7];
                                      
                                      //push this position into the list
                                      ListOfPositions.push_back(currentRobotPositionInMFIS);

                                      //add this position into all
                                      robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions

                                      // combine each local map
                                      CombineLocalMaps(NewLM, Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

                                      RobotPs.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                                      if(TwopathSegment(RobotPs)) // detect two new path is intersect 
                                             TwoPathsCrossed(v);
                                      
                                      Init_variables(); // clear all variables
                                 } 
                                else 
                                    if(OutOfASR_Flag == 1)
                                    {
                                            NewLM = PreLM; // take the pre local map as current local map
                                           
                                            //coordinate transform the point onto Previous coordinate system
                                            rb = TransPointToOldCrd(xAxisIncurrentCrd.getAngleWithXaxis(), rb, Prerb);
                                            positions = TransPositions(positions, xAxisIncurrentCrd.getAngleWithXaxis(), Prerb);
                                            if(detectIntersect(NewLM, rb))
                                            {
                                                    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                                   // plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);
                                                    PreLM = NewLM; // store as previous local map
                                                    NewLM = currentView; // new local map, new coordinate system

                                                    //push this position into the list
                                                    ListOfPositions.pop_back();
                                                    ListOfPositions.push_back(currentRobotPositionInMFIS);

                                                    //add this position into all
                                                    //robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions
                                                    robotPositionsInLMs.clear();
                                                    for(int i = 0; i < ListOfPositions.size() ; i++)
                                                           robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i]);
                                                    
                                                    Memories.pop_back();
                                                    // combine each local map
                                                    TransforIntoGloabl(NewLM, Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);
                                                    MapofLMs.clear();
                                                    for(int i = 0; i < Memories.size(); i++)
                                                        MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);
                                                    
                                                    sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");            
                                                  //  plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
                                                    
                                                    RobotPs.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                                                    //if crossed with any surface in this ASR, generate and clear
                                                    Init_variables(); 
                                            }
                                            else // no intersection with all surfaces in this ASR -- two cases 1 in ASR 2 back of ASR
                                            {
                                                    if(rb.Y() < 0) // still back of the ASR
                                                    {
                                                            sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                                           // plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);
                                                            PreLM = NewLM; // store as previous local map
                                                            NewLM = currentView; // new local map, new coordinate system

                                                            Prerb = rb; // save it as old position
                                                            xAxisIncurrentCrd = positions[7];

                                                            //push this position into the list
                                                            ListOfPositions.pop_back();
                                                            ListOfPositions.push_back(currentRobotPositionInMFIS);

                                                            //add this position into all
                                                            //robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions
                                                            robotPositionsInLMs.clear();
                                                            for(int i = 0; i < ListOfPositions.size() ; i++)
                                                                   robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i]);

                                                            Memories.pop_back();
                                                            // combine each local map
                                                            TransforIntoGloabl(NewLM, Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

                                                            MapofLMs.clear();
                                                            for(int i = 0; i < Memories.size(); i ++)
                                                                 MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);

                                                            sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", v, ".png");            
                                                            //plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());

                                                            if(TwopathSegment(RobotPs)) // detect two new path is intersect 
                                                                    TwoPathsCrossed(v);

                                                            //if crossed with any surface in this ASR, generate and clear
                                                             Init_variables(); 
                                                    }
                                                    else
                                                        if(rb.Y() > 0) // still in the ASR
                                                        {
                                                                //positions = TransPositions(positions, xAxisIncurrentCrd.getAngleWithXaxis(), Prerb);
                                                                //ListOfPositions.pop_back();
                                                                //Memories.pop_back();

                                                                clear_Flag = 0;

                                                                cout << "test.................programme" << endl << endl;
                                                                cout << "where is this step v: " << v << endl;
                                                                cout << "clear flag is : " << (int)clear_Flag << endl;
                                                                cout << "rb x : " << rb.X() << "  rb y: " << rb.Y() << endl;
                                                                cout << "positions[6] x: " << positions[6].X1() << "  positions[6] y: " << positions[6].Y1() << endl;  
                                                                //waitHere();

                                                                Pathfromlimiting.set(routeMapCo.back().X2(), routeMapCo.back().Y2(),
                                                                positions[6].X1(), positions[6].Y1(), v);

                                                                routeMapCo.push_back(Pathfromlimiting);
                                                                if(Refs.size() == 0)
                                                                    waitHere();
                                                                Cmoutput = updatePerceptualMapATPlace(pla, GlobalLM, currentView, positions,
                                                                      AP, Refs, v, ASRNumber, exitCrossed, crossedExit, limitingPoints,routeMapCo);

                                                                GlobalLM = Cmoutput.getView();
                                                                pla = Cmoutput.getASRs();
                                                                PreLandmarks = Cmoutput.getTargetObjects();
                                                        }
 
                                            }
                                            
                                            OutOfASR_Flag = 0;
                                    }
                                 //Init_variables(); // clear all variables
                           }

                  /*===========================================================================================*/
                      //routeMap
                      pathFromLastLimitingPoint.set(routeMapConnLP.back().X2(), routeMapConnLP.back().Y2(),
                            currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), v);
                      routeMapConnLP.push_back(pathFromLastLimitingPoint);

                      robotPositionsAtLimitingPoints = addTwoVectorsOfObjects(robotPositionsAtLimitingPoints, currentRobotPositionInMFIS);
                      updatingPoints.push_back(Point(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                      //computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                      //      allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                        
                      computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                            allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, limitingPoints,routeMapConnLP);

                      //computedOutput = computeMap(MFIS, currentView, currentRobotPositionInMFIS[7], referenceObjects, ASRNumber);
                      computedLocalMapOutput = updatePerceptualMapATPlace(places, LocalMap, currentView, currentRobotPositionInMFIS,
                            allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, limitingPoints,routeMapConnLP);
                      MFIS = computedOutput.getView();
                      LocalMap = computedLocalMapOutput.getView();
                      places = computedOutput.getASRs();
                      targetObjectsInPV = computedOutput.getTargetObjects();
                      limitingPoints.push_back(v); //limiting/updating points just for printing at the end


                      //to print local and global map

                      sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-", v, ".png");
                      //plotpathandobject(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS, updatingPoints);                      
                      //plotObjectsOf3Kinds(mfisFileName, myrobot.getRobot(), currentRobotPositionInMFIS, MFIS);


                      sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalEnv-", v, ".png");
                      //plotObjects(viewFileName, myrobot.getRobot(), currentView);

                      recognizedTargetObjectInPV = recognizedTargetObjects.getTargetObjects();
                } 
                else
                {
                        targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
                        PreLandmarks = computeRe.getTargetObjects();
                }

                
                    if(v == w) // the last step update still
                      {
                          
                          cout << "----This is the last step, it is going to process and generate final map here----" << endl; 
                          //waitHere();
                                    sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v, ".png");
                                 //   plotObjectsOf3Kinds(viewFileName, myrobot.getRobot(), NewLM, positions);

                                    PreLM = NewLM; // store as previous local map
                                    NewLM = currentView; // new local map, new coordinate system
                                    Prerb = rb; // save it as old position
                                    xAxisIncurrentCrd = positions[7];

                                    //push this position into the list
                                    ListOfPositions.push_back(currentRobotPositionInMFIS);

                                    //add this position into all
                                    robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, currentRobotPositionInMFIS); // store this position into all positions

                                    // combine each local map
                                    CombineLocalMaps(NewLM, Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()), currentRobotPositionInMFIS, xAxisAtCRP.getAngleWithXaxis(), v);

                                    RobotPs.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));

                                    if(TwopathSegment(RobotPs)) // detect two new path is intersect 
                                           TwoPathsCrossed(v);

                                    Init_variables(); // clear all variables
                      }
                
                
            //cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;

              
            //save lastStep information to update at next step(in case)
            lastStepInfo.setMFIS(MFIS);
            lastStepInfo.setView(currentView);
            lastStepInfo.setTargetObjects(targetObjectsInCV);
            lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
            lastStepInfo.setReferenceObjects(referenceObjects);
            lastStepInfo.setExits(exitsFromCV);
            
              if(clear_Flag == 0)
              {
                    //for computing local maps
                    lastLMInfo.setMFIS(GlobalLM);
                    lastLMInfo.setView(currentView);
                    lastLMInfo.setTargetObjects(PreLandmarks);
                    lastLMInfo.setRobotPosition(positions);
                    lastLMInfo.setReferenceObjects(Refs);
                    //lastLMInfo.setExits(exitsFromCV);
              }
              else
                  Transporter lastLMInfo;


            firstloop_Flag = 1;
        } while (v < w); //(v != 79);//  //(v != 55); //  while(n == 'y' || n == 'Y');//
        
        cout << "MFIS size " << MFIS.size() << " all robot position size " << allRobotPositions.size() << endl;

        outFile.close();
        outFileForRefError.close();
        //localization 
        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());

        vector<Object> firstAndLastRP = myrobot.getRobot();
        for (int i = 0; i < currentRobotPositionInMFIS.size(); i++)
        {
            firstAndLastRP.push_back(currentRobotPositionInMFIS[i]);
        }

        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/MFIS-refTH-", referenceNumberTH, ".png");
       // plotObjects(mfisFileName, robotPositionsAtLimitingPoints, MFIS);
        sprintf(mfisFileName, "%s%d%s", "Maps/Offline/finalPercptualMap-", set, ".png");
      //  plotObjectsOf3Kinds(mfisFileName, firstAndLastRP, crossedExit, MFIS);

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

void Init_variables()
{
                //re-initialization coordinate system and all positions
                GlobalLM.clear();
                pla.clear();
                PreLandmarks.clear();
                routeMapCo.clear();
                positions.clear();
                Refs.clear();
                AP.clear();
                odomRef.clear();
                odometricPosition.clear();
                MyRobot robot(0,0);
                CrossedP1 = 0;

                //NewLM = currentView; // new local map, new coordinate system
                GlobalLM = NewLM;

                PreLandmarks = findTargetObjects(NewLM);
                Refs.push_back(PreLandmarks[0]);
                Refs.push_back(PreLandmarks[0]);
                AP = positions = robot.getRobot();
                odometricPosition = positions;
                routeMapCo.push_back(Object(0, 0, 0, 0, 1));
                pathCrossed_Flag = 0;
                //OutOfASR_Flag = 0;
                clear_Flag = 0; // clear flag first
}




///********************************************************************///
//**** Vector Product method****//
double mult(Point a, Point b, Point c)  
{  
      return (a.X()-c.X())*(b.Y()-c.Y())-(b.X()-c.X())*(a.Y()-c.Y());  
} 

bool intersect(vector<Object> CurrentV, Point Pos)  
{  
    
        Point aa, bb, cc, dd;
        
        //Object line1, line2;
        
        //line1.set(0.0, 0.0, CurrentV[0].X1(), CurrentV[0].Y1(), -1);  // the imaginary surface joined by starting position and first object P1
        //line2.set(CurrentV.back().X2(), CurrentV.back().Y2(), 0.0, 0.0, -2); // the imaginary surface joined by starting position and last object P2
        //CurrentV.push_back(line1);
        //CurrentV.push_back(line2);
        
        cc.set(0.0, 0.0); // origin point (0,0)
        dd.set(Pos.X(), Pos.Y());
    
         for(int i = 0; i < CurrentV.size(); i++)
         {
                aa.set(CurrentV[i].X1(), CurrentV[i].Y1());
                bb.set(CurrentV[i].X2(), CurrentV[i].Y2());

                if ( max(aa.X(), bb.X()) < min(cc.X(), dd.X()) )  
                {  
                    return false;  
                }  
                if ( max(aa.Y(), bb.Y()) < min(cc.Y(), dd.Y()) )  
                {  
                    return false;  
                }  
                if ( max(cc.X(), dd.X()) < min(aa.X(), bb.X()) )  
                {  
                    return false;  
                }  
                if ( max(cc.Y(), dd.Y()) < min(aa.Y(), bb.Y()) )  
                {  
                    return false;  
                }  
                if ( mult(cc, bb, aa) * mult(bb, dd, aa) < 0 )  
                {  
                    return false;  
                }  
                if ( mult(aa, dd, cc) * mult(dd, bb, cc) < 0 )  
                {  
                    return false;  
                }  
                return true;  
         }
}

///****************************************************************///
//** detect whether two line segments are intersected **//
bool between(double a, double X0, double X1)  
{  
        double temp1 = a-X0;  
        double temp2 = a-X1;  
        if ( ( temp1 < 1e-8 && temp2 > -1e-8 ) || ( temp2 < 1e-6 && temp1 > -1e-8 ))  
        {  
            return true;  
        }  
        else  
        {  
            return false;  
        }  
}  
  
 
bool detectIntersect(vector<Object> CurrentV, Point Pos)  
{  
    cout << "***** This is going to a process two line segments are intersected *****" << endl << endl;

        Point p1, p2, p3, p4;
        double line_x, line_y; //intersect position  
 
        
        p3.set(0.0, 0.0); // origin point (0,0)
        p4.set(Pos.X(), Pos.Y());
        
        inter_Flag = 0;
       
        for(int i = 0; i < CurrentV.size(); i++)
        {        
                p1.set(CurrentV[i].X1(), CurrentV[i].Y1());
                p2.set(CurrentV[i].X2(), CurrentV[i].Y2());
                /*
                if ( (fabs(p1.X() - p2.X()) < 1e-6) && (fabs(p3.X() - p4.X()) < 1e-6) )  
                {  
                    return false;  
                }  
                else 
                {
                            if ( (fabs(p1.X() - p2.X()) < 1e-6) ) 
                            {  
                                    if (between(p1.X(), p3.X(), p4.X()))  
                                    {  
                                            double k = (p4.Y() - p3.Y()) / (p4.X() - p3.X());  
                                            line_x = p1.X();  
                                            line_y = k * (line_x - p3.X()) + p3.Y();  

                                            if (between(line_y, p1.Y(), p2.Y()))  
                                            {  
                                                return true;  
                                                inter_Flag = 1;
                                            }  
                                            else  
                                            {  
                                                return false;  
                                            }  
                                    }  
                                    else   
                                    {  
                                         return false;  
                                    }  
                            }  
                            else 
                            {
                                    if ( (fabs(p3.X() - p4.X()) < 1e-6) ) 
                                    {  
                                            if (between(p3.X(), p1.X(), p2.X()))  
                                            {  
                                                double k = (p2.Y() - p1.Y()) / (p2.X() - p1.X());  
                                                line_x = p3.X();  
                                                line_y = k * (line_x - p2.X()) + p2.Y();  

                                                if (between(line_y, p3.Y(), p4.Y()))  
                                                {  
                                                        return true;  
                                                        inter_Flag = 1;
                                                }  
                                                else  
                                                {  
                                                      return false;  
                                                }  
                                            }  
                                            else   
                                            {  
                                                 return false;  
                                            }  
                                  }  
                                  else  
                                  {  
                                        double k1 = (p2.Y() - p1.Y())/(p2.X() - p1.X());   
                                        double k2 = (p4.Y() - p3.Y())/(p4.X() - p3.X());  

                                        if (fabs(k1 - k2) < 1e-6)  
                                        {  
                                             return false;  
                                        }  
                                        else   
                                        {  
                                                line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1 - k2);  
                                                line_y = k1*(line_x - p1.X()) + p1.Y();  
                                        }  

                                        if (between(line_x, p1.X(), p2.X()) && between(line_x, p3.X(), p4.X()))  
                                        {  
                                                return true;  
                                                inter_Flag = 1;
                                        }  
                                        else   
                                        {  
                                                return false;  
                                        }  
                                  }  
                            }
                }*/
                
                if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                {  
                    //return false;  
                    inter_Flag = 0;
                }  
                else if ( (fabs(p1.X()-p2.X())<1e-6) )
                {  
                    if (between(p1.X(),p3.X(),p4.X()))  
                    {  
                            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                            line_x = p1.X();  
                            line_y = k*(line_x-p3.X())+p3.Y();  

                            if (between(line_y,p1.Y(),p2.Y()))  
                            {  
                                 
                                inter_Flag = 1;
                                return true; 
                            }  
                            else  
                            {  
                                //return false;  
                                inter_Flag = 0;
                            }  
                    }  
                    else   
                    {  
                            //return false;  
                            inter_Flag = 0;
                    }  
                }  
                else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                {  
                    if (between(p3.X(),p1.X(),p2.X()))  
                    {  
                        double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                        line_x = p3.X();  
                        line_y = k*(line_x-p2.X())+p2.Y();  

                        if (between(line_y,p3.Y(),p4.Y()))  
                        {  
                            
                            inter_Flag = 1;
                            return true;  
                        }  
                        else  
                        {  
                            //return false;  
                            inter_Flag = 0;
                        }  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                }  
                else  
                {  
                    double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                    double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                    if (fabs(k1-k2)<1e-6)  
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                    else   
                    {  
                        line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                        line_y = k1*(line_x-p1.X())+p1.Y();  
                    }  

                    if (between(line_x,p1.X(),p2.X())&&between(line_x,p3.X(),p4.X()))  
                    {  
                        
                        inter_Flag = 1;
                        return true;  
                    }  
                    else   
                    {  
                        //return false;  
                        inter_Flag = 0;
                    }  
                } 
                
        }
        
                if(inter_Flag == 0)
                    return false;
               // if(inter_Flag == 1)
               //     return true;
}


//*** detect two positions are in same place ***//
bool TwoAdjacentPos(Point A, Point B)
{
    double distBetween = 0;
    double Xaxis = 0;
    double Yaxis = 0;
    
    Xaxis = B.X()-A.X();
    Yaxis = B.Y()-A.Y();
    distBetween = sqrt(Xaxis * Xaxis + Yaxis * Yaxis);
    
    if(distBetween < 200)
        return false;
    else
        return true;
        
}

void CombineLocalMaps(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
        MyRobot robot(0,0); 
        vector<Object> temp;
       //combine a new local map into global map                           
        
       double x1, y1, x2, y2;
       angle = ((angle / 180) * PI);

        for(int i=0;i<int(addView.size());i++)
        {
               
                x1=addView[i].X1()*cos(angle)-addView[i].Y1()*sin(angle)+coord.X();
                y1=addView[i].X1()*sin(angle)+addView[i].Y1()*cos(angle)+coord.Y();

                x2=addView[i].X2()*cos(angle)-addView[i].Y2()*sin(angle)+coord.X();
                y2=addView[i].X2()*sin(angle)+addView[i].Y2()*cos(angle)+coord.Y();
                
                Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());			
                s.setKP(1);
                temp.push_back(s);
        }
        
        //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/test-", num, ".png");            
        //plotObjects(LocalMFileName, currentPos, temp);
        
        currentTransLM = temp; 
        Memories.push_back(currentTransLM); // storing all transformed local maps;
        
        MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);
        sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");            
     //   plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}

void TransforIntoGloabl(vector<Object> addView, Point coord, vector<Object> currentPos, double angle, int num)
{
        MyRobot robot(0,0); 
        vector<Object> temp;
       //combine a new local map into global map                           
        
       double x1, y1, x2, y2;
       angle = ((angle / 180) * PI);

        for(int i=0;i<int(addView.size());i++)
        {
               
                x1=addView[i].X1()*cos(angle)-addView[i].Y1()*sin(angle)+coord.X();
                y1=addView[i].X1()*sin(angle)+addView[i].Y1()*cos(angle)+coord.Y();

                x2=addView[i].X2()*cos(angle)-addView[i].Y2()*sin(angle)+coord.X();
                y2=addView[i].X2()*sin(angle)+addView[i].Y2()*cos(angle)+coord.Y();
                
                Object s(x1, y1, x2, y2, addView[i].getID(), addView[i].nearness(), addView[i].getP1OS(), addView[i].getP2OS(), addView[i].getGID());			
                s.setKP(1);
                temp.push_back(s);
        }
        
        //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/test-", num, ".png");            
        //plotObjects(LocalMFileName, currentPos, temp);
        
        currentTransLM = temp; 
        Memories.push_back(currentTransLM); // storing all transformed local maps;
        
        MapofLMs = addTwoVectorsOfObjects(MapofLMs, temp);
        //sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");            
        //plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}

bool TwopathSegment(vector<Point> AllPositions)
{
        Point p1, p2, p3, p4;
        double line_x, line_y; //intersect position  
        int num_pos = 0;
        num_pos = AllPositions.size();
        
        p3 = AllPositions[num_pos-2];
        p4 = AllPositions[num_pos-1];
        
        if(AllPositions.size() > 3)
        {
                for(int i = 1; i < (AllPositions.size() - 2); i++)
                {
                        p1 = AllPositions[i-1];
                        p2 = AllPositions[i];


                        if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
                        {  
                            //return false;  
                            pathCrossed_Flag = 0;
                        }  
                        else if ( (fabs(p1.X()-p2.X())<1e-6) )
                        {  
                                if (between(p1.X(),p3.X(),p4.X()))  
                                {  
                                        double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
                                        line_x = p1.X();  
                                        line_y = k*(line_x-p3.X())+p3.Y();  

                                        if (between(line_y,p1.Y(),p2.Y()))  
                                        {  
                                            pathCrossed_Flag = 1;
                                            CrossedP1 = i-1;
                                            return true; 
                                        }  
                                        else  
                                        {  
                                            //return false;  
                                            pathCrossed_Flag = 0;
                                        }  
                                }  
                                else   
                                {  
                                        //return false;
                                    pathCrossed_Flag = 0;
                                }  
                        }  
                        else if ( (fabs(p3.X()-p4.X())<1e-6) ) 
                        {  
                                if (between(p3.X(),p1.X(),p2.X()))  
                                {  
                                    double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
                                    line_x = p3.X();  
                                    line_y = k*(line_x-p2.X())+p2.Y();  

                                    if (between(line_y,p3.Y(),p4.Y()))  
                                    {  
                                        pathCrossed_Flag = 1;
                                        CrossedP1 = i-1;
                                        return true;  
                                    }  
                                    else  
                                    {  
                                        //return false;
                                        pathCrossed_Flag = 0;
                                    }  
                                }  
                                else   
                                {  
                                    //return false;  
                                    pathCrossed_Flag = 0;
                                }  
                        }  
                        else  
                        {  
                                double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
                                double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  

                                if (fabs(k1-k2)<1e-6)  
                                {  
                                    //return false;
                                    pathCrossed_Flag = 0;
                                }  
                                else   
                                {  
                                    line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
                                    line_y = k1*(line_x-p1.X())+p1.Y();  
                                }  

                                if (between(line_x,p1.X(),p2.X())&&between(line_x,p3.X(),p4.X()))  
                                {  
                                    pathCrossed_Flag = 1;
                                    CrossedP1 = i-1;
                                    return true;  
                                }  
                                else   
                                {  
                                    //return false;  
                                    pathCrossed_Flag = 0;
                                }  
                        } 

                }
        }
        if(pathCrossed_Flag == 0)
            return false;
        
}


//given two slopes of two adjacent paths , tanA=|k1-k2|/|1+k1*k2|
// cos law a^2 = b^2 + c^2 -2*b*c*CosA   cosA=(b^2+c^2-a^2)/2bc

bool TwoPathAngle(vector<Point> AllPositions)
{
        cout << "*****This is going to calculate angle between two adjacent paths*****" << endl << endl;
        double k1, k2;
        double angleTwoPath;
        double a, b, c;
        double cosfi = 0, fi = 0, norm = 0;
        
        Point p1, p2, p3;
        int num = AllPositions.size();
        if(num >= 3)
        {
                //last three points
                p3 = AllPositions[num-1];
                p2 = AllPositions[num-2];
                p1 = AllPositions[num-3];
                
                b = sqrt((p1.X() - p2.X()) * (p1.X() - p2.X()) + (p1.Y() - p2.Y()) * (p1.Y() - p2.Y())); // b

                c = sqrt((p3.X() - p2.X()) * (p3.X() - p2.X()) + (p3.Y() - p2.Y()) * (p3.Y() - p2.Y())); // c

                a = sqrt((p3.X() - p1.X()) * (p3.X() - p1.X()) + (p3.Y() - p1.Y()) * (p3.Y() - p1.Y())); // a

                
                //k1 = (p2.Y()-p1.Y()) / (p2.X() - p1.X());
                //k2 = (p2.Y()-p3.Y()) / (p2.X() - p3.X());
                
                //angleTwoPath = atan((abs(k1 - k2)) / (abs(1+ k1 * k2)));
                
                //cosfi = dsx * dex + dsy * dey;
                //norm = (dsx * dsx + dsy * dsy) * (dex * dex + dey * dey);
                //cosfi /= sqrt(norm);
                
                //if (cosfi >= 1.0) return 0;
                //if (cosfi <= -1.0) return PI;
                
                cosfi = (b * b + c * c - a * a) / (2 * b * c);
                
                fi = acos(cosfi);
               
                
                if (180 * fi / PI < 180)     
                {
                    angleTwoPath =  180 * fi / PI;
                }
                else
                {
                    angleTwoPath =  360 - 180 * fi / PI;
                } 
                
                cout << "This is for testing angle calculation" << endl << endl;
                cout << "The parameter angleTwoPath/fi is : " << angleTwoPath << endl << endl;
                //waitHere();
         }
    

    
        if(angleTwoPath < 45 && angleTwoPath != 0)
            return true;
        else
            return false;
}

//**************************************************//
// Transform current position onto old coordinate system
Point TransPointToOldCrd(double Transf_angle, Point Transf_point, Point coord)
{
        cout << "This is going to transform current robot position onto pre coordinate" << endl << endl;
    
        double angle = 0;
        double x, y;
        Point point;
        Point temp;


        angle = Transf_angle;
        point = Transf_point;

        angle = ((angle / 180) * PI);


        x = point.X() * cos(angle) - point.Y() * sin(angle) + coord.X();
        y = point.X() * sin(angle) + point.Y() * cos(angle) + coord.Y();

        temp.set(x, y);
    
        return temp;
}

void TwoPathsCrossed(int num)
{
//        cout << "This is going to process two path segments" << endl;
//        cout << "Path Crossed Flag : " << (int)pathCrossed_Flag << endl;
//        //waitHere();
//
//        // delete useless positions
//        robotPositionsInLMs.clear();
//        for(int i = RobotPs.size() - 1; i > CrossedP1; i--)
//             RobotPs.pop_back();
//        RobotPs.push_back(Point (currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1()));
//
//         // delete useless positions, which are used to present on map
//        for(int i = ListOfPositions.size() - 1; i > CrossedP1; i--)
//             ListOfPositions.pop_back();
//        ListOfPositions.push_back(currentRobotPositionInMFIS);
//
//        // re-build robot positions
//        for(int i = 0; i < ListOfPositions.size() ; i++)
//             robotPositionsInLMs = addTwoVectorsOfObjects(robotPositionsInLMs, ListOfPositions[i+1]);
//
//        // this part is to delete ASRs, which are useless
//        for(int i = Memories.size() - 1; i > CrossedP1; i--)
//             Memories.pop_back();
//        Memories.push_back(currentTransLM);
//
//         // re-build global map using local maps
//        MapofLMs.clear(); // clear Global map and re-compute it
//        for(int i = 0; i < Memories.size() ; i++)
//             MapofLMs = addTwoVectorsOfObjects(MapofLMs, Memories[i]);
//
//
//        // re-plot this current local map
//        sprintf(LocalMFileName, "%s%d%s", "Maps/Offline/Globalmap-", num, ".png");            
//        plotObjectsOf3Kinds(LocalMFileName, robotPositionsInLMs, MapofLMs, robot.getRobot());
}


//********************************************************************************//
// Test Programme for transforming positions onto pre coordinate
vector<Object> TransPositions(vector<Object> positions, double TransAngle, Point coord)
{
        MyRobot robot(0,0); 
        vector<Object> temp;
        Object result;
       //combine a new local map into global map                           
        
       double x1, y1, x2, y2;
       double angle = TransAngle;
      
       angle = ((angle / 180) * PI);

        for(int i=0;i<int(positions.size());i++)
        {
               
                x1=positions[i].X1()*cos(angle)-positions[i].Y1()*sin(angle)+coord.X();
                y1=positions[i].X1()*sin(angle)+positions[i].Y1()*cos(angle)+coord.Y();

                x2=positions[i].X2()*cos(angle)-positions[i].Y2()*sin(angle)+coord.X();
                y2=positions[i].X2()*sin(angle)+positions[i].Y2()*cos(angle)+coord.Y();
               
                Object result(x1,y1,x2,y2, 1, 0, 0, 0, 1);
                temp.push_back(result);
        }
       
       return temp;
}

