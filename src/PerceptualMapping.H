/* 
 * File:   PerceptualMapping.h
 * Author: mhossain
 *
 * Created on 4 October 2012, 8:31 PM
 */

#ifndef PERCEPTUALMAPPING_H
#define	PERCEPTUALMAPPING_H

#include <vector>


#include "Transporter.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "mfisOp.H"
#include "CompareASR.H"
#include "Mapping.H"
#include "PointAndSurface.H"
#include "thesis.H"

using namespace std;

Transporter updatePerceptualMap(vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, vector<Object> refobjects,int viewNumber, int ASRNumber);
Transporter updatePerceptualMapATPlace(vector<ASR> places,vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, 
        vector <Object> allRobotPositions,vector<Object> refobjects, int viewNumber, int ASRNumber, 
        bool exitCrossed, vector<Object> crossedExit,vector<int> limitingPoints, vector<Object> routeMap4CPlace);

void updateMapAtLastStep(Transporter lastStepInfo);
bool isBelongToSameSpace(int limitingPoint, vector<int> spaceIDs);
bool isBelongToSameSpace(vector<int> limitingPoint, vector<int> spaceIDs);

bool isThisIDAdded(int newID, vector<int> previousIDs);

//return true if thisObject close to current view
bool isThisCloseToCurrentView(Object thisObject, vector<Object> currentView);

bool isThisCloseToCVPolygonBoundary(Object thisObject, vector<Object> polygonObjects, double distTh);

bool findCrossedExit(vector<Object> & allCrossedExit, vector<Object> lastView, 
        vector<Object> refObjects, vector<double> distanceAngle, int set);

void updatePerceptualMapUsingPlaceInformation(vector<ASR> & places, vector<Object> & MFIS, 
        vector<Object> cView, vector<Object> currentRobotPositionInMFIS,vector <Object> allRobotPositions, 
        vector<Object> refobjects, int viewNumber, int ASRNumber, bool exitCrossed, 
        vector<Object> crossedExit, vector<Object> & targetObjectsInPV, vector<Object> refObjectForLoopClosing,
        int updatingASRNumber,vector<Object> routeMap4CPlace);

void findBestReferenceObjectsUsingOdometryInfo(vector<Object> & referenceObjects,
        vector< vector<Object> > allRPoseAtOneStep, vector<Object> currentRobotPositionInMFIS,
        vector<double> coordTransInfo);

void findRouteMap(vector<Object> MFIS,vector<vector<Object> > allRobotPose);

//it uses PM to see where current robot position is reachable directly. Date - 21-10-14
void updateRouteMap(vector<Object> & entireRouteMap,Object cRobotPosition,vector<Object> MFIS);

#endif	/* PERCEPTUALMAPPING_H */

