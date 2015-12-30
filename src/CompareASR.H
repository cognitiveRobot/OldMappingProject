
/* it's comapre two set of lines which belong to two ASR(or two map)
 Theory: Godefroy
 Implemented by: Hossain
 */

#ifndef COMPAREASR_H
#define COMPAREASR_H

#include <iostream>
#include <vector>

#include "asr.H"
#include "Object.H"
#include "Map.H"
#include "Transporter.H"

using namespace std;


//vector<Object> findSameObjectsFromTwoASRs(ASRNetwork pm, ASR oldASR, ASR newASR, Object rPositionOfNewASR);
Transporter findSameObjectsFromTwoASRs(ASRNetwork pm, ASR oldASR, ASR newASR, Object rPositionOfNewASR,Point currentRobotPosition);
vector<Object> findSameObjectsFromTwoViews(vector<Object> oldView, vector<Object> newView, double dist, double angle);
Transporter compareObjects(vector<Object> newObjects, vector<Object> oldObjects);
vector<Object> deleteSmallObjects(vector<Object> objects);
double shortestDistanceBtwTwoObjects(Object old,Object new1);
Transporter adjustNewASRsAfterLoopClosing(vector<Object> refObjects, vector<ASR> allASRs);
Transporter mapOldASRsIntoNewASRsAfterLoopClosing(vector<Object> refObjects,vector<ASR> allASRs);
Transporter makeNewPerceptualMap(vector<Object> refObjects,vector<ASR> allASRs);
void makeFinalASR(ASRNetwork pm, vector<Object> referenceObjects);
ASR mergeOldandNewASRs(ASR oldASR, ASR newASR);
ASR mergeOldandNewASRsUsingAngleAndLoS(ASR oldASR, ASR newASR);
void makeFinalPMUsingOldASRs(ASRNetwork pm,vector<Object> fAndLRP);
vector<Object> findObjectsForRotatingTwoViews(vector<Object> oldView, vector<Object> newView, Object exit);
void perceptualMapUptoCurrentView(ASRNetwork pm,vector<ASR> ASRs);






#endif