#include <iostream>
#include <vector>
#include "Object.H"
#include "asr.H"

#include "Map.H"

//************************** Map Methods *************************

ASRNetwork::ASRNetwork(vector<ASR> allASRs) {
    asrs = allASRs;
}

vector<ASR> ASRNetwork::getASRs() {
    return asrs;
}
void ASRNetwork::setASRs(vector<ASR> allASRs) {
    asrs = allASRs;
}

int ASRNetwork::getWhereAmI() {
    return whereAmI;
}

void ASRNetwork::setWhereAmI(int a) {
    whereAmI = a;
}

bool ASRNetwork::getCloseTheLoop() {
    return closeTheLoop;
}

void ASRNetwork::setCloseTheLoop(bool a) {
    closeTheLoop = a;
}

 bool ASRNetwork::getLeftHome() {
     return leftHome;
 }
 void ASRNetwork::setLeftHome(bool a) {
     leftHome = a;
 }
    
 bool ASRNetwork::getArrivedHome() {
     return arrivedHome;
 }
 void ASRNetwork::setArrivedHome(bool a) {
     arrivedHome = a;
 }
 
 bool ASRNetwork::getOldExitRecognization() {
     return oldExitRecognized;
 }
 
 void ASRNetwork::setOldExitRecognization(bool a) {
     oldExitRecognized = a;
 }
 
 bool ASRNetwork::isLoopClosed() {
     return loopClosed;
 }
 void ASRNetwork::setLoopClosed(bool a) {
     loopClosed = a;
 }
 
 ASR ASRNetwork::getCurrentASR() {
     return currentASR;
 }
    void ASRNetwork::setCurrentASR(ASR cASR) {
        currentASR = cASR;
       // currentASR.setASRExit1(cASR.getASRExit1());
        //currentASR.setASRExit2(cASR.getASRExit2());
        //currentASR.replaceTheWholeRoute(cASR.getRoute());
        
    }
    
    vector<Object> ASRNetwork::getCurrentView() {
        return currentView;
    }
    void ASRNetwork::setCurrentView(vector<Object> cview) {
        currentView = cview;
    }
    
        Object ASRNetwork::getCurrentRobotPosition() {
            return currentRobotPosition;
        }
    void ASRNetwork::setCurrentRobotPosition(Object cRobotPosition) {
        currentRobotPosition = cRobotPosition;
    }
    
    void ASRNetwork::addASR(ASR oneASR) {
        asrs.push_back(oneASR);
    }
    vector<ASR> ASRNetwork::getAllASRs() {
        vector<ASR> allASRs;
        allASRs = asrs;
        allASRs.push_back(currentASR);
        return  allASRs;
    }
        void ASRNetwork::setAllCurrentViews(vector<vector<Object> > allviews) {
            allCurrentViews = allviews;
        }
    vector<vector<Object> > ASRNetwork::getAllCurrentViews(){
        return allCurrentViews;
    }
   void ASRNetwork::setAllRobotPositions(vector<vector<Object> >  allrobotpositions){
       allRobotPositions = allrobotpositions;
   }
    vector<vector<Object> > ASRNetwork::getAllRobotPositions() {
        return allRobotPositions;
    }
    
    void ASRNetwork::setAllReferenceObjects(vector<vector<Object> > allreferenceobjects) {
        allReferenceObjects = allreferenceobjects;
    }
    vector<vector<Object> > ASRNetwork::getAllReferenceObjects() {
        return allReferenceObjects;
    }
    
    
    void ASRNetwork::setMFIS(vector<Object> mfisObj) {
        mfis = mfisObj;
    }
    vector<Object> ASRNetwork::getMFIS() {
        return mfis;
    }