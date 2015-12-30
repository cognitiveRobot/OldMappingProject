/*
    Transporter, it follows three rules
    Rule#1: Deal's the deal.
    Rule#2: No names.
    Rule#3: Never look in the package.

        Designed by........
                        MHossain
                        10.07.2011
                        Inspired by......The Transporter(2002)
 */

#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"
#include "PathPlanning.H"

using namespace std;

#ifndef _TRANSPORTER_H_
#define _TRANSPORTER_H_

class Transporter {
private:
    vector<Object> MFIS;
    vector<vector<Object> > views;
    vector<Object> view; //vector of objects
    vector<Object> refObjects; //vector of ref objects
    vector<Object> tObjects; //vector of target objects--- landmark objects
    vector<Object> refObjectsForLoopClosing;
    vector<Object> robotPosition;
    vector<Object> allRobotPositions;
    vector<Exit> exits;
    vector<ASR> ASRs;
    ASR asr;
    bool lostSituation;

public:

    Transporter() : lostSituation(false) {
    };

    ~Transporter() {
    };
    
    void setViews(vector<vector<Object> > moreviews);
    vector<vector<Object> > getViews();
    void setView(vector<Object> cv);
    vector<Object> getView();
    void setReferenceObjects(vector<Object> refobs);
    vector<Object> getReferenceObjects();
    void setTargetObjects(vector<Object> tobs);
    vector<Object> getTargetObjects();
    void setASRs(vector<ASR> asrs);
    vector<ASR> getASRs();
    void setExits(vector<Exit> ex);
    vector<Exit> getExits();    
    void setASR(ASR oneasr);
    ASR getASR();
    void setLostSituation(bool lost);
    bool getLostSituation();
    
    void setReferenceObjectsForLoopClosing(vector<Object> refobs);
    vector<Object> getReferenceObjectsForLoopClosing();
    
    void setRobotPosition(vector<Object> rp);
    vector<Object> getRobotPosition();
    
    void setAllRobotPositions(vector<Object> aRP);
    vector<Object> getAllRobotPositions();

    void setMFIS(vector<Object> mfis);
    vector<Object> getMFIS();
};


#endif
