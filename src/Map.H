#ifndef _MAP_H_INCLUDE
#define _MAP_H_INCLUDE



#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"

using namespace std;

class ASRNetwork {
private:
    vector<ASR> asrs;
    int whereAmI;
    bool closeTheLoop;
    bool leftHome;
    bool arrivedHome;
    bool oldExitRecognized;
    bool loopClosed;
    ASR currentASR;
    vector<Object> currentView;
    Object currentRobotPosition;
    vector<vector<Object> > allCurrentViews;
    vector<vector<Object> > allRobotPositions;
    vector<vector<Object> > allReferenceObjects;
    vector<Object> mfis;
public:

    ASRNetwork()  : whereAmI(1), closeTheLoop(false), leftHome(false), arrivedHome(true), oldExitRecognized(false), loopClosed(false){
    };
    ASRNetwork(vector<ASR> allASRs);

    ~ASRNetwork() {
    };
    
    vector<ASR> getASRs();
    void setASRs(vector<ASR> allASRs);

    int getWhereAmI();
    void setWhereAmI(int a);

    bool getCloseTheLoop();
    void setCloseTheLoop(bool a);
    
    bool getLeftHome();
    void setLeftHome(bool a);
    
    bool getArrivedHome();
    void setArrivedHome(bool a);
    
    bool getOldExitRecognization();
    void setOldExitRecognization(bool a);
    
    bool isLoopClosed();
    void setLoopClosed(bool a);
    
    ASR getCurrentASR();
    void setCurrentASR(ASR cASR);
    
    vector<Object> getCurrentView();
    void setCurrentView(vector<Object> cview);
    
    Object getCurrentRobotPosition();
    void setCurrentRobotPosition(Object cRobotPosition);
    void addASR(ASR oneASR);
    vector<ASR> getAllASRs();//returns ASRs including current one
    //void setCurrentASRID(int a);
    //void setCurrentASR
    
    void setAllCurrentViews(vector<vector<Object> > allviews);
    vector<vector<Object> > getAllCurrentViews();
    
    void setAllRobotPositions(vector<vector<Object> >allrobotpositions);
    vector<vector<Object> > getAllRobotPositions();
    
    void setAllReferenceObjects(vector<vector<Object> > allreferenceobjects);
    vector<vector<Object> > getAllReferenceObjects();
    
    void setMFIS(vector<Object> mfisObj);
    vector<Object> getMFIS();
};


#endif