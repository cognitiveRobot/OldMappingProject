#ifndef _COMPAREASRCLASS_H_
#define _COMPAREASRCLASS_H_

#include <iostream>
#include <vector>

#include "Object.H"

using namespace std;

class PossibleRobotPosition {
private:
    Object possibleRobotPosition;
    /*this possible robot position is calculated using these two ASR objects*/
    Object objectOfOldASR;
    Object objectOfNewASR;
public:

    PossibleRobotPosition() {
    };
    PossibleRobotPosition(Object obj);

    ~PossibleRobotPosition() {
    };
    
    
    void setPossibleRobotPosition(Object obj);
    Object getPossibleRobotPosition();
    
    void setObjectOfOldASR(Object obj);
    Object getObjectOfOldASR();
    
    void setObjectOfNewASR(Object obj);
    Object getObjectOfNewASR();
    
    double getSqDistanceFromP1ToP1(PossibleRobotPosition prp);
    double getAngleWith(PossibleRobotPosition prp);
    
    
    vector<Object> convertAllPossibleRobotPositions(vector<PossibleRobotPosition> allprps);
    
};


















#endif
