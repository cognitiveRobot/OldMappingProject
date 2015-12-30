#include <iostream>
#include <vector>
#include <cmath>

#include "Object.H"
#include "CompareASRClass.H"

using namespace std;

//used to find neighbor possible robot position
#define distanceError 100;
#define angleError 5; 


PossibleRobotPosition::PossibleRobotPosition(Object obj) {
    possibleRobotPosition = obj;
}

void PossibleRobotPosition::setPossibleRobotPosition(Object obj) {
    possibleRobotPosition = obj;
}
Object PossibleRobotPosition::getPossibleRobotPosition() {
    return possibleRobotPosition;
}

    void PossibleRobotPosition::setObjectOfOldASR(Object obj) {
        objectOfOldASR = obj;
    }
    Object PossibleRobotPosition::getObjectOfOldASR() {
        return objectOfOldASR;
    }
    
    void PossibleRobotPosition::setObjectOfNewASR(Object obj) {
        objectOfNewASR = obj;
    }
    Object PossibleRobotPosition::getObjectOfNewASR() {
        return objectOfNewASR;
    }
double PossibleRobotPosition::getSqDistanceFromP1ToP1(PossibleRobotPosition prp) {    
    double a;
    a = (possibleRobotPosition.X1() - prp.getPossibleRobotPosition().X1());
    if(a > 100)
        return 10001;
    else {
        a = a*a;
        if(a > 10001)
            return a;
        else {
            double b = (possibleRobotPosition.Y1() - prp.getPossibleRobotPosition().Y1());
            b = b*b;
            return a + b;
        }
    }
 
}
double PossibleRobotPosition::getAngleWith(PossibleRobotPosition prp) {
    double angle = possibleRobotPosition.getAngleWithLine(prp.getPossibleRobotPosition());
    return angle;
}
    
vector<Object> PossibleRobotPosition::convertAllPossibleRobotPositions(vector<PossibleRobotPosition> allprps) {
    vector<Object> objects;
    for(int i=0;i<int(allprps.size());i++) {
        objects.push_back(allprps[i].getPossibleRobotPosition());
    }
    return objects;
}