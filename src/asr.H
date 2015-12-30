#ifndef _ASR_H_INCLUDE
#define _ASR_H_INCLUDE



#include <iostream>
#include <vector>

#include "Object.H"

using namespace std;

class ASR {
private:
    int id;
    vector<Object> objects;
    Object exits;
    Object exit2;
    vector<Object> route;
    vector<Object> limitingPoints;
    vector<Object> lineOfSitePoints;
    Object startPoint;
    Object endPoint;
    
public:

    ASR() {
    };
    ASR(vector<Object> objs);

    ~ASR() {
    };

    int getASRID();
    void setASRID(int a);
    vector<Object> getASRObjects();
    void setASRObjects(vector<Object> objs);
    Object getASRExit1();
    void setASRExit1(Object exs);
    Object getASRExit2();
    void setASRExit2(Object exs);
    void replaceASR(ASR newasr);
    void updateRoute(Object robotpath); //update route after everysteps
    vector<Object> getRoute();
    void replaceTheWholeRoute(vector<Object> newRoute);
    void clearRoute();
    void addLimitingPoint(Object lpoint);
    void setLimitingPoints(vector<Object> lpoints);
    vector<Object> getLimitingPoints();
    void addLineOfSitePoints(Object lineOfSitePoint);
    void setLineOfSitePoints(vector<Object> lOfSitePoints);
    vector<Object> getLineOfSitePoints();
    void setStartPoint(Object sPoint);
    Object getStartPoint();
    void setEndPoint(Object ePoint);
    Object getEndPoint();
};


#endif
