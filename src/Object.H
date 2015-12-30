//		 Object Class
//			by Jhon
//			modified by mhossain


#ifndef _Object_H
#define	_Object_H

#include <vector>
#include "PointAndSurface.H"
#include "Point.H"

using namespace std;



class Object {
private:
    int ID;
    double x1;
    double y1;
    double x2;
    double y2;
    
    int NS;
    int GID;
    int P1OS; //p1 occluding status
    int P2OS;
    bool PEP1; //probable exit status
    bool PEP2;
    double Ort; //orientation angle
    double distance;
    int kp; //Key point 1=p1, 2=p2, 3=both
    bool po; //potential objects
    int pos; //position of this line -1=left +1=right
    bool OoPV; //Objects of previous view
    int vn; //view number
    int asrno;
    int limitingPoint;
    vector<int> localEnvID;
    int colorID;

public:

    Object() {};
    
    Object(double X1, double Y1, double X2, double Y2, int id, int ns, int p1os, int p2os, int gid);
    Object(double X1, double Y1, double X2, double Y2, int id, int ns);
    Object(double X1, double Y1, double X2, double Y2);
    Object(double X1, double Y1, double X2, double Y2, int id);

    ~Object() {};

    int getID();
    void setID(int id);
    int getGID();
    void setGID(int gid);
    double X1() const;
    double Y1() const;
    double X2() const;
    double Y2() const;

    double mpX();
    double mpY();

    void set(double a1, double b1, double a2, double b2, int id);
    void set(Object s);
    //void convertExitToObject(Exit e);
    void setP1(double a1, double b1);
    void setP2(double a1, double b1);
    int getP1OS();
    int getP2OS();
    void setP1OS(int a);
    void setP2OS(int a);
    double length() const;
    double distP1ToPoint(double a, double b);
    double distP2ToPoint(double a, double b);
    double distMPToPoint(double a, double b);


    void replace(Object s);
    double distP1ToP1(Object s);
    double distP2ToP2(Object s);
    double distP1ToP2(Object s);
    double distP2ToP1(Object s);

    void setPEP1(bool p1pe);
    void setPEP2(bool p2pe);
    bool getPEP1();
    bool getPEP2();


    void setOrt(double ang);
    double getOrt();
    
    void setDistance(double dist);
    double getDistance();

    void setKP(int a);
    int getKP();

    void setPO(bool a);
    bool getPO();

    void setPos(int a);
    int getPos();

    void display();
    Point midpoint();
    int nearness();
    void setNS(int a);

    void setOoPV(bool a);
    bool getOoPV();

    void setVN(int a);
    int getVN();

    void setASRNo(int a);
    int getASRNo();
    
    void setLimitingPoint(int a);
    int getLimitingPoint();
    
    void setLocalEnvID(int a);
    void setLocalEnvID(vector<int> a);
    vector<int> getLocalEnvID();
    
    void setColorID(int a);
    int getColorID();

    double ppLocOfPoint(double x3, double y3);
    Point ppCordOfPoint(double x3, double y3);
    double perpendicularDistOfPoint(double x3, double y3);
    double distP1ToPPOfPoint(double x3, double y3);
    double isP1ToBeExtended(Object s1);
    double isP2ToBeExtended(Object s1);
    double shortestDistP1ToP1(Object s1);
    Point shortestDistPointFrom(double a, double b);
    double shortestDistFrom(double a, double b);

    double getAngleWithXaxis();
    double getAngleWithPoint(double a, double b);
    double getAngleWithPointFP1E(double a, double b);
    double getAngleWithLine(Object s);
    double getAngleWithPointForASR(double a, double b);
    
    
    double getAngleFromP1ToPoint(const double & a, const double & b) const;
    double getAngleFromP2ToPoint(const double & a, const double & b) const;
    
    double distFromP1ToPoint(const float & a, const float & b) const;
    double distFromP2ToPoint(const float & a, const float & b) const;
    
    

    bool isP1MatchedWith(Object s);
    bool isP1MatchedWith(Object s, Object rp);
    bool isMidpointMatchedWith(Object s, Object rpc);
    bool isP2MatchedWith(Object s);
    bool isMidpointMatchedWith(Object s);
    bool isOverlappedWith(Object s, double rx, double ry);
    bool isInsideObject(Object s, Object crp);
    
    //created on 26 Jan, 2012. used to update MFIS/ASR while robot moves randomly 
    double shortestDistanceWithObject(Object old);
    bool isThisInsideCV(vector<double> boundaries);
    bool isThisOverlappingObject(vector<Object> currentView);    
    
    
    void reverse();

    /////for exit
    double whereIsExit(Object s);
};

bool MyDataSortPredicate(Object d1, Object d2);
bool sortA2L(Object d1, Object d2);
bool sortA2MPDistanceFromOrigin(Object d1, Object d2);
bool sortA2OrtAngleL2R(Object d1, Object d2);//sort objects according 2 orientation angle (ort) from left to right
bool sortA2OrtAngleR2L(Object d1, Object d2);//sort objects according 2 orientation angle (ort) from right to left
bool sortA2Distance(Object d1, Object d2);//sort objects according 2 distance parameter 

typedef vector<Object> Objects;

inline Objects& operator <<(Objects& objs, const Object& obj) {objs.push_back(obj); return objs;}
inline Objects& operator +(Objects& objs, const Objects& obj2) {
    for(unsigned int i=0; i<obj2.size(); i++)
        objs.push_back(obj2[i]); 
    return objs;
}

vector<Object> setOccludingEdges(vector<Object> Objects);
int getIndex(vector<Object> Objects, int id);
Object getObject(vector<Object> Objects, int id);
vector<Object> deleteObject(vector<Object> Objects, int dsur);
vector<Object> removeSmallObjects(vector<Object> mfis);
vector<Object> considerLongObjects(vector<Object> Objects, double per);
void displayObjects(vector<Object> Objects);
vector<double> Objects2Vector(vector<Object> Objects);
vector<Object> addTwoVectorsOfObjects(vector<Object> first, vector<Object> second);
vector<Object> convertSurfaceToObject(vector<Surface> surfs);
vector<Surface> convertObjectToSurface(vector<Object> surfs);

Object convertSurfaceToObject(Surface surf);



//my robot class

/* albot1's actual dimension:
                length- 40.35 cm
                width-  30.8 cm
 */

class MyRobot {
private:
    vector<Object> sides;
    //vector<vector<Object> > allrobotpos;
public:

    MyRobot() {
    };
    MyRobot(double a, double b);

    ~MyRobot() {
    };

    vector<Object> getRobot();
    vector<Object> inMFIS(Object rmfis, Object rcv, int refpoint);
    vector<Object> allRobPos(vector<vector<Object> > allrobotpos);
    //vector<Object> specificRobotPosition(
    
    vector<Object> robotPathForNextDest(double angle, double distance);
};

class Obstacle {
private:
    int id;
    double dist;
    double angle;
public:

    Obstacle() {
    };
    Obstacle(int i, double d, double a);

    ~Obstacle() {
    };

    void display();
    int getID();
    double getAngle();
    double getDist();
};
void displayObstacles(vector<Obstacle> obs);
bool sortObsA2D(Obstacle a, Obstacle b);


#endif
