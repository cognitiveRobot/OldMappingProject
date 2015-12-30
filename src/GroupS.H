//		 GroupS Class	:	A group of Objects
//			by mhossain


#ifndef _GROUPS_H
#define	_GROUPS_H

#include <vector>
#include "Point.H"
#include "Object.H"
using namespace std;

class GroupS
{
private:
    double x1;
    double y1;
    double x2;
    double y2;
    int GID;
   	vector<Object> Objects;
	
	int ms;
        Point averagePoint;
    
public:
    GroupS(){};
    GroupS(double X1, double Y1, double X2, double Y2, int gid, vector<Object> Objects);
    GroupS(double X1, double Y1, double X2, double Y2,  int gid, int mss, vector<Object> Objects);
    GroupS(double X1, double Y1, double X2, double Y2, vector<Object> Objects);
    ~GroupS(){};

    int getGID();
    void setGID(int gid);
	int getNoS();
	
	void setSP(double a, double b);
	void setEP(double a, double b);
	double length();
	void setObjects(vector<Object> objects);
	vector<int> getObjectID();
        
        void setAveragePoint(double x, double y);
        Point getAveragePoint();
	
    double SPX();
    double SPY();
    double EPX();
    double EPY();
	void setMS(int a);
	int getMS();
	vector<Object> getObjects();
	void display();
	double distP1ToP1(GroupS s);
	double distP2ToP2(GroupS s);
	double distP1ToPoint(double a, double b);
	double distP2ToPoint(double a, double b);
	double getAngleWithXaxis();
	double getAngleWithGroup(GroupS s);
	double isP1MatchedWith(GroupS s, double ox, double oy);
	double isP2MatchedWith(GroupS s, double ox, double oy);
	double isMidpointMatchedWith(GroupS s, double ox, double oy);
	double perpendicularDistOfPoint(double x3, double y3);
  
};
vector<GroupS> makeGroup(vector<Object> Objects);
vector<GroupS> makeGroupIFoR(vector<Object> Objects);
void displayGroups(vector<GroupS> gs);
GroupS getGroupS(vector<GroupS> Objects, int id);
vector<GroupS> deleteGroup(vector<GroupS> groups, int dg);
//vector<int> getNewLinesID(vector<GroupS> cpr_cv, vector<int> g_common_lines);

vector<Object> updateUsingAveragePoint(vector<Object> MFIS, vector<Object> cView);
#endif
