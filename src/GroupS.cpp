//		 GroupS Class
//			by mhossain


#include <iostream>
#include "Object.H"
#include "GroupS.H"
#include "mfisOp.H"
#include "Plotting.H"
#include <cmath>

using namespace std;
#define PI 3.14159265

GroupS::GroupS(double X1, double Y1, double X2, double Y2, int gid, vector<Object> Objects)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    GID = gid;
   Objects=Objects;
    	ms=0;
}

GroupS::GroupS(double X1, double Y1, double X2, double Y2, int gid, int mss, vector<Object> Objects)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    GID = gid;
  	Objects=Objects;
	ms=mss;
    
}


GroupS::GroupS(double X1, double Y1, double X2, double Y2, vector<Object> Objects)
{
    x1 = X1; y1 = Y1;
    x2 = X2; y2 = Y2;
    GID = -1;
   
	Objects=Objects;
   	
	ms=0;
}

int GroupS::getGID()
{
    return GID;
}



void GroupS::setGID(int gid)
{
  GID = gid;
}

int GroupS::getNoS()
{
    return Objects.size();
}


void GroupS::setSP(double a, double b)
{
	x1=a; y1=b;
}

void GroupS::setEP(double a, double b)
{
	x2=a; y2=b;
}

void GroupS::setMS(int a)
{
	ms=a;
}

int GroupS::getMS()
{
	return ms;
}


double GroupS::length()
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


vector<Object> GroupS::getObjects()
{
    return Objects;
}

void GroupS::setObjects(vector<Object> objects)
{
	Objects=objects;
}

vector<int> GroupS::getObjectID()
{
	vector<int> result;
	for(int i=0;i<int(Objects.size());i++)
	{
		result.push_back(Objects[i].getID());
	}
	return result;
}

Point GroupS::getAveragePoint() {
    return averagePoint;
}
void GroupS::setAveragePoint(double x, double y) {
    averagePoint = Point(x,y);
}


double GroupS::SPX(){return x1;}
double GroupS::SPY(){return y1;}
double GroupS::EPX(){return x2;}
double GroupS::EPY(){return y2;}

void GroupS::display()
{
	cout<<"Gid: "<<GID<<" SP X: "<<x1<<" Y: "<<y1<<" ----------- EP X: "<<x2<<" Y: "<<y2<<" NoS: "<<getNoS()<<" MS: "<<ms<<" s.ids:::";
	for(int i=0;i<int(Objects.size());i++)
	cout<<" "<<Objects[i].getID();
	cout<<endl;
}

//returns distance between point1 of two Objects
double GroupS::distP1ToP1(GroupS s)
{
	return sqrt((x1-s.SPX())*(x1-s.SPX())+(y1-s.SPY())*(y1-s.SPY()));
}

//returns distance between point 2 of two Objects
double GroupS::distP2ToP2(GroupS s)
{
	return sqrt((x2-s.EPX())*(x2-s.EPX())+(y2-s.EPY())*(y2-s.EPY()));
}

double GroupS::distP1ToPoint(double a, double b)
{

	return sqrt((x1-a)*(x1-a)+(y1-b)*(y1-b));

}

double GroupS::distP2ToPoint(double a, double b)
{

	return sqrt((x2-a)*(x2-a)+(y2-b)*(y2-b));

}
double GroupS::getAngleWithXaxis()
{
	double x21=x2-x1;
	double y21=y2-y1;
	
	double angle1=acos(x21/length());

	if(y21 < 0)
	angle1 = 2*PI - angle1;	

	return ((180/PI)*angle1);
}


double GroupS::getAngleWithGroup(GroupS s)
{
	
	double angdiff=s.getAngleWithXaxis()-getAngleWithXaxis();
	return angdiff;
}

//last two arguments carries origin information
double GroupS::isP1MatchedWith(GroupS s, double ox, double oy)
{
	double a=ox; //x1
	double b=oy; //y1
	double c=x1; //x2
	double d=y1; //y2
	
	double x3= s.SPX();
	double y3= s.SPY();
	double x4= s.EPX();
	double y4= s.EPY();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/

	double result;
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double dist=s.distP2ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<" ms- "<<s.getMS()<<endl;
		if(dist > 80)
		{
		
		result = min(distP1ToPoint(x,y), s.perpendicularDistOfPoint(x1,y1));
		}
		else 	
		result=-1;
		
	}
	else 
	result= -1;

	return result;
}

double GroupS::isP2MatchedWith(GroupS s, double ox, double oy)
{
	double a=ox; //x1
	double b=oy; //y1
	double c=x2; //x2
	double d=y2; //y2
	
	double x3= s.SPX();
	double y3= s.SPY();
	double x4= s.EPX();
	double y4= s.EPY();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	double result;
	
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		if(dist > 80)
		result = distP2ToPoint(x,y);
		else 	
		result=-1;
	}
	else 
	result= -1;

	return result;
}

double GroupS::isMidpointMatchedWith(GroupS s, double ox, double oy)
{
	double a=ox; //x1
	double b=oy; //y1
	double c=(x1+x2)/2; //x2
	double d=(y1+y2)/2; //y2
	
	double x3= s.SPX();
	double y3= s.SPY();
	double x4= s.EPX();
	double y4= s.EPY();

	double y43= y4-y3;
	double x21= c-a;
	double x43= x4-x3;
	double y21= d-b;

	double u_deno=y43*x21-x43*y21;
	
	double y13= b-y3;
	double x13= a-x3;
	//double ua_num= x43*y13-y43*x13;
	double ub_num= x21*y13-y21*x13;

	//double ua=ua_num/u_deno;
	double ub=ub_num/u_deno;

	/*if(ub>0&&ub<1)
	return 1;
	else
	return 0;*/
	double result;
	
	if((ub >= 0) && (ub < 1))
	{
		double ua_num= x43*y13-y43*x13;
		double ua=ua_num/u_deno;
		
		double x=a+ua*x21;
		double y=b+ua*y21;
		//double dist=s.distP1ToPoint(x, y);
		//cout<<"g-id "<<s.getGID()<<" dist- "<<dist<<endl;
		//if(dist > 80)
		result = sqrt(((c-x)*(c-x))+((d-y)*(d-y)));
		//else 	
		//result=-1;
	}
	else 
	result= -1;

	return result;
}

//it returns perpendicular distance of given point and line
double GroupS::perpendicularDistOfPoint(double x3, double y3)
{
	double x31=x3-x1;
	double x21=x2-x1;
	double y31=y3-y1;
	double y21=y2-y1;
	
	double length=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));	

	double r=((x31*x21)+(y31*y21))/(length*length);
	
	double x=x1+(r*x21);
	double y=y1+(r*y21);
	
	return sqrt((x-x3)*(x-x3)+(y-y3)*(y-y3));
}

vector<GroupS> makeGroup(vector<Object> Objects)
{
	double th=400;
	vector<GroupS> result;
	vector<Object> sur;
	GroupS gs;
	//int idd=1;
	gs.setSP(Objects[0].X1(), Objects[0].Y1());
	gs.setGID(Objects[0].getGID());
	
	//gs.setP1OS(Objects[0].getP1OS());
	//int noss=0;
	sur.push_back(Objects[0]);
	for(int i=0;i<int(Objects.size()-1);i++)
	{	//cout<<"d "<<Objects[i].distP2ToP1(Objects[i+1])<<endl;
		//noss++;
		if(Objects[i].distP2ToP1(Objects[i+1])>th)
		{
			gs.setEP(Objects[i].X2(), Objects[i].Y2());
			//gs.setNoS(noss);
			gs.setObjects(sur);
			//gs.setP2OS(Objects[i].getP2OS());
			gs.setMS(0);
			result.push_back(gs);
			gs.setSP(Objects[i+1].X1(), Objects[i+1].Y1());
			//idd++;
			gs.setGID(Objects[i+1].getGID());
			//gs.setP1OS(Objects[i+1].getP1OS());
			
			//noss=0;
			sur.clear();
			sur.push_back(Objects[i+1]);
			if(i+1==int(Objects.size()-1))
			{
				gs.setEP(Objects[i+1].X2(),Objects[i+1].Y2());
				gs.setObjects(sur);
				gs.setMS(0);
				result.push_back(gs);
			}
		}
		else
		{
			
			sur.push_back(Objects[i+1]);
			if(i+1==int(Objects.size()-1))
			{
				gs.setEP(Objects[i+1].X2(),Objects[i+1].Y2());
				gs.setObjects(sur);
				gs.setMS(0);
				result.push_back(gs);
			}
		}
	}
        
        double averagePointX,averagePointY;
        for(unsigned int i=0;i<result.size();i++) {
            double totalX=0,totalY=0;
            for(unsigned int j=0;j<result[i].getObjects().size();j++) {
                totalX = totalX+result[i].getObjects()[j].mpX();
                totalY = totalY+result[i].getObjects()[j].mpY();
            }
            averagePointX = totalX/result[i].getObjects().size();
            averagePointY = totalY/result[i].getObjects().size();
            result[i].setAveragePoint(averagePointX,averagePointY);
            cout<<"size of current group: "<<result[i].getObjects().size()<<endl;
            cout<<"x: "<<averagePointX<<" y: "<<averagePointY<<endl;
        }
	return result;
}

vector<GroupS> makeGroupIFoR(vector<Object> Objects)
{
	//double th=400;
	vector<GroupS> result;
	vector<Object> sur;
	GroupS gs;
	
	//int idd=1;
	gs.setSP(Objects[0].X1(), Objects[0].Y1());
	gs.setGID(Objects[0].getGID());
	
	//gs.setP1OS(Objects[0].getP1OS());
	//int noss=0;
	sur.push_back(Objects[0]);
	for(int i=0;i<int(Objects.size());i++)
	{	
		//noss++;
		if(Objects[i].getGID()!=Objects[i+1].getGID())
		{
			gs.setEP(Objects[i].X2(), Objects[i].Y2());
			//gs.setNoS(noss);
			gs.setObjects(sur);
			//gs.setP2OS(Objects[i].getP2OS());
			gs.setMS(0);
			result.push_back(gs);
			gs.setSP(Objects[i+1].X1(), Objects[i+1].Y1());
			//idd++;
			gs.setGID(Objects[i+1].getGID());
			//gs.setP1OS(Objects[i+1].getP1OS());
			
			//noss=0;
			sur.clear();
			sur.push_back(Objects[i+1]);
		}
		else
		{
			
			sur.push_back(Objects[i+1]);
		}
	}
	return result;
}


void displayGroups(vector<GroupS> gs)
{
	for(int i=0;i<int(gs.size());i++)
		gs[i].display();
}

GroupS getGroupS(vector<GroupS> Objects, int id)
{
	//Object result;
	for(int i=0;i<int(Objects.size());i++)
	{
		if(Objects[i].getGID()==id)
			return Objects[i];	
		
	}
	cout<<" asking Object not found"<<endl;
	GroupS s;
	return s;
}

vector<GroupS> deleteGroup(vector<GroupS> groups, int dg)
{
	for(vector<GroupS>::iterator it = groups.begin(); it != groups.end();)
    	{
        	if((*it).getGID() == dg)
       		{
            		it = groups.erase(it);
       		}
       		else
       		{
          		++it;
       		}
  	}
	return groups;
}
/*
vector<int> getNewLinesID(vector<GroupS> cpr_cv, vector<int> g_common_lines)
{
		vector<int> g_new_lines;
		for(int i=0;i<int(cpr_cv.size());i++)
		{
			if(findNewLines(g_common_lines,cpr_cv[i].getGID())!=1)
			g_new_lines.push_back(cpr_cv[i].getGID());
		}	
		vector<int> gnew;
		for(int r=0;r<int(g_new_lines.size());r++)
		{
			//cout<<"g new lines: "<<g_new_lines[r]<<endl;	
			GroupS gs=getGroupS(cpr_cv, g_new_lines[r]);
			vector<int> sids=gs.getObjectID();
			for(int l=0;l<int(sids.size());l++)
			gnew.push_back(sids[l]);
		}
	return gnew;
}*/

vector<Object> updateUsingAveragePoint(vector<Object> MFIS, vector<Object> cView) {
    vector<GroupS> pViewGroups = makeGroup(MFIS);
        vector<GroupS> cViewGroups = makeGroup(cView);
        
        MyRobot myrobot(0, 0);
    vector<Object> currentRobotPositionInMFIS = myrobot.getRobot();
    
//    for(int i=0;i<cViewGroups.size();i++) {
//        cout<<"X: "<<cViewGroups[i].getAveragePoint().X()<<" Y: "<<cViewGroups[i].getAveragePoint().Y()<<endl;
//    }
        
        vector<Point> pPoints,cPoints;
        for(int i=0;i<pViewGroups.size();i++) {
            pPoints.push_back(pViewGroups[i].getAveragePoint());
        }
        for(int i=0;i<cViewGroups.size();i++) {
            cPoints.push_back(cViewGroups[i].getAveragePoint());
        }
        
        Object refObjectMap,refObjectCV;
        
        refObjectMap.set(pPoints[1].X(),pPoints[1].Y(),pPoints[2].X(),pPoints[2].Y(),1);//points 2 and 3
        refObjectCV.set(cPoints[1].X(),cPoints[1].Y(),cPoints[2].X(),cPoints[2].Y(),2);//points 2 and 3
        
        vector<Object> cRobotPosition = myrobot.inMFIS(refObjectMap,refObjectCV,1);
        
        plotObjectsAndPoints("Maps/pView.png",currentRobotPositionInMFIS,pPoints,MFIS);
        plotObjectsAndPoints("Maps/cView.png",currentRobotPositionInMFIS,cPoints,cView);
    cout<<"(PV)number of groups: "<<pViewGroups.size()<<endl;
    cout<<"(CV)number of groups: "<<cViewGroups.size()<<endl;
    return cRobotPosition;
}