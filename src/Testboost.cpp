#include <iostream>
#include <cmath>
#include <deque>
#include <cstdlib>
#include <fstream>
#include <math.h>
#include "readAndwriteASCII.H"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>

#include "Laser2Surface.H"
#include "GeometricOp.H"

#include "Object.H"
#include "PointAndSurface.H"
//#include "clipper.hpp"
#include "Point.H"
#include "PathPlanning.H"
#include "Plotting.H"


using namespace std;
namespace bg = boost::geometry;


bool polygonGen(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB); //
void drawTwoOverlappingPics(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB);//
bool between(double a, double X0, double X1);
bool detectIntersect(Point p1, Point p2, Point p3, Point p4);  

void ConvertLogFile() ;

vector<Object> surface1;
vector<Object> surface2;


const char* levelName = "inputData/level";
const char* surfaceName = "/surfaces-";
char viewFileName[80];

char ImgFileName1[80];
char ImgFileName2[80];
char ImgFileName3[80];

Point Origin;
Point A,B,C,D;

vector<double> I;

char rebuildFileNmae[80];
char exitsFileName[80];

vector<Object> currentView;
vector<Object> rebuildView;
vector<Exit> exitView;

MyRobot myrobot(0,0);


int main() 
{
         Origin.set(0,0);
         A.set(10,20);
         B.set(50,100);
         C.set(-5,20);
         D.set(-20,6);

        int v, w, r,level, set;

         cout << "Which level?? ";
         cin >> level;
         cout << "Which dataset?? ";
         cin >> set;
         cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
         cin >> v;
         cin >> w;

         //take surface informaiton as inputs
         //sprintf(viewFileName, "%s%d%s%d%s%d", levelName, 1, "set", 100, surfaceName, 1);
         //surface1 = readASCII(viewFileName);
         //sprintf(viewFileName, "%s%d%s%d%s%d", levelName, 1, "set", 100, surfaceName, 2);
         //surface2 = readASCII(viewFileName);  
         //detectIntersect(A,B,C,D);
         //cout << detectIntersect(A,B,C,D) << endl;
         
         do 
         {
                sprintf(viewFileName, "%s%d%s%d%s%d", levelName, level, "set", set, surfaceName, v);
                currentView = readASCII(viewFileName);
                
                sprintf(viewFileName, "%s%d%s", "Maps/Offline/CurrentView-", v, ".png");
                plotObjects(viewFileName, myrobot.getRobot(), currentView);    
                
                
                 exitView = findGapasExits(currentView);
                 
                sprintf(exitsFileName, "%s%d%s", "Maps/Offline/ExitView-", v, ".png");
                plotObjectsOf3KindswithExits(exitsFileName, myrobot.getRobot(), currentView, exitView);  
                
                //rebuildView = BoundaryByExits(currentView);
                
                //sprintf(rebuildFileNmae, "%s%d%s", "Maps/Offline/RebuildView-", v, ".png");
               // plotObjects(rebuildFileNmae, myrobot.getRobot(), rebuildView);    
                v++;
           }while(v < w);
         

         return 0;
}



bool polygonGen(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB)
{
         typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;

         char coordinates_1[10000];
         char coordinates_2[10000];

         char para_1[10000];
         char para_2[10000];

         double Varea;
         double Iarea;

         polygon poly1,poly2;
         std::deque<polygon> output;
    
         sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotA.X(), " ", robotA.Y(), ",");   
         //std::cout << coordinates_1 << std::endl;
        //read the coordinates of each points, 
        for ( int i = 0; i < ViewA.size(); i++) 
        {
                  sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");   
                  sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
                  if (i != ViewA.size() - 1) 
                 {
                      sprintf(coordinates_1, "%s%s", coordinates_1, ",");
                  }
                  else
                  {
                        //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
                        sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotA.X(), " ", robotA.Y());
                  }
                  //cout << coordinates_1 << endl;

         }
    
     
         sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotB.X(), " ", robotB.Y(), ",");
         for ( int i = 0; i < ViewB.size(); i++) 
         {
                  sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
                  sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());

                  if (i != ViewB.size() - 1) 
                  {
                        sprintf(coordinates_2, "%s%s", coordinates_2, ",");
                  }
                  else
                 {
                        //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
                        sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotB.X(), " ", robotB.Y());
                  }

         }
    
    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");
    
    // Calculate the area of a cartesian polygon
    
    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    

    
    bg::intersection(poly1, poly2, output);
    
    BOOST_FOREACH(polygon const& p, output)
    {
        Iarea = bg::area(p) / 100000;
        std::cout <<"Intersection area:" << Iarea << std::endl;
    }
    
    Varea = bg::area(poly1) / 100000;
    std::cout << "View 'A' area:" << Varea <<std::endl;


    double percent = (Iarea / Varea) * 100;
    
    if(percent >= 50)
        return true;
    else
        return false;
}

void drawTwoOverlappingPics(vector<Object> ViewA, Point robotA, vector<Object> ViewB, Point robotB) 
{
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    char coordinates_1[10000] = {};
    char coordinates_2[10000] = {};

    char para_1[10000];
    char para_2[10000];

    polygon poly1, poly2, p;
    std::deque<polygon> output;

    sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotA.X(), " ", robotA.Y(), ",");

    //read the coordinates of each points, 
    for (int i = 0; i < ViewA.size(); i++) {
        sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");
        sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
        if (i != ViewA.size() - 1) {
            sprintf(coordinates_1, "%s%s", coordinates_1, ",");
        } else {
            //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
            sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotA.X(), " ", robotA.Y());
        }

    }
    cout << "coordinates_1:" << coordinates_1 << endl;

    sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotB.X(), " ", robotB.Y(), ",");
    for (int i = 0; i < ViewB.size(); i++) {
        sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
        sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());

        if (i != ViewB.size() - 1) {
            sprintf(coordinates_2, "%s%s", coordinates_2, ",");
        } else {
            //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
            sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotB.X(), " ", robotB.Y());
        }

    }

    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");

    // Calculate the area of a cartesian polygon

    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    
    sprintf(ImgFileName1, "%s", "intersectionmap1.svg");
    sprintf(ImgFileName2, "%s", "intersectionmap2.svg");
    sprintf(ImgFileName3, "%s", "intersectionmap3.svg");

    std::ofstream svg1(ImgFileName1);
    std::ofstream svg2(ImgFileName2);
    std::ofstream svg3(ImgFileName3);

    boost::geometry::svg_mapper<point_type> mapper1(svg1, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper2(svg2, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper3(svg3, 2000, 2000);
    // Add geometries such that all these geometries fit on the map



    bg::intersection(poly1, poly2, output);

    BOOST_FOREACH(p, output) {

    }

    mapper1.add(poly1);

    mapper2.add(poly1);
    mapper2.add(poly2);

    mapper3.add(poly1);
    mapper3.add(poly2);
    mapper3.add(p);


    mapper1.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(p, "fill-opacity:0.8;fill:rgb(0,255,0);stroke:rgb(0,0,153);stroke-width:2", 3);

}
//////**************************************************************************/////

bool between(double a, double X0, double X1)  
{  
    double temp1= a-X0;  
    double temp2= a-X1;  
    if ( ( temp1<1e-8 && temp2>-1e-8 ) || ( temp2<1e-6 && temp1>-1e-8 ) )  
    {  
        return true;  
    }  
    else  
    {  
        return false;  
    }  
}  
  
  
// 判断两条直线段是否有交点，有则计算交点的坐标  
// p1,p2是直线一的端点坐标  
// p3,p4是直线二的端点坐标  
bool detectIntersect(Point p1, Point p2, Point p3, Point p4)  
{  
    double line_x,line_y; //交点  
    if ( (fabs(p1.X()-p2.X())<1e-6) && (fabs(p3.X()-p4.X())<1e-6) )  
    {  
        return false;  
    }  
    else if ( (fabs(p1.X()-p2.X())<1e-6) ) //如果直线段p1p2垂直与y轴  
    {  
        if (between(p1.X(),p3.X(),p4.X()))  
        {  
            double k = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
            line_x = p1.X();  
            line_y = k*(line_x-p3.X())+p3.Y();  
  
            if (between(line_y,p1.Y(),p2.Y()))  
            {  
                return true;  
            }  
            else  
            {  
                return false;  
            }  
        }  
        else   
        {  
            return false;  
        }  
    }  
    else if ( (fabs(p3.X()-p4.X())<1e-6) ) //如果直线段p3p4垂直与y轴  
    {  
        if (between(p3.X(),p1.X(),p2.X()))  
        {  
            double k = (p2.Y()-p1.Y())/(p2.X()-p1.X());  
            line_x = p3.X();  
            line_y = k*(line_x-p2.X())+p2.Y();  
  
            if (between(line_y,p3.Y(),p4.Y()))  
            {  
                return true;  
            }  
            else  
            {  
                return false;  
            }  
        }  
        else   
        {  
            return false;  
        }  
    }  
    else  
    {  
        double k1 = (p2.Y()-p1.Y())/(p2.X()-p1.X());   
        double k2 = (p4.Y()-p3.Y())/(p4.X()-p3.X());  
  
        if (fabs(k1-k2)<1e-6)  
        {  
            return false;  
        }  
        else   
        {  
            line_x = ((p3.Y() - p1.Y()) - (k2*p3.X() - k1*p1.X())) / (k1-k2);  
            line_y = k1*(line_x-p1.X())+p1.Y();  
        }  
  
        if (between(line_x,p1.X(),p2.X())&&between(line_x,p3.X(),p4.X()))  
        {  
            return true;  
        }  
        else   
        {  
            return false;  
        }  
    }  
}


void ConvertLogFile() 
{
            Point singleRobotPosition;
            vector< Point > allRobotPosition;

            Point singleLaserPoint;
            vector<Point> aScan;
            vector< vector<Point> > allScans;


            string dName = "Maps/";
            string addForLogFile = dName + "input.log";
        //            addForLogFile += 

            string addForAlbot1 = dName;


            double time;
            vector<double> times;
            ifstream inputFile(addForLogFile.c_str(), ios::in);
            if (inputFile.is_open()) {
                cout << "Reading logfile...." << endl;
                double x, y, theta;
                string data;



                while (!inputFile.eof())
                {

                    inputFile >> data;
                    if (data.compare("robotGlobal:") == 0) 
                    {
                        //reading odometry information
                        inputFile >> x;
                        inputFile >> y;
                        inputFile >> theta;

                        //cout<<"x: "<<x<<" y: "<<y<<" th: "<<theta<<endl;
                        singleRobotPosition.set(x, y);
                        singleRobotPosition.setOAngle(theta);

                        allRobotPosition.push_back(singleRobotPosition);
                        cout << "x: " << singleRobotPosition.X() << " y: " << singleRobotPosition.Y() << " th: " << theta + 90 << endl;
                        //waitHere();
                    }
                    inputFile >> data;
                    if (data.compare("scan1:") == 0) {

                        for (int i = 0; i < 181; i++) {//181 for 1degree resolution 360 for .5 degree resolution
                            inputFile >> x;
                            inputFile >> y;
                            singleLaserPoint.set(x, y);
                            aScan.push_back(singleLaserPoint); //storing all laser readings for this scan
                            cout << "x " << x << " y " << y;
                        }
                        allScans.push_back(aScan); //saving all scans
                        aScan.clear();
                        //waitHere();
                    }
                }


            } else
                cout << "Error opening " << addForLogFile << " .." << endl;


            inputFile.close();


            double thisLaserRange;
            vector<double> distang;
            vector<PointXY> laserPoints;
            vector<Surface> surfaces;
            char fileName[1000];
            int skipCount = 10;
            int stepCount = 1;
            double lastRX = 0.0, lastRY = 0.0, lastRA = 0.0;
            double travelDist = 0.0, turnAngle = 0.0;


            double clusterThreshold = 600; // ct; //atof(argv[1]);
            int surfaceSize = 200; //ss;// atoi(argv[2]);
            int errorThreshold = 150; //et;//atoi(argv[3]);
            //open a file to write for DPSLAM
            cout << "No of scan " << allRobotPosition.size() << endl;
            cout << "No of time " << times.size() << endl;

            for (unsigned int i = 0; i < allRobotPosition.size(); i++) {
                travelDist = getDistBtw2Points(lastRX, lastRY, allRobotPosition[i].X(), allRobotPosition[i].Y());
                turnAngle = allRobotPosition[i].getOAngle() - lastRA;
                if (travelDist > 1000.0 or abs(turnAngle) > 10.0 or i == 0) {

                    //writing laserScan for albot1
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "laser-", stepCount);
//                    writeLaserScan(fileName, allScans[i]);


                    //writing for albot1
                    if (i == 0) {
                        distang.push_back(0.0);
                        distang.push_back(0.0);
                    } 
                    else 
                    {
                        //distang.push_back(getDistBtw2Points(allRobotPosition[i-1].X(),allRobotPosition[i-1].Y(),allRobotPosition[i].X(),allRobotPosition[i].Y()));
                        //distang.push_back(allRobotPosition[i].getOAngle()-allRobotPosition[i-1].getOAngle());
                        distang.push_back(getDistBtw2Points(lastRX, lastRY, allRobotPosition[i].X(), allRobotPosition[i].Y()));
                        distang.push_back(allRobotPosition[i].getOAngle() - lastRA);
                    }
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "coordTrans-", stepCount);
                    writeASCII(distang, 2, fileName);
                    distang.clear();
                    for (unsigned int j = allScans[i].size(); j-- > 0;) {
                        //for(unsigned int j=0;j<allScans[i].size();j++) {
                        thisLaserRange = sqrt(allScans[i][j].X() * allScans[i][j].X() + allScans[i][j].Y() * allScans[i][j].Y());
                        if (thisLaserRange < 30000.0)
                            laserPoints.push_back(PointXY(-allScans[i][j].Y(), allScans[i][j].X()));
                    }
                    surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
                    sprintf(fileName, "%s%s%d", addForAlbot1.c_str(), "surfaces-", stepCount);
                    writeASCII(surfaces, fileName);

                    lastRX = allRobotPosition[i].X();
                    lastRY = allRobotPosition[i].Y();
                    lastRA = allRobotPosition[i].getOAngle();

                    laserPoints.clear();
                    skipCount = 0;
                    stepCount++;
                    cout << "Step " << stepCount << " completed" << endl;

                }

                skipCount++;

            }
}



