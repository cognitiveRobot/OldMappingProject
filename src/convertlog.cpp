/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   convertlog.cpp
 * Author: arthur
 *
 * Created on December 3, 2015, 9:57 PM
 */

#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <string>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "GeometricOp.H"
#include "Laser2Surface.H"
#include "Point.H"
#include "Object.H"
#include "asr.H"
#include "mfisOp.H"
#include "asrOp.H"
#include "PathPlanning.H"
#include "Transporter.H"

#include "Minfo.H"
#include "Map.H"
#include "CompareASR.H"
#include "Mapping.H"

#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "PolygonChoice.h"
#include "PerceptualMapping.H"
#include "Comparison.h"

#include "thesis.H"

#include <cstdlib>
#include <ctime>
void ConvertLogFile();
using namespace std;

/*
 * 
 */
int main(int argc, char** argv) 
{
        ConvertLogFile();
        return 0;
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
        cout << "Before Reading logfile...." << endl;
        if (inputFile.is_open()) {
            cout << "Reading logfile...." << endl;
            double x, y, theta;
            string data;



            while (!inputFile.eof()) {

                inputFile >> data;
                if (data.compare("robotGlobal:") == 0) {
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


        } else{
            cout << "Error opening " << addForLogFile << " .." << endl;
        }


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
               // writeLaserScan(fileName, allScans[i]);


                //writing for albot1
                if (i == 0) {
                    distang.push_back(0.0);
                    distang.push_back(0.0);
                } else {
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

