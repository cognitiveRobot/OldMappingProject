#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <dirent.h>
#include "Object.H"
#include "readAndwriteASCII.H"
#include "PointAndSurface.H"
#include "GeometricOp.H"
#include "Plotting.H"

#include "Laser2Surface.H"


using namespace std;

#define PI 3.14159265

//convert dp-slam input data to albot1 input data

void readInput(char *filename) {


    Point singleRobotPosition;
    vector< Point > allRobotPosition;
    vector<double> dists;
    vector< vector<double> > alldists;
    ifstream inputFile(filename, ios::in);
    if (!inputFile) {
        cout << "Error opening " << filename << " .." << endl;
        //return Objects;
    } else {
        double dist, x, y, theta, lNum;
        string odo, laser;
        int scanNumer = 0;

        inputFile >> odo;
        inputFile >> x;
        inputFile >> y;
        inputFile >> theta;
        inputFile >> laser;
        inputFile >> lNum;

        //odometry data
        singleRobotPosition.set(x * 1000, y * 1000);
        singleRobotPosition.setOAngle(theta);

        //laser data
        scanNumer++;
        for (int i = 0; i < 181; i++) {
            inputFile >> dist;
            dists.push_back(dist);
        }

        while (!inputFile.eof()) {
            allRobotPosition.push_back(singleRobotPosition); //odometry data
            //singleRobotPosition.clear();

            alldists.push_back(dists); //laser data
            dists.clear();

            //printing on the screen
            cout << endl << scanNumer << endl;
            cout << odo << " " << x << "  " << y << "  " << endl;
            for (int i = 0; i < alldists.back().size(); i++)
                cout << alldists.back()[i] << " ";

            inputFile >> odo;
            inputFile >> x;
            inputFile >> y;
            inputFile >> theta;
            inputFile >> laser;
            inputFile >> lNum;

            //odometry data
            singleRobotPosition.set(x * 1000, y * 1000);
            singleRobotPosition.setOAngle(theta);
            //singleRobotPosition.push_back(x*1000);
            //singleRobotPosition.push_back(y*1000);
            //singleRobotPosition.push_back(theta);

            //laser data
            scanNumer++;
            for (int i = 0; i < 181; i++) {
                inputFile >> dist;
                dists.push_back(dist);
            }
        }
    }
    inputFile.close();

    double x, y;
    double angle;
    double a;
    MyRobot myrobot(0, 0);
    vector<PointXY> laserPoints;
    vector<Surface> surfaces;
    vector <Object> view;
    char sname[50];
    int viewNumber = 0;


    double clusterThreshold = 600; // ct; //atof(argv[1]);
    int surfaceSize = 200; //ss;// atoi(argv[2]);
    int errorThreshold = 150; //et;//atoi(argv[3]);

    //converting all laser reading. each scan will be a view where robot is at 0,0. all laserpoints co-ordinates
    //are with respect to current robot position
    cout << "Converting all laser reading" << endl << endl;
    for (unsigned int i = 0; i < alldists.size(); i++) {
        cout << endl << i << endl;
        viewNumber++;
        //converting laser reading from Polar to Cartesian for the current scan
        for (unsigned int j = 0; j < 181; j++) {
            angle = 180 - j; //angle with respect to (+)x-axis, robot is facing toward (+)y-axis.
            angle = (angle / 180) * PI; //angle in radian
            a = alldists[i][j]*1000; //converted to mm
            x = a * cos(angle);
            y = a * sin(angle);
            laserPoints.push_back(PointXY(x, y));
            cout << j << " " << alldists[i][j] << " " << x << " " << y << endl;
        }
        surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
        //writing surface file for the current view
        sprintf(sname, "%s%d", "bin/level1set100/surfaces-", viewNumber);
        writeASCII(surfaces, sname);
        laserPoints.clear();

        //print current view
        //        if (i < 25) {
        //            view = readASCII(sname);
        //            sprintf(sname, "%s%d", "Maps/view-", viewNumber);
        //            plotObjects(sname, view, myrobot.getRobot());
        //        }
    }
    Object aRobot;
    vector<Object> allARobots;
    Object arbitraryAxis(0, 0, 0, 500);
    char coordTransFileName[100];
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);
    sprintf(coordTransFileName, "%s%d", "bin/level1set100/coordTrans-", 1);
    writeASCII(distang, 2, coordTransFileName);
    for (unsigned int i = 0; i < allRobotPosition.size() - 1; i++) {
        //if(abs(allRobotPosition[i+1].getOAngle()*(180/PI) - angle) > 5)
        //cout<<allRobotPosition[i+1].getOAngle()*(180/PI) - angle <<" "<<endl;
        //angle = ((allRobotPosition[i+1].getOAngle()/PI)*180) - ((allRobotPosition[i].getOAngle()/PI)*180);
        //angle = (allRobotPosition[i+1].getOAngle()-allRobotPosition[i].getOAngle())*180;
        angle = (allRobotPosition[i].getOAngle() - allRobotPosition[i + 1].getOAngle())*180;
        angle = angle / PI;
        a = getDistBtw2Points(allRobotPosition[i].X(), allRobotPosition[i].Y(), allRobotPosition[i + 1].X(), allRobotPosition[i + 1].Y());
        //        if(angle > 90) 
        //            angle = angle-180;
        //        cout<<"Angle: "<<allRobotPosition[i].getOAngle()<<" = "<<allRobotPosition[i].getOAngle()*(180/PI)<<endl;
        //        aRobot = makeLineAtPointWithObject(angle,arbitraryAxis.distP1ToPoint(allRobotPosition[i].X(),allRobotPosition[i].Y()),500,arbitraryAxis);
        //        allARobots.push_back(aRobot);
        distang.clear();
        distang.push_back(a);
        distang.push_back(angle);
        sprintf(coordTransFileName, "%s%d", "bin/level1set100/coordTrans-", i + 2);
        writeASCII(distang, 2, coordTransFileName);

        if (abs(angle) > 200)
            cout << endl << i << " " << angle << endl;
        else
            cout << angle << " ";

    }
    vector<Object> dummy;
    //plotObjectsAndPoints("Maps/allRobotPositions.png", dummy, allARobots, allRobotPosition);

}

//convert DP-SLAM input data to albot1 input data

void readDPSLAMInput(char *filename) {


    Point singleRobotPosition;
    vector< Point > allRobotPosition;
    vector<double> dists;
    vector< vector<double> > alldists;
    ifstream inputFile(filename, ios::in);
    if (!inputFile) {
        cout << "Error opening " << filename << " .." << endl;
        //return Objects;
    } else {
        cout << "Reading datafile...." << endl;
        double dist, x, y, theta, lNum, laser;
        string data;


        while (!inputFile.eof()) {
            inputFile >> data;
            if (data.compare("Odometry") == 0) {
                //reading odometry information
                inputFile >> x;
                inputFile >> y;
                inputFile >> theta;

                cout << x << " " << y << " " << theta * (180.0 / PI) << endl;

                singleRobotPosition.set(x * 1000, y * 1000);
                singleRobotPosition.setOAngle(theta);

                allRobotPosition.push_back(singleRobotPosition);
                //waitHere();

            } else if (data.compare("Laser") == 0) {
                inputFile >> data; // no. of laser readings                
                for (int i = 0; i < 181; i++) {
                    inputFile >> dist;
                    dists.push_back(dist); //all laser readings for this scan
                }
                alldists.push_back(dists); //saved this scan
                dists.clear();
            }
        }
        //waitHere();
        cout << endl << "no of robot positions: " << allRobotPosition.size() << endl;
        cout << "no of scan: " << alldists.size() << endl;
    }
    inputFile.close();

    //    for(unsigned int i=0;i<alldists[8].size();i++) {
    //        cout<<alldists[8][i]<<" ";
    //        
    //    }
    //    waitHere();
    double x, y;
    double angle;
    double a;
    MyRobot myrobot(0, 0);
    vector<PointXY> laserPoints;
    vector<PointXY> tempLPoints;
    vector<Surface> surfaces;
    vector <Object> view;
    char sname[50];
    int viewNumber = 0;


    double clusterThreshold = 600; // ct; //atof(argv[1]);
    int surfaceSize = 200; //ss;// atoi(argv[2]);
    int errorThreshold = 150; //et;//atoi(argv[3]);

    //converting all laser reading. each scan will be a view where robot is at 0,0. all laserpoints co-ordinates
    //are with respect to current robot position
    cout << "Converting all laser reading" << endl << endl;
    for (unsigned int i = 0; i < alldists.size(); i++) {
        cout << endl << i << endl;
        viewNumber++;
        //converting laser reading from Polar to Cartesian for the current scan
        for (unsigned int j = 0; j < 181; j++) {
            angle = 180 - j; //angle with respect to (+)x-axis, robot is facing toward (+)y-axis.
            angle = (angle / 180) * PI; //angle in radian
            a = alldists[i][j]*1000; //converted to mm
            x = -a * cos(angle);
            y = a * sin(angle);
            laserPoints.push_back(PointXY(x, y));
            cout << j << " " << (-90 + j) << " x: " << a * cos(((-90.0 + j) * PI) / 180.0) << " y: " << a * sin(((-90.0 + j) * PI) / 180.0) << endl;
        }
        //waitHere();
        tempLPoints = laserPoints;
        laserPoints.clear();
        for (unsigned int k = tempLPoints.size() - 1; k > 0; k--) {
            laserPoints.push_back(tempLPoints[k]);
        }
        surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
        //writing surface file for the current view
        sprintf(sname, "%s%d", "bin/level1set104/surfaces-", viewNumber);
        writeASCII(surfaces, sname);
        laserPoints.clear();
    }
    Object aRobot;
    vector<Object> allARobots;
    Object arbitraryAxis(0, 0, 0, 500);
    char coordTransFileName[100];
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);
    sprintf(coordTransFileName, "%s%d", "bin/level1set104/coordTrans-", 1);
    writeASCII(distang, 2, coordTransFileName);
    for (unsigned int i = 0; i < allRobotPosition.size() - 1; i++) {
        //if(abs(allRobotPosition[i+1].getOAngle()*(180/PI) - angle) > 5)
        //cout<<allRobotPosition[i+1].getOAngle()*(180/PI) - angle <<" "<<endl;
        //angle = ((allRobotPosition[i+1].getOAngle()/PI)*180) - ((allRobotPosition[i].getOAngle()/PI)*180);
        //angle = (allRobotPosition[i+1].getOAngle()-allRobotPosition[i].getOAngle())*180;
        angle = (allRobotPosition[i].getOAngle() - allRobotPosition[i + 1].getOAngle())*180;
        angle = angle / PI;
        a = getDistBtw2Points(allRobotPosition[i].X(), allRobotPosition[i].Y(), allRobotPosition[i + 1].X(), allRobotPosition[i + 1].Y());
        //        if(angle > 90) 
        //            angle = angle-180;
        //        cout<<"Angle: "<<allRobotPosition[i].getOAngle()<<" = "<<allRobotPosition[i].getOAngle()*(180/PI)<<endl;
        //        aRobot = makeLineAtPointWithObject(angle,arbitraryAxis.distP1ToPoint(allRobotPosition[i].X(),allRobotPosition[i].Y()),500,arbitraryAxis);
        //        allARobots.push_back(aRobot);
        distang.clear();
        distang.push_back(a);
        distang.push_back(angle);
        sprintf(coordTransFileName, "%s%d", "bin/level1set104/coordTrans-", i + 2);
        writeASCII(distang, 2, coordTransFileName);
    }
    vector<Object> dummy;
    //plotObjectsAndPoints("Maps/allRobotPositions.png", dummy, allARobots, allRobotPosition);

}



//convert carmen input data to albot1 input data

void readCarmenInput(char *filename) {


    Point singleRobotPosition;
    vector< Point > allRobotPosition;
    vector<double> dists;
    vector< vector<double> > alldists;

    const char* addForAlbot1 = "bin/level1set64/";

    ifstream inputFile(filename, ios::in);
    if (!inputFile) {
        cout << "Error opening " << filename << " .." << endl;
        //return Objects;
    } else {
        cout << "Reading datafile...." << endl;
        double dist, x, y, theta, lNum, laser;
        string data;



        while (!inputFile.eof()) {
            inputFile >> data;
            if (data.compare("FLASER") == 0) {
                inputFile >> data; // no. of laser readings


                for (int i = 0; i < 180; i++) {
                    inputFile >> dist;
                    dists.push_back(dist); //all laser readings for this scan
                }
                alldists.push_back(dists); //saved this scan
                dists.clear();

                //reading odometry information
                inputFile >> x;
                inputFile >> y;
                inputFile >> theta;

                singleRobotPosition.set(x * 1000, y * 1000);
                singleRobotPosition.setOAngle(theta);

                allRobotPosition.push_back(singleRobotPosition);

            }
        }
        //        for(unsigned int i=0;i<alldists.size();i++) {
        //            cout<<endl<<alldists[i].size()<<" ";
        //            for(unsigned int j=0;j<alldists[i].size();j++)
        //                cout<<alldists[i][j]<<" ";
        //        }
        //        for(unsigned int i=0;i<allRobotPosition.size();i++) {
        //            cout<<endl<<allRobotPosition[i].X()<<" "<<allRobotPosition[i].Y()<<" ";
        //            
        //        }

        cout << endl << "no of robot positions: " << allRobotPosition.size() << endl;
        cout << "no of scan: " << alldists.size() << endl;


    }
    inputFile.close();

    double x, y;
    double angle;
    double a;
    MyRobot myrobot(0, 0);
    vector<PointXY> laserPoints;
    vector<Surface> surfaces;
    vector <Object> view;
    char sname[50];
    int viewNumber = 0;


    double clusterThreshold = 600; // ct; //atof(argv[1]);
    int surfaceSize = 200; //ss;// atoi(argv[2]);
    int errorThreshold = 150; //et;//atoi(argv[3]);

    //converting all laser reading. each scan will be a view where robot is at 0,0. all laserpoints co-ordinates
    //are with respect to current robot position
    cout << "Converting all laser reading" << endl << endl;
    for (unsigned int i = 0; i < alldists.size(); i++) {
        cout << endl << i << endl;
        viewNumber++;
        //converting laser reading from Polar to Cartesian for the current scan
        for (unsigned int j = 0; j < 180; j++) {
            angle = 180 - j; //angle with respect to (+)x-axis, robot is facing toward (+)y-axis.
            angle = (angle / 180) * PI; //angle in radian
            a = alldists[i][j]*1000; //converted to mm
            x = a * cos(angle);
            y = a * sin(angle);
            laserPoints.push_back(PointXY(x, y));
            cout << j << " " << alldists[i][j] << " " << x << " " << y << endl;
        }
        surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
        //writing surface file for the current view
        sprintf(sname, "%s%s%d", addForAlbot1, "surfaces-", viewNumber);
        writeASCII(surfaces, sname);
        laserPoints.clear();

        //print current view
        //        if (i < 25) {
        //            view = readASCII(sname);
        //            sprintf(sname, "%s%d", "Maps/view-", viewNumber);
        //            plotObjects(sname, view, myrobot.getRobot());
        //        }
    }
    Object aRobot;
    vector<Object> allARobots;
    Object arbitraryAxis(0, 0, 0, 500);
    char coordTransFileName[100];
    vector<double> distang;
    distang.push_back(0); //rdist);
    distang.push_back(0); //rangle);
    sprintf(coordTransFileName, "%s%s%d", addForAlbot1, "coordTrans-", 1);
    writeASCII(distang, 2, coordTransFileName);
    for (unsigned int i = 0; i < allRobotPosition.size() - 1; i++) {
        //if(abs(allRobotPosition[i+1].getOAngle()*(180/PI) - angle) > 5)
        //cout<<allRobotPosition[i+1].getOAngle()*(180/PI) - angle <<" "<<endl;
        //angle = ((allRobotPosition[i+1].getOAngle()/PI)*180) - ((allRobotPosition[i].getOAngle()/PI)*180);
        //angle = (allRobotPosition[i+1].getOAngle()-allRobotPosition[i].getOAngle())*180;
        angle = (allRobotPosition[i].getOAngle() - allRobotPosition[i + 1].getOAngle())*180;
        angle = angle / PI;
        a = getDistBtw2Points(allRobotPosition[i].X(), allRobotPosition[i].Y(), allRobotPosition[i + 1].X(), allRobotPosition[i + 1].Y());
        //        if(angle > 90) 
        //            angle = angle-180;
        //        cout<<"Angle: "<<allRobotPosition[i].getOAngle()<<" = "<<allRobotPosition[i].getOAngle()*(180/PI)<<endl;
        //        aRobot = makeLineAtPointWithObject(angle,arbitraryAxis.distP1ToPoint(allRobotPosition[i].X(),allRobotPosition[i].Y()),500,arbitraryAxis);
        //        allARobots.push_back(aRobot);
        distang.clear();
        distang.push_back(a);
        distang.push_back(angle);
        sprintf(coordTransFileName, "%s%s%d", addForAlbot1, "coordTrans-", i + 2);
        writeASCII(distang, 2, coordTransFileName);
        //
        //        if (abs(angle) > 200)
        //            cout << endl << i << " " << angle << endl;
        //        else
        //            cout << angle << " ";

    }
    vector<Object> dummy;
    //plotObjectsAndPoints("Maps/allRobotPositions.png", dummy, allARobots, allRobotPosition);

}

//offline version 1

void readAlbotInput4DPSLAM(int steps) {
    vector<double> laserReading;
    vector <double> coordTransInfo;

    //open a file to write
    ofstream outFile("bin/level1set50/fromAlbotInput.log", ios::out);

    //making file name to read
    const char* folderName = "bin/level1set50";
    char fileName[100];

    outFile << fixed;
    outFile.precision(6);

    double robotX = 0, robotY = 0, robotTH = 0;
    for (unsigned int i = 0; i < steps; i++) {
        //reading odometry(Albot) info
        sprintf(fileName, "%s%s%d", folderName, "/odometry-", i + 1);
        coordTransInfo = readCoordTrans(fileName);

        robotX = -coordTransInfo[1] / 1000.0;
        robotY = coordTransInfo[0] / 1000.0;
        robotTH = (coordTransInfo[2]) * (PI / 180.0);

        cout << "x " << robotX << " y " << robotY << " th " << coordTransInfo[2] << endl;

        //reading laserReading(Albot) info
        sprintf(fileName, "%s%s%d", folderName, "/laserRange-", i + 1);
        laserReading = readCoordTrans(fileName);

        //cout<<laserReading.size()<<endl;


        //writing odometry info of last step
        outFile << "Odometry" << " ";
        outFile << robotX << " ";
        outFile << robotY << " ";
        outFile << robotTH << endl;

        //writing laserReading of last step
        outFile << "Laser" << " ";
        outFile << laserReading.size() << " ";
        //for(unsigned int j=laserReading.size();j-->0;) {
        for (unsigned int j = 0; j < laserReading.size(); j++) {
            outFile << laserReading[j] / 1000.0 << " ";
            //cout<< j <<" ";
        }
        outFile << endl;

    }
    outFile.close();
}


//read and save at every five or so step from DP-SLAM input data 

void readDPSLAMInput4DPSLAM(char *filename) {



    vector<double> laserReading;

    int stepInterval = 0;
    ifstream inputFile(filename, ios::in);
    if (!inputFile) {
        cout << "Error opening " << filename << " .." << endl;
        //return Objects;
    } else {
        cout << "Reading datafile...." << endl;
        double dist, x, y, theta;
        string data;

        //open a file to write
        ofstream outFile("bin/loop5-new.log", ios::out);
        outFile << fixed;
        outFile.precision(6);

        while (!inputFile.eof()) {
            //reading
            inputFile >> data; //odometry string

            //reading odometry information
            inputFile >> x;
            inputFile >> y;
            inputFile >> theta;


            inputFile >> data; // laser string
            inputFile >> data; // no. of laser readings   
            laserReading.clear();
            for (int i = 0; i < 181; i++) {
                inputFile >> dist;
                laserReading.push_back(dist); //all laser readings for this scan
            }


            //writing
            if (stepInterval == 0 or stepInterval == 10) {
                //writing odometry info of last step
                outFile << "Odometry" << " ";
                outFile << x << " ";
                outFile << y << " ";
                outFile << theta << endl;

                //writing laserReading of last step
                outFile << "Laser" << " ";
                outFile << laserReading.size() << " ";

                for (unsigned int j = 0; j < laserReading.size(); j++) {
                    outFile << laserReading[j] << " ";
                }
                outFile << endl;
                if (stepInterval == 10)
                    stepInterval = 1;
            }

            stepInterval++;

        }
        outFile.close();
    }
    inputFile.close();

}

//read albot1's keyboard controlled input
//writes for Albot1, DP-SLAM and Carmen

void readAlbotKBCInput() {
    Point singleRobotPosition;
    vector< Point > allRobotPosition;

    Point singleLaserPoint;
    vector<Point> aScan;
    vector< vector<Point> > allScans;

    //mention how many laser reading at (int i = 0; i < 181; i++) {
    //181 for 1degree resolution 360 for .5 degree resolution
    const char* addForLogFile = "bin/level1set500/1.log";
    const char* addForDPSLAM = "bin/level1set500/wholeFloor-dpslam-100-1-CW-R2.log";
    const char* addForAlbot1 = "bin/level1set500/";
    const char* addForCarmen = "bin/level1set500/wholeFloor-carmen-100-1-CW-R2.log";

    double time;
        vector<double> times;
    ifstream inputFile(addForLogFile, ios::in);
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


    } else
        cout << "Error opening " << addForLogFile << " .." << endl;
    
    //plotPoints("bin/KeyBoardControlled/aScan.png",allScans[0]);
    vector<Object> dummy;
    //plotObjectsAndPoints("Maps/allRobotPositions.png", dummy, dummy, allRobotPosition);
    inputFile.close();

    double lastPoint, thisLaserRange;
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
    cout<<"No of time "<<times.size()<<endl;
    //waitHere();
    ofstream outFile(addForDPSLAM, ios::out);
    ofstream outCarmen(addForCarmen, ios::out);

    outFile << fixed;
    outFile.precision(6);
    for (unsigned int i = 0; i < allRobotPosition.size(); i++) {
        //for(unsigned int i=0;i<howFar;i++) {    
        travelDist = getDistBtw2Points(lastRX, lastRY, allRobotPosition[i].X(), allRobotPosition[i].Y());
        turnAngle = allRobotPosition[i].getOAngle() - lastRA;
        //if(skipCount > 5) {//0 means no skipping, 5 means skip 5 steps 
        if (travelDist > 100.0 or abs(turnAngle) > 1.0 or i == 0) {
            
            //writing odometry info for DP-SLAM
            outFile << "Odometry" << " ";
            outFile << -allRobotPosition[i].Y() / 1000.0 << " ";
            outFile << allRobotPosition[i].X() / 1000.0 << " ";
            outFile << (allRobotPosition[i].getOAngle() + 90.0) * (PI / 180.0) << endl;

            //writing laserReading    for DP-SLAM         
            outFile << "Laser" << " ";
            outFile << allScans[i].size() << " ";
            
            for (unsigned int j = 0; j < allScans[i].size(); j++) {
                //for (unsigned int j = allScans[i].size(); j-->0;) {
                thisLaserRange = sqrt(allScans[i][j].X() * allScans[i][j].X() + allScans[i][j].Y() * allScans[i][j].Y());
                thisLaserRange = thisLaserRange / 1000.0; //mm to m
                if (thisLaserRange == 0.0) {
                    //cout<<"x "<<allScans[i][j].X()<<" y "<<allScans[i][j].Y()<<endl;
                    outFile << lastPoint << " ";
                } else {
                    outFile << thisLaserRange << " ";
                    lastPoint = thisLaserRange;
                }
             }
            outFile << endl;
            
            //writing for carmen format //this initial values are extremely important to compute correct map
            outCarmen << fixed;            
            outCarmen << "ROBOTLASER1" << " ";            
            outCarmen <<0<<" ";
            outCarmen <<-1.570796<<" ";
            outCarmen <<3.141593<<" ";
            outCarmen <<0.017453<<" ";
            outCarmen <<81.900000<<" ";
            outCarmen <<0.010000<<" ";
            outCarmen <<0<<" ";
            outCarmen << allScans[i].size() << " ";
            outCarmen.precision(2);
            for (unsigned int j = 0; j < allScans[i].size(); j++) {
                thisLaserRange = sqrt(allScans[i][j].X() * allScans[i][j].X() + allScans[i][j].Y() * allScans[i][j].Y());
                thisLaserRange = thisLaserRange / 1000.0; //mm to m
                                
                outCarmen << thisLaserRange << " ";
            }
            outCarmen <<0<<" ";
            outCarmen.precision(6);
            outCarmen << -allRobotPosition[i].Y() / 1000.0 << " ";
            outCarmen << allRobotPosition[i].X() / 1000.0 << " ";
            outCarmen << (allRobotPosition[i].getOAngle() + 90.0) * (PI / 180.0) << " ";
            outCarmen <<endl;
            
            outCarmen << "ODOM" << " ";
            outCarmen << -allRobotPosition[i].Y() / 1000.0 << " ";
            outCarmen << allRobotPosition[i].X() / 1000.0 << " ";
            outCarmen << (allRobotPosition[i].getOAngle() + 90.0) * (PI / 180.0) << " ";
            outCarmen <<endl;

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
            sprintf(fileName, "%s%s%d", addForAlbot1, "coordTrans-", stepCount);
            writeASCII(distang, 2, fileName);
            distang.clear();
            for (unsigned int j = allScans[i].size(); j-- > 0;) {
                //for(unsigned int j=0;j<allScans[i].size();j++) {
                thisLaserRange = sqrt(allScans[i][j].X() * allScans[i][j].X() + allScans[i][j].Y() * allScans[i][j].Y());
                if (thisLaserRange < 30000.0)
                    laserPoints.push_back(PointXY(-allScans[i][j].Y(), allScans[i][j].X()));
            }
            surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
            sprintf(fileName, "%s%s%d", addForAlbot1, "surfaces-", stepCount);
            writeASCII(surfaces, fileName);

            lastRX = allRobotPosition[i].X();
            lastRY = allRobotPosition[i].Y();
            lastRA = allRobotPosition[i].getOAngle();

            laserPoints.clear();
            skipCount = 0;
            stepCount++;
            cout << "Step " << stepCount << " completed" << endl;
//             if(stepCount == 128) {
//                outFile.close();
//                waitHere();
//            }
        }

        skipCount++;

    }
    outFile.close();
}

//read carmen's keyboard controlled input
//save carmen format varying step distance distFromLastStep >= 0.1m 

void carmenToCarmenConverter() {
    Point singleRobotPosition;
    vector< Point > allRobotPosition;
    Point singleLaserPoint;
    vector<Point> aScan;
    vector< vector<Point> > allScans;

    const char* addForLogFile = "bin/KeyBoardControlled/log1-sloop-C-carmen-robot2.log";
    const char* addForDPSLAM = "bin/KeyBoardControlled/log1-sloop-C-carmen-robot2-new-10cm-240s.log";
    //const char* addForAlbot1 = "bin/level1set63/";
    int howFar = 500;
    ifstream inputFile(addForLogFile, ios::in); //open to read
    ofstream outFile(addForDPSLAM, ios::out); //open to write


    vector<string> odoms;
    int scanID = 1;
    double distFromLastStep = 0;

    if (inputFile.is_open()) {
        cout << "Reading logfile...." << endl;
        double lastX, lastY;
        string data;

        //while (scanID < 240) {
        while (!inputFile.eof()) {
            inputFile >> data;

            if (data.compare("ODOM") == 0) {
                odoms.clear();
                do {
                    odoms.push_back(data); //"ODOM"
                    inputFile >> data;
                } while (data.compare("CogBot") != 0);
                odoms.push_back(data); //"CogBot"
                inputFile >> data; //for last odom data
                odoms.push_back(data); //for last odom data

            }
            if (scanID > 1 && data.compare("ROBOTLASER1") == 0) {
                distFromLastStep = getDistBtw2Points(atof(odoms[1].c_str()), atof(odoms[2].c_str()), lastX, lastY);
            }
            //(0.1 means 10cm apart)
            if (data.compare("ROBOTLASER1") == 0 && (scanID == 1 || distFromLastStep >= 0.1)) {
                for (unsigned int i = 0; i < odoms.size(); i++) {
                    outFile << odoms[i] << " ";
                }
                outFile << endl;
                lastX = atof(odoms[1].c_str());
                lastY = atof(odoms[2].c_str());

                do {
                    outFile << data; //writing for new version
                    outFile << " ";
                    inputFile >> data;
                } while (data.compare("CogBot") != 0);

                outFile << data << " "; // for CogBot
                inputFile >> data; //for last odom data
                outFile << data << " " << endl; //

                scanID++;
            }
        }
    } else
        cout << "Error opening " << addForLogFile << " .." << endl;

    cout << scanID << endl;

    inputFile.close();
    outFile.close();
}

//read carmen's keyboard controlled input
//save albot1 format varying step distance distFromLastStep >= 0.1m 

void carmenToAlbot1Converter() {
    cout<<"CarmenToAlbot1Converter"<<endl;

    const char* addForLogFile = "bin/level1set70/1.log";
    //const char* addForDPSLAM = "bin/KeyBoardControlled/log1-sloop-C-carmen-robot2-new-10cm-240s.log";
    const char* addForAlbot1 = "bin/level1set70/";

    ifstream inputFile(addForLogFile, ios::in); //open to read
    //ofstream outFile(addForDPSLAM, ios::out); //open to write

    if (inputFile.is_open()) {
        cout << "Reading logfile...." << endl;
        double lastX, lastY, lastAngle, dist, x, y, theta;
        vector<double> dists, distang;
        vector< vector<double> > alldists;
        string data;

        Point singleRobotPosition;
        vector< Point > allRobotPosition;



        int scanID = 1;
        double distFromLastStep = 0.0;
        double rotation = 0.0;

        char coordTransFileName[100], surfaceFileName[100];
        vector<PointXY> laserPoints;
        vector<Surface> surfaces;
        double clusterThreshold = 600; // ct; //atof(argv[1]);
        int surfaceSize = 200; //ss;// atoi(argv[2]);
        int errorThreshold = 150; //et;//atoi(argv[3]);

        //while (scanID < 240) {
        while (!inputFile.eof()) {
            inputFile >> data;

            //if (data.compare("ODOM ") == 0) {
            if (data.compare("ROBOTLASER1") == 0) {
                cout<<data<<endl;
                //odoms.clear();
                do {
                    //odoms.push_back(data);//"ODOM"
                    inputFile >> data;
                } while (data.compare("181") != 0);


                //reading laser readings
                dists.clear();
                for (unsigned int i = 0; i < 181; i++) {
                    inputFile >> dist;
                    dists.push_back(dist); //all laser readings for this scan
                }
                alldists.push_back(dists); //saved this scan

                inputFile >> data; //for 0
                 inputFile >> data;//laser_pose_x
                inputFile >> data;//laser_pose_y
                inputFile >> data;//laser_pose_theta

                //reading odometry information
                inputFile >> x;//robot_pose_x
                inputFile >> y;//robot_pose_y
                inputFile >> theta;//robot_pose_theta

                singleRobotPosition.set(x, y);
                singleRobotPosition.setOAngle(theta);
                cout<<"odo: "<<x<<" "<<y<<endl;

                allRobotPosition.push_back(singleRobotPosition);

                if (scanID > 1) {
                    distFromLastStep = getDistBtw2Points(singleRobotPosition.X(), singleRobotPosition.Y(), lastX, lastY);
                    rotation = ( lastAngle - singleRobotPosition.getOAngle() ) * 180.0;
                    rotation = rotation / PI; //rotation in degree
                }

                if (scanID == 1 || distFromLastStep >= 1.0 || abs(rotation) >= 10.0) { //(1000 means 1m apart)

                    //converting laser reading from Polar to Cartesian for the current scan
                    for (unsigned int j = dists.size(); j-- > 0; ) {
                        theta = 180 - j; //angle with respect to (+)x-axis, robot is facing toward (+)y-axis.
                        theta = (theta / 180) * PI; //angle in radian
                        dist = dists[j]*1000; //converted to mm
                        x = dist * cos(theta);
                        y = dist * sin(theta);
                        if(dist < 30000.0) //filtering faraway points
                         laserPoints.push_back(PointXY(-x, y));
                        cout << j << " " << dists[j] << " " << x << " " << y << endl;
                    }
                    surfaces = Laser2Surface(laserPoints, clusterThreshold, surfaceSize, errorThreshold);
                    //writing surface file for the current view
                    sprintf(surfaceFileName, "%s%s%d", addForAlbot1, "surfaces-", scanID);
                    writeASCII(surfaces, surfaceFileName);
                    laserPoints.clear();

                    //writing odo info for albot1
                    distang.clear();
                    distang.push_back(distFromLastStep*1000.0);
                    distang.push_back(-rotation);
                    sprintf(coordTransFileName, "%s%s%d", addForAlbot1, "coordTrans-", scanID);
                    writeASCII(distang, 2, coordTransFileName);

                    lastX = singleRobotPosition.X();
                    lastY = singleRobotPosition.Y();
                    lastAngle = singleRobotPosition.getOAngle();
                    cout<<"Save for "<<scanID<<endl;
                    scanID++;
                    //waitHere();
                }                
            }
            

        }
        cout << scanID << endl;
        vector<Object> dummy;
        //plotObjectsAndPoints("Maps/allRobotPositions.png", dummy, dummy, allRobotPosition);
    } else
        cout << "Error opening " << addForLogFile << " .." << endl;

    inputFile.close();
}