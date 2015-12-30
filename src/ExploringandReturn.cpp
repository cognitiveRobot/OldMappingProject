/**********************************************
 * File:   ExploringandReturn.cpp
 * Author: Hossain
 *Modified by: Wenwang P
 * Created on November 20, 2015, 6:44 PM
 **********************************************/

#include <iostream>
#include <stdio.h>
#include <dirent.h>
#include <vector>
#include <cmath>
#include <sstream>
#include <algorithm>
#include <fstream>

#include "readAndwriteASCII.H"
#include "Plotting.H"
#include "GeometricOp.H"

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
#include "GeometryFuncs.H"
#include "GeometricOp.H"

#include "Aria.h"
#include "Laser2Surface.H"
#include "RobotFuncs.H"


#define PI 3.14159265


using namespace std;
vector<double> moveToTheDestination(ArRobot&, double distance, double angle, int viewNumber, vector<int> limitingPoints);
//void ReturningDecision(vector<Object> ReturingView); // return function


ArRobot robot;
ArSick sick;

unsigned char return_Flag = 0; // return execution flag 
unsigned char homeFlag = 0;   // home flag
int vnumb = 1;
char Flag_LM;

vector<Object> NewLM; // a new local map
vector<Object> PreLM;  // previous local map
vector<Surface> currentPG;  // the polygon of current local map

vector<Object> LandMarks;

double crx = 0;  // robot position in local map coordinate
double cry = 0;  // 
double dist_currentLM = 0; // accumulate distance in a local map
double ang_currentLM = 0; // accumulate angle in a local map
double angl = 0 ; // accumulating angle
double angle = 0;
double robotFacing;

unsigned char PinLM_Flag = 0; // the flag of previous position of robot is in local map 0 -- not 1 -- yes

vector<Object> currentView_Return;
vector<Object> LocalMap;
vector<PointXY> exitPosition;
vector < vector<Object> > Memories;

int main(int argc, char **argv) 
{

            //    if (argc < 7) {
            //        cout << endl << "Command: AutonomousXplorationAndMapping -rp /dev/ttyUSB0 -lp /dev/ttyUSB1 -li half" << endl << endl;
            //        return 0;
            //    }


                //CODE ADDED HERE BY PASCAL
                remove("Maps/OdometerData.txt");

            srand(getpid());

            cout << "argc: " << argc << " argv: " << argv << endl;

            Aria::init();
            ArSimpleConnector connector(&argc, argv);

            if (!connector.parseArgs() || argc > 1) {
                Aria::logOptions();
                Aria::shutdown();
                Aria::exit(1);
            }
            /*
                    ArKeyHandler keyHandler;
                    Aria::setKeyHandler(&keyHandler);
                    robot.attachKeyHandler(&keyHandler);
             */
            robot.addRangeDevice(&sick);
            // Try to connect, if we fail exit
            if (!connector.connectRobot(&robot)) {
                cout << "Could not connect to robot... exiting" << endl;
                Aria::shutdown();
                return 1;
            }
            // Turn on the motors, turn off amigobot sounds
            robot.runAsync(true);
            robot.comInt(ArCommands::ENABLE, 1);
            robot.comInt(ArCommands::SOUNDTOG, 0);
            robot.lock();
            robot.clearDirectMotion();
            robot.unlock();
            // Set up the laser
            connector.setupLaser(&sick);
            sick.runAsync();
            if (!sick.blockingConnect()) {
                cout << "Could not connect to SICK laser... exiting" << endl;
                robot.disconnect();
                Aria::shutdown();
                return 1;
            }
            ArUtil::sleep(1500);
            cout << "----------------Connected to robot and laser-------------------" << endl;

            int v; //, w, level, set;
            //    cout << "Which level you would like to map" << endl
            //            << "'0' for groudlevel" << endl
            //            << "'1' for firstlevel " << endl
            //            << "set 1 -- 999 for Guided xploration" << endl
            //            << "set  1001 --  for autonomous xploration" << endl << endl;
            //    level = 1; //cin >> level;
            //    cout << "Which Dataset ??? ";
            //    cin >> set;
            //    cout << "How far to go????????? (e.g. 1 71)" << endl << endl;
            //    cin >> v;
            //    cin >> w;
            v = 1;

            bool computeASR = false;

            //variables
            Transporter recognizedTargetObjects, computedOutput;
            vector <double> coordTransInfo;
            vector<Object> targetObjectsInPV, targetObjectsInCV;
            vector<Object> allTargetObjectsInPV;
            vector<Object> referenceObjects, odometricReferenceObject;
            ;
            vector<Object> currentRobotPositionInMFIS, odometricCRPositionInMFIS;
            vector<Object> previousRobotPositionInMFIS;
            vector<Object> routeMap;
            vector<Object> routeMapForOneASR;
            vector<Object> routeMapConnLP; //route map simply connecting limiting points
            vector<vector<Object> > routeMapForallASR;
            Object lastRouteMapNode, tempLastRouteMapNode;
            vector <Object> allRobotPositions;
            vector<Object> robotPositionsAtLimitingPoints;
            Transporter objectForPlaceRecognition;
            Transporter loopClosingInfo;
            Transporter lastStepInfo;
            vector<Object> refObjectForLoopClosing;
            string environmentType = "unknown";
            vector<int> lostPoints, limitingPoints, exitPoints;
            Object lineOfSitePoint;
            limitingPoints.push_back(1);
            Object lastLocomotion;
            vector<Object> wholeRoute;
            double traveledDistance = 0;
            double angleError, distanceError;

            vector<Exit> exitsFromCV;
            vector<Object> exitsFromCVInMFIS, allExitsInMFIS, crossedExit;

            vector<Object> objectOfCurrentASR;
            ASR currentASR;
            ASRNetwork perceptualMap;
            int ASRNumber = 1;

            vector<ASR> places;

            MyRobot myrobot(0, 0);
            currentRobotPositionInMFIS = myrobot.getRobot();
            allRobotPositions = currentRobotPositionInMFIS;
            robotPositionsAtLimitingPoints = currentRobotPositionInMFIS;

            //for routeMap
            previousRobotPositionInMFIS = currentRobotPositionInMFIS;
            lastRouteMapNode = currentRobotPositionInMFIS[6];
            tempLastRouteMapNode = lastRouteMapNode;
            routeMap.push_back(lastRouteMapNode);
            routeMapForOneASR.push_back(lastRouteMapNode);

            //save coordinate transformation info
            vector<double> distang;
            distang.push_back(0); //rdist);
            distang.push_back(0); //rangle);
            char coordTransFileName[100];
            char viewFileName[80], mfisFileName[80];
            sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", v);
            writeASCII(distang, 2, coordTransFileName);

            //scanning for the current view
            vector <Object> currentView = scanAndSaveView(sick, v);
            
            PreLM = NewLM = currentView;          // first local map
            currentPG = makePolygonOfCV(NewLM); // first local to generate polygon
            Memories.push_back(NewLM); //store the first local map as first memory

            //finding target objects
            targetObjectsInPV = findTargetObjects(currentView);
            referenceObjects.push_back(targetObjectsInPV[0]); //bcz ref objects will be used in findNextDestination function
            referenceObjects.push_back(targetObjectsInPV[0]);

            //tagging side and view number
            currentView = tagObjectsAsSideAndViewNumber(currentView, 1);

            for (unsigned int i = 0; i < currentView.size(); i++) {
                currentView[i].setASRNo(1);
            }

            //initializing MFIS
            vector<Object> MFIS = currentView;
            cout << "..........MFIS.........." << endl;
            displayObjects(MFIS);
            //waitHere();

            //initializing Perceptual Map
            objectOfCurrentASR = currentView;
            currentASR.setASRObjects(objectOfCurrentASR);
            currentASR.setASRExit1(Object(-500, 0, 500, 0));
            currentASR.setASRID(1);
            lineOfSitePoint = currentRobotPositionInMFIS[6];
            currentASR.addLineOfSitePoints(currentRobotPositionInMFIS[6]);
            perceptualMap.setCurrentASR(currentASR);

            sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
            plotObjects(mfisFileName, allRobotPositions, MFIS);
            sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
            plotObjects(mfisFileName, allRobotPositions, MFIS);
            sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
            plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);
            sprintf(viewFileName, "%s", "Maps/CurrentView.png");
            plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInPV);

            bool exitCrossed = false;
            crossedExit.push_back(currentRobotPositionInMFIS[6]); //consider home as a exit

            cout << "\n\033[1;34m******************MFIS is computed at step" << v << "********************\033[0m" << endl << endl;
            char n = 'y';
            double angleToMove;
            double distanceToMove;

                //CHANGES DONE BY PASCAL CROSS
                std::ofstream outfile;
                outfile.open("Maps/OdometerData.txt", std::ios_base::app);
                outfile << 1;
                outfile << endl;
                outfile << 0;
                outfile << endl;
                outfile << 0;
                outfile << endl;
                outfile << "true";
                outfile << endl;


                ///// Here, this function is to ask whether let the robot carry on working
            while (n != 'n' && n != 'N') 
            {
                v++;

                cout << endl << "How much to turn? ";
                cin >> angleToMove;
                cout << "How much to move? ";
                cin >> distanceToMove;

                cout << "Now @ step " << v << endl;
                coordTransInfo = moveToTheDestination(robot, distanceToMove, angleToMove, v, limitingPoints);
                traveledDistance = traveledDistance + coordTransInfo[0];
                robotFacing = robotFacing + coordTransInfo[1];                 //The second element.
                //scanning for the current view
                currentView = scanAndSaveView(sick, v);
               
                cout << v << " coordTrans d- " << coordTransInfo[0] << " a- " << coordTransInfo[1] << endl;

                angle = (coordTransInfo[1] / 180) * PI;
                if(PinLM_Flag == 0)
                {   
                        crx = coordTransInfo[0] * sin(-angle); 
                        cry = coordTransInfo[0] * cos(-angle);
                }
                else
                {
                        crx += coordTransInfo[0] * sin(-angle); 
                        cry += coordTransInfo[0] * cos(-angle);
                }
                
                //finding Exits
                exitsFromCV = findShortestExits(currentView);
                //finding target objects
                targetObjectsInCV = findTargetObjects(currentView);
                //tagging sides and view number
                currentView = tagObjectsAsSideAndViewNumber(currentView, v);

                //saving current view
                sprintf(viewFileName, "%s%d%s", "Maps/view-", v, ".png");
                plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);
                sprintf(viewFileName, "%s", "Maps/CurrentView.png");
                plotObjectsOf3Kinds(viewFileName, currentView, myrobot.getRobot(), targetObjectsInCV);

                //recognizing target Objects
                recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
                referenceObjects = recognizedTargetObjects.getReferenceObjects();

                cout << "(After recognition) no of targetObjects for next step " << recognizedTargetObjects.getTargetObjects().size() << endl;
                cout << "(After recognition) ref objects " << recognizedTargetObjects.getReferenceObjects().size() << endl;

                //localization using odometer
                Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
                aLine.setKP(1);
                odometricReferenceObject.clear();
                odometricReferenceObject.push_back(aLine);
                odometricReferenceObject.push_back(allRobotPositions[6]);
                odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0], odometricReferenceObject[1], odometricReferenceObject[0].getKP());

                if (referenceObjects.size() > 0)
                {
                        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
                        //localization error checking
                        angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(currentRobotPositionInMFIS[6]));
                        distanceError = odometricCRPositionInMFIS[6].distP1ToP1(currentRobotPositionInMFIS[6]);

                        if (distanceError > 400.0 or angleError > 5.0) 
                        {
                            referenceObjects = odometricReferenceObject;
                            currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                        }
                } 
                else 
                {
                        referenceObjects = odometricReferenceObject;
                        currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                }
                allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

                //update MFIS and ASR if necessary
                if (recognizedTargetObjects.getTargetObjects().size() < 3) 
                {
                        cout << endl << "Updating situation " << endl;
              
                        if ((v - limitingPoints.back()) > 1)  //update at last step //retriving info to update at last step
                        {
                                cout << "Updating at last step. i.e. @ " <<v-1<< endl << endl;
                                //MFIS = lastStepInfo.getMFIS();
                                //currentView = lastStepInfo.getView();
                                //referenceObjects = lastStepInfo.getReferenceObjects();
                                //currentRobotPositionInMFIS = lastStepInfo.getRobotPosition();
                                v = v - 1;

                                //updating at last step
                                computedOutput = updatePerceptualMapATPlace(places, lastStepInfo.getMFIS(), lastStepInfo.getView(),
                                        lastStepInfo.getRobotPosition(), lastStepInfo.getAllRobotPositions(), lastStepInfo.getReferenceObjects(),
                                        v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                                MFIS = computedOutput.getView();
                                places = computedOutput.getASRs();
                                targetObjectsInPV = computedOutput.getTargetObjects();
                                allRobotPositions = lastStepInfo.getAllRobotPositions();
                                limitingPoints.push_back(v); //limiting/updating points just for printing at the end

                                sprintf(mfisFileName, "%s%d%s", "Maps/PM-u-", v, ".png");
                                plotObjects(mfisFileName, allRobotPositions, MFIS);
                                sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
                                plotObjects(mfisFileName, allRobotPositions, MFIS);
                                    
                                /* judging the position of robot in polygon */
                                PointXY robotP(crx, cry); // robot position is going to be detected in a particular local map

                                Flag_LM = pointInPolygon(robotP, currentPG, 0); // Whether the robot position in local map
                                if(!Flag_LM)
                                {
                                         currentPG.clear();
                                         NewLM = lastStepInfo.getView();
                                         LandMarks= findTargetObjects(NewLM);
                                         currentPG = makePolygonOfCV(NewLM); // make a new polygon/boundary of local environment
                                         Memories.push_back(NewLM);                  // all local environments are memories
                                         crx -= coordTransInfo[0] * sin(-angle); 
                                         cry -= coordTransInfo[0] * cos(-angle);
                                         PointXY robotP(crx,cry);
                                         exitPosition.push_back(robotP);
                                            
                                         sprintf(viewFileName, "%s%d%s", "Maps/Offline/LocalMap-", v-1, ".png");
                                         //plotObjects(viewFileName, myrobot.getRobot(), NewLM);         
                                         plotObjectsOf3Kinds(viewFileName, NewLM, myrobot.getRobot(), LandMarks);
                                         PinLM_Flag = 0;
                                }
                                else
                                         PinLM_Flag = 1;
  
                                v++;
                                //now processing current view
                                //recognizing target Objects
                                recognizedTargetObjects = recognizeTargetObjects(MFIS, targetObjectsInPV, targetObjectsInCV, coordTransInfo, v);
                                referenceObjects = recognizedTargetObjects.getReferenceObjects();

                                if (referenceObjects.size() > 0) 
                                {
                                        currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
                                        //localization error checking
                                        angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(currentRobotPositionInMFIS[6]));
                                        distanceError = odometricCRPositionInMFIS[6].distP1ToP1(currentRobotPositionInMFIS[6]);
                                        if (distanceError > 400.0 or angleError > 5.0) {
                                            referenceObjects = odometricReferenceObject;
                                            currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                                        }
                                } 
                                else 
                                {
                                        referenceObjects = odometricReferenceObject;
                                        currentRobotPositionInMFIS = odometricCRPositionInMFIS;
                                }
                                allRobotPositions = addTwoVectorsOfObjects(allRobotPositions, currentRobotPositionInMFIS);

                                if (recognizedTargetObjects.getTargetObjects().size() < 3) 
                                {
                                        //updating at current step
                                        computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                                                allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                                        MFIS = computedOutput.getView();
                                        places = computedOutput.getASRs();
                                        targetObjectsInPV = computedOutput.getTargetObjects();
                                        limitingPoints.push_back(v);       //limiting/updating points just for printing at the end
                                } 
                                else 
                                {
                                     targetObjectsInPV = recognizedTargetObjects.getTargetObjects();
                                }
                                

                    } 
                    else 
                        if ((v - limitingPoints.back()) == 1) //update at this step
                        {   
                                //updating at current step
                                computedOutput = updatePerceptualMapATPlace(places, MFIS, currentView, currentRobotPositionInMFIS,
                                        allRobotPositions, referenceObjects, v, ASRNumber, exitCrossed, crossedExit, routeMapConnLP);
                                MFIS = computedOutput.getView();
                                places = computedOutput.getASRs();
                                targetObjectsInPV = computedOutput.getTargetObjects();
                                limitingPoints.push_back(v); //limiting/updating points just for printing at the end
                         }
                } 
                else
                    targetObjectsInPV = recognizedTargetObjects.getTargetObjects();


                     
                cout << "\n\033[1;34m******************PM is computed at step" << v << "********************\033[0m" << endl << endl;

                //save lastStep information to update at next step(in case)
                lastStepInfo.setMFIS(MFIS);
                lastStepInfo.setView(currentView);
                lastStepInfo.setTargetObjects(targetObjectsInCV);
                lastStepInfo.setRobotPosition(currentRobotPositionInMFIS);
                lastStepInfo.setAllRobotPositions(allRobotPositions);
                lastStepInfo.setReferenceObjects(referenceObjects);
                lastStepInfo.setExits(exitsFromCV);

                sprintf(mfisFileName, "%s%d%s", "Maps/PM-", v, ".png");
                plotObjects(mfisFileName, allRobotPositions, MFIS);
                sprintf(mfisFileName, "%s", "Maps/PerceptualMap.png");
                plotObjects(mfisFileName, allRobotPositions, MFIS);

                cout << "Take another step? (y/n) ";
                cin >> n;
            }

            cout << "Traveled Dist: " << traveledDistance << endl;
            currentRobotPositionInMFIS = myrobot.inMFIS(referenceObjects[0], referenceObjects[1], referenceObjects[0].getKP());
            pointingExperiment(MFIS,allRobotPositions,currentRobotPositionInMFIS);
            //keyInfoOfCompuetedPM(ASRNumber, exitPoints, lostPoints, limitingPoints);
            
            if (n == 'n' || n == 'N') 
            {
                    //robot.disconnect();
                    //Aria::shutdown();
                    cout << "Would you return right now ??" << endl << endl;
                    cin >> return_Flag;

                    // start returning travel 
                    if(return_Flag == 1)
                    {   
                            //setHeading(robot, 180); // robot turns over
                            while(!homeFlag)
                            {
                                    //scanning for the current view when robot returns 
                                    currentView_Return = scanAndSaveView(sick, vnumb);



                                    //ReturningDecision(currentView_Return);
                                    vnumb++;
                            }
                    }
                    else // stop and shutdown the robot
                    {
                            robot.disconnect();
                            Aria::shutdown();
                    }
            }
    
    
    
    return 0;
}


// movement funciton in Guided mode
vector<double> moveToTheDestination(ArRobot& robot, double distance, double angle, int viewNumber, vector<int> limitingPoints) {

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //save obometer and orientation angle before moving       
    robot.lock();
    double oldDistance = robot.getOdometerDistance();
    double oldAngle = robot.getTh();
    robot.unlock();

    //movement execution
    setHeading(robot, angle);
    moveDistance(robot, distance);
    angle = 0;

    // Stop the robot and wait a bit
    robot.lock();
    robot.stop();
    robot.clearDirectMotion();
    robot.unlock();
    ArUtil::sleep(2000);

    //get the actual distance traveled for this step
    robot.lock();
	//DATA HERE your name
    double traveledDistance = robot.getOdometerDistance() - oldDistance;
    double turnedAngle = robot.getTh() - oldAngle;
    robot.unlock();

    vector<double> distang;
    distang.push_back(traveledDistance); //rdist);
    distang.push_back(turnedAngle); //rangle);
    char coordTransFileName[100];
    sprintf(coordTransFileName, "%s%d", "inputData/coordTrans-", viewNumber);
    writeASCII(distang, 2, coordTransFileName);

    cout << "Traveled Dist: " << traveledDistance << " turned Angle: " << turnedAngle << endl;
	
	//CHANGES DONE BY PASCAL CROSS
	std::ofstream outfile;
	outfile.open("Maps/OdometerData.txt", std::ios_base::app);
	outfile << viewNumber;
	outfile << endl;
	outfile << traveledDistance;
	outfile << endl;
	outfile << turnedAngle;
	outfile << endl;
	if(std::find(limitingPoints.begin(), limitingPoints.end(), viewNumber) != limitingPoints.end()) {
		outfile << "true";
	} else {
		outfile << "false";
	}
	outfile << endl;
	
	
    return distang;
}

/*
void ReturningDecision(vector<Object> ReturingView)
{
    
         double angleToH = 0;
         double disToH = 0;
          vector<Exit> exitsInview;
         vector<Object> recognisedLandmarks;
        
         
         
         /*finding current potential exit that is accessing to the in local view*/
         
         //finding Exits
         //exitsInview = findShortestExits(ReturingView);
         
         //recognizing target Objects
        // recognisedLandmarks = recognizeTargetObjects(MFIS, targetObjectsInPV, ReturingView, coordTransInfo, v);
        // referenceObjects = recognizedTargetObjects.getReferenceObjects();
         /*finding landmarks, which can guided robot to the relative exit that is to home position*/
         
         
         
         
         // Stop the robot and wait a bit
 /*        robot.lock();
         robot.stop();
         robot.clearDirectMotion();
         robot.unlock();
         ArUtil::sleep(2000);

         //save obometer and orientation angle before moving       
         robot.lock();
         double oldDistance = robot.getOdometerDistance();
         double oldAngle = robot.getTh();
         robot.unlock();

         

         
         
         
         cout << "The direction/angle is :" << angleToH << "And Distance will be :" << disToH << endl << endl;

         /*movement execution*/
  //       setHeading(robot, angleToH);
  ///       moveDistance(robot, disToH);
         
         
//}






