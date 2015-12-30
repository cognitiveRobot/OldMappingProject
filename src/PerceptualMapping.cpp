


#include <vector>
#include <iostream>
#include <valarray>

#include "PerceptualMapping.H"


using namespace std;

#define PI 3.14159265
int DELETING_ALGORITHM = 1;
bool PRINT_UPDATING_STAGES = false;

Transporter updatePerceptualMap(vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS, vector<Object> refobjects, int viewNumber, int ASRNumber) {
   // cout << BOLDGREEN<<"-------------------Inside updatePerceptualMap module---------------------"<<RESET << endl;

    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;

    //computing all current view objects in MFIS
    int currentViewNumber = cView[0].getVN();
    Object temp;
    for (unsigned int i = 0; i < cView.size(); i++) {
        temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
        if (cView[i].getPos() == 1)
            temp.setPos(1);
        else
            temp.setPos(-1);
        temp.setVN(currentViewNumber);
        temp.setOoPV(true);
        currentViewInMFIS.push_back(temp);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0) {
        cout << "Current ASR is empty" << endl;
        //no need to go down
        for (unsigned int i = 0; i < currentViewInMFIS.size(); i++) {
            currentViewInMFIS[i].setASRNo(ASRNumber);
        }
        Transporter package;
        package.setView(currentViewInMFIS);
        return package;
        //waitHere();
    }
    // if(viewNumber == 39)

    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);


    vector<double> boundariesOfCV = findBoundariesOfCV(cView,0.0);


    vector<Object> exppandableObjects;

    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    //Finding MFIS objects in CV
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) {
        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) {
            if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                perceptualMap[i].setPos(-1);
        }
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) {
            if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                perceptualMap[i].setPos(1);
        }

        //finding MFIS objects which are behind current robot position
        //if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getASRNo() == ASRNumber) {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (perceptualMap[i].getOoPV() == true || pMapOnCV[i].isThisInsideCV(boundariesOfCV) == true)) {
            //if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getOoPV() == true) {
            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV

            if (pMapOnCV[i].length() > 3000 && viewNumber == 128) {//special case for set11 and first corridor 
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
            }

        } else {
            objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
            objectsBehindRobot.back().setOoPV(false);



            //finding expandable objects on LEFT
            if (perceptualMap[i].getPos() == -1) {
                if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) {
                    //if(expandableOnLeft == 0) {
                    expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                    // }
                    exppandableObjects.push_back(perceptualMap[i]);
                    cout << "LEFT: " << perceptualMap[i].getID() << endl;
                }
            }

            //finding expandable objects on RIGHT
            if (perceptualMap[i].getPos() == 1)
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0) {
                    if (expandableOnRight == 0) {
                        expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                        exppandableObjects.push_back(perceptualMap[i]);
                        cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                    }
                }
        }


    }

    //plotObjects("Maps/expandableObects.png",firstRobotPosition,exppandableObjects);
    cout << "Current view in MFIS" << endl;
    //displayObjects(currentViewInMFIS);
    cout << "Obects behind CRP" << endl;
    //displayObjects(objectsBehindRobot);

    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);

    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);
    cout << endl << endl << "Target Objects in CV " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if ((abs(expAngle) < 6 || abs(expAngle) > 354) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnRight) {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    cout << "Angle: " << expAngle << " dist: " << expDist << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 6 || abs(expAngle) > 354) && expDist < 500 && currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                cout << "Hello" << endl;
                break;
            }
        }
    }

    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }

    //inserting NEW objects
    cout << "InsertTo: " << insertTo << endl;
    cout << "size: " << objectsBehindRobot.size() << endl;
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        updatedPM.back().setASRNo(ASRNumber);

        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;
    // displayObjects(updatedPM);
    // plotObjects("Maps/updatedMFIS.png", updatedPM, firstRobotPosition);
    //waitHere();

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    return package;
}

//it's THE new version of updatePerceptualMap
//all surfaces are tagged as their place number
//place is initiated when robot crosses a exit

Transporter updatePerceptualMapATPlace(vector<ASR> places, vector<Object> perceptualMap, vector<Object> cView, vector<Object> cRobotPositionInMFIS,
        vector <Object> allRobotPositions, vector<Object> refobjects, int viewNumber, int ASRNumber,
        bool exitCrossed, vector<Object> crossedExit, vector<int> limitingPoints, vector<Object> routeMap4CPlace) {
    cout << "\033[1;32m-------------------Inside updatePerceptualMap module---------------------\033[0m" << endl;

    int lastLimitingPoint = limitingPoints.back();
    MyRobot myrobot(0, 0);
    vector<Object> firstRobotPosition = myrobot.getRobot();

    //cout << endl << endl << "PerceptualMap" << endl;
    //displayObjects(perceptualMap);

    //cout << endl << endl << "Current View" << endl;
    //displayObjects(cView);
    cout << endl << endl << "Ref objects size: " << refobjects.size()<< endl;
    displayObjects(refobjects);

    Object xAxisAtCRP = cRobotPositionInMFIS[7];
    vector<Object> updatedPM; //will be return as new MFIS
    vector<Object> objectsBehindRobot;
    vector<Object> objectsBehindRobotOnCV;
    vector<Object> objectsInSideCV;
    vector<Object> currentViewInMFIS;

    //computing all current view objects in MFIS
    cout << "transforming all cv surfaces to MFIS" << endl;
    cView[0].display();
    int currentViewNumber = cView[0].getVN();
    Object temp;
    int cViewSize = cView.size();
    //    if(viewNumber == 87) //bug
    //        cViewSize = cView.size()-1;
    //bug. need to fix. 
    if(viewNumber == 128) {//stupid bug. need to change. exp1 part1. small rectangle. deleting last object.
//        cView = deleteObject(cView, cView[cView.size()-1].getID());
//        cView = deleteObject(cView, cView[cView.size()-1].getID());
    }
    for (unsigned int i = 0; i < cView.size(); i++) {

        temp = remakeLineP2(refobjects[0], refobjects[1], cView[i], i + 1, 0, refobjects[0].getKP());
        if (cView[i].getPos() == 1)
            temp.setPos(1);
        else
            temp.setPos(-1);
        temp.setVN(currentViewNumber);
        temp.setOoPV(true);
        temp.setASRNo(ASRNumber);
        temp.setLimitingPoint(viewNumber);
        temp.setLocalEnvID(viewNumber);
        temp.setPO(cView[i].getPO());
        currentViewInMFIS.push_back(temp);
    }

    //demonstrating error during updating (only for print)
    if (refobjects[1].getColorID() == 100) {
        vector<Object> dummy;
        char mfisNCVfileName[80];
        sprintf(mfisNCVfileName, "%s%d%s", "Maps/MFISandCurrentView-", viewNumber, ".png");
        dummy.push_back(refobjects[0]);
        //plotObjectsOf4Kinds(mfisNCVfileName, cRobotPositionInMFIS, dummy, currentViewInMFIS, perceptualMap);
        sprintf(mfisNCVfileName, "%s%d%s", "Maps/CurrentView-", viewNumber, ".png");
        dummy.clear();
        dummy.push_back(refobjects[1]);
        //plotObjectsOf3Kinds(mfisNCVfileName, myrobot.getRobot(), dummy, cView);
        //waitHere();
    }


    //constructing place     
    if (exitCrossed == true ){//or viewNumber == 163) {
        cout << "Constructing place !!" << endl;
        cout << "crossedExits: " << crossedExit.size() << endl;
        char placeFileName[80];
        vector<Object> place;

        ASR onePlace;
        vector<Object> exits;
        sprintf(placeFileName, "%s%d%s", "Maps/place-", ASRNumber - 1, ".png");
        for (unsigned int i = 0; i < perceptualMap.size(); i++) {
            if (perceptualMap[i].getASRNo() == ASRNumber - 1)
                place.push_back(perceptualMap[i]);
        }

        //adding exits
        exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 2])); //exit1
        exits = addTwoVectorsOfObjects(exits, makeSquare(crossedExit[crossedExit.size() - 1])); //exit2
        plotObjectsOf3Kinds(placeFileName, exits, routeMap4CPlace, place);

        onePlace.setASRObjects(place);
        onePlace.setASRExit1(crossedExit[crossedExit.size() - 2]);
        onePlace.setASRExit2(crossedExit[crossedExit.size() - 1]);
        onePlace.replaceTheWholeRoute(routeMap4CPlace); //this route just connecting limiting points
        places.push_back(onePlace);
    }


    //for a fresh ASR
    if (perceptualMap.size() == 0) {
        cout << "Current ASR is empty" << endl;
        //no need to go down
        Transporter package;
        package.setView(currentViewInMFIS);
        return package;
    }


    //computing all MFIS objects on CV to find objects in Current view
    Point currentRobPoint; //Point for current robot position in MFIS
    currentRobPoint.set(xAxisAtCRP.X1(), xAxisAtCRP.Y1());
    double angle = xAxisAtCRP.getAngleWithXaxis(); //Angle btw current robot x-axis and Original x-axis
    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);
    vector<Object> pMapOnCV = xformPVIntoCV(perceptualMap, currentRobPoint, angle);

    //  plotObjects("Maps/MIFSonCV.png", cRobotPositionInMFIS, pMapOnCV);    
    vector<Object> pMapOnCVinsideCV;
    //making polygon of CV
    cout << "Finding cv boundary !!" << endl;
    //vector<Surface> polygon = makePolygonOfCV(cView); //findExactBoudaryFrom(cView);//
    vector<Surface> polygon = constructPolygon(cView);
    vector<Object> polygonObjects = convertSurfaceToObject(polygon); //only for printing
    
//    vector<Object> temps;
//    for(unsigned int i=0;i < polygonObjects.size(); i++) {
//        temps.push_back(polygonObjects[i]);
//        plotObjects("Maps/polygonGrowing.png",cView,temps);
//        cout<<"Polygon Growing"<<endl;
//        waitHere();
//    }
//    plotObjects("Maps/polygonAndCV.png",cView,polygonObjects);
//    cout<<"polygon and cv"<<endl;
//    waitHere();
    vector<Object> polygonObjectsOnMFIS; //for demonstration of wiping out algorithm
    for (unsigned int i = 0; i < polygonObjects.size(); i++) {
        temp = remakeLineP2(refobjects[0], refobjects[1], polygonObjects[i], i + 1, 0, refobjects[0].getKP());
        polygonObjectsOnMFIS.push_back(temp);
    }
    //plotObjects("Maps/CV-Boundary.png",cView,polygonObjects);
    //plotObjectsOf3Kinds("Maps/Polygon.png",xformPVIntoCV(allRobotPositions, currentRobPoint, angle),
    //breakTheLinesInto(polygonObjects),pMapOnCV);    
    //waitHere();

    vector<Object> exppandableObjects;
    vector<int> lpToDelete; //limiting points to be deleted.
    vector<Object> objectsBelongToSameSpace;
    vector<Object> deletedObjectsOnCV;

    //saving local environment number 
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) {
        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true)) {

            if (lpToDelete.size() == 0) {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++)
                    lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
            } else {
                for (unsigned int le = 0; le < perceptualMap[i].getLocalEnvID().size(); le++) {
                    bool alreadySaved = false;
                    for (unsigned int lp = 0; lp < lpToDelete.size(); lp++) {
                        if (lpToDelete[lp] == perceptualMap[i].getLocalEnvID()[le]) {
                            alreadySaved = true;
                            break;
                        }
                    }
                    if (alreadySaved == false)
                        lpToDelete.push_back(perceptualMap[i].getLocalEnvID()[le]);
                }
            }
        }
    }

    //finding MFIS objects on CV. first three conditions are to tag surfaces according to their
    //orientation w.r.t. robot(e.g. left or right)
    int expandableOnLeft = 0;
    int expandableOnRight = 0;
    cout << "Finding MFIS objects in CV !!" << endl;
    //    cout<<"PMonCV"<<endl;
    //    displayObjects(pMapOnCV);
    for (unsigned int i = 0; i < pMapOnCV.size(); i++) {

        //changing position tag of those objects which appeared on opposite side but now should be other side
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() < 0 && perceptualMap[i].getPos() == 1 && perceptualMap[i].getOoPV() == true) {

            if (i == 0)
                perceptualMap[i].setPos(-1);
            else if (perceptualMap[i - 1].getPos() == -1)//some objects position is be changed when robot takes big turn
                perceptualMap[i].setPos(-1);
        }
        //changing position tag
        if (pMapOnCV[i].X1() > 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) {

            if (i == 0)
                perceptualMap[i].setPos(1);
            else if (perceptualMap[i - 1].distP2ToP1(perceptualMap[i]) > 600 or perceptualMap[i - 1].getPos() == 1)
                perceptualMap[i].setPos(1);
        }

        //changing position tag
        if (pMapOnCV[i].X1() < 0 && pMapOnCV[i].X2() > 0 && perceptualMap[i].getPos() == -1 && perceptualMap[i].getOoPV() == true) {

            //if (viewNumber == 84)
               // perceptualMap[i].setPos(1);
        }

        //finding MFIS objects which are behind current robot position
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && perceptualMap[i].getOoPV() == true) {
        //       if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0 && 
        //                perceptualMap[i].getASRNo() == ASRNumber) {
        //       if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (perceptualMap[i].getOoPV() == true || 
        //            pMapOnCV[i].isThisInsideCV(boundariesOfCV) == true)) {
        //        if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
        //                (pointInPolygon(PointXY(pMapOnCV[i].X1(),pMapOnCV[i].Y1()),polygon) == true || 
        //                pointInPolygon(PointXY(pMapOnCV[i].X2(),pMapOnCV[i].Y2()),polygon) == true)) {
        if ((pMapOnCV[i].Y1() > 0.0 && pMapOnCV[i].Y2() > 0.0) &&
                (pointInPolygon(PointXY(pMapOnCV[i].X1(), pMapOnCV[i].Y1()), polygon) == true ||
                pointInPolygon(PointXY(pMapOnCV[i].X2(), pMapOnCV[i].Y2()), polygon) == true ||
                isThisSurfaceAcrossThePolygon(polygonObjects,pMapOnCV[i]) == true || 
                isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true)) {//inside or close to cv
            // cout<<"Checking for inside!!"<<endl;
            objectsInSideCV.push_back(perceptualMap[i]); //MFIS objects which are inside CV
            pMapOnCVinsideCV.push_back(pMapOnCV[i]); //only for printing
            deletedObjectsOnCV.push_back(pMapOnCV[i]);
            


            //            if (pMapOnCV[i].length() > 3000 && viewNumber == 128) {//special case for set11 and first corridor 
            //                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
            //                objectsBehindRobot.back().setOoPV(false);
            //            }

        } else {//outside cv
            //   cout<<"Checking for space id!!"<<endl;
            //           if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                   isBelongToSameSpace(perceptualMap[i].getLimitingPoint(),lpToDelete)==true) {

            //            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && 
            //                    isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true &&
            //                    isThisCloseToCurrentView(pMapOnCV[i],cView) == true) {//condition to delete surfaces which belong to same localEnv
            if ((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 200.0) == true) {
                //            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true ) {
                //            if((pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() > 0) && (checkForIntersection(Object(0,0,0,2000),pMapOnCV[i]) || (isBelongToSameSpace(perceptualMap[i].getLocalEnvID(),lpToDelete)==true && 
                //                      isThisCloseToCVPolygonBoundary(pMapOnCV[i], polygonObjects, 2500.0) == true))) {
                objectsBelongToSameSpace.push_back(perceptualMap[i]);
                deletedObjectsOnCV.push_back(pMapOnCV[i]);
            } else {
                //  cout<<"its outside of cv!!"<<endl;
                objectsBehindRobot.push_back(perceptualMap[i]); //objects behind Current robot position including objects on x-axis
                objectsBehindRobot.back().setOoPV(false);
                objectsBehindRobotOnCV.push_back(pMapOnCV[i]);
            }


            //finding expandable objects on LEFT
            if (perceptualMap[i].getPos() == -1) {
                if ((pMapOnCV[i].Y1() < 0 && pMapOnCV[i].Y2() > 0) || (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0)) {
                    //if(expandableOnLeft == 0) {
                    expandableOnLeft = perceptualMap[i].getID(); //storing last expandable objectID
                    // }
                    exppandableObjects.push_back(perceptualMap[i]);
                    cout << "LEFT: " << perceptualMap[i].getID() << endl;
                }
            }

            //finding expandable objects on RIGHT
            if (perceptualMap[i].getPos() == 1)
                if (pMapOnCV[i].Y1() > 0 && pMapOnCV[i].Y2() < 0) {
                                            //perceptualMap[i].getID() != 29 is for dataset 1
                    if (expandableOnRight == 0 && perceptualMap[i].getID() != 29) {
                        expandableOnRight = perceptualMap[i].getID(); //storing last expandable objectID
                        exppandableObjects.push_back(perceptualMap[i]);
                        cout << "RIGHT: " << perceptualMap[i].getID() << endl;
                    }
                }
        }
        
//        if(viewNumber == 505 && perceptualMap[i].getID() == 6980)
//            waitHere();
    }

    //just for demonstration
    char mfisFileName[80];

    
    //not using - 29/09/2014
    if (objectsBelongToSameSpace.size() > 10) {

        sprintf(mfisFileName, "%s%d%s", "Maps/ObjectInsideCV-", viewNumber, ".png");
        //if(viewNumber == 242 or viewNumber == 864 or viewNumber == 1009 or viewNumber == 1425) 
        //plotObjectsOf4Kinds(mfisFileName,pMapOnCV,pMapOnCVinsideCV,objectsBelongToSameSpace,polygonObjects);
        //waitHere();

        cout << "Current view in MFIS" << endl;
        //displayObjects(currentViewInMFIS);
        cout << "Obects behind CRP" << endl;
        //displayObjects(objectsBehindRobot);
        //displayObjects(objectsBelongToSameSpace);

        cout << "lp " << lpToDelete.size() << " " << lpToDelete[0] << endl;
        //waitHere();
    }


    cout << "deletedObjects on CV" << endl;
    //displayObjects(deletedObjectsOnCV);

    cout << "CV in MFIS" << endl;
    //displayObjects(currentViewInMFIS);

    

    // plotObjects("Maps/objectBehindRnCV.png", objectsBehindRobot, firstRobotPosition);
   
    int lastObjectID = perceptualMap[perceptualMap.size() - 1].getID();
    vector<Object> targetObjectsInCV = findTargetObjects(cView);

    cout << endl << endl << "Number of Target Objects in CV: " << targetObjectsInCV.size() << endl;
    //displayObjects(targetObjectsInCV);
    vector<Object> targetObjectsForNextStep;


    double expAngle;
    double expDist;
    double similarity;
    int insertFrom = 0;
    int insertTo = currentViewInMFIS.size() + 1;


    //LEFT expanding objects 
    if (expandableOnLeft != 0) {//finding same object of cv on LEFT
        cout << "(LEFT) Object number " << expandableOnLeft << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnLeft) {
                for (unsigned int j = 0; j < currentViewInMFIS.size(); j++) {//will consider first same object of CV on LEFT
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 350.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == -1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 && currentViewInMFIS[j].getPos() == -1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        //new kind of extension. just extend old surface as it is. not averaging.
                        Point p1 = objectsBehindRobot[i].ppCordOfPoint(currentViewInMFIS[j].X2(),currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setP2(p1.X(),p1.Y());
                        //objectsBehindRobot[i].setP2(currentViewInMFIS[j].X2(), currentViewInMFIS[j].Y2());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        //objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertFrom = currentViewInMFIS[j].getID();

                        //finding whether this object is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                break;
            }
        }
    }

    //RIGHT side expending 
    if (expandableOnRight != 0) {
        cout << "(RIGHT) Object number " << expandableOnRight << " going to be expanded" << endl;
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
            if (objectsBehindRobot[i].getID() == expandableOnRight) {
                for (unsigned int j = currentViewInMFIS.size() - 1; j > 0; j--) {//will consider last same object of cv on RIGHT
                    cout << "finding same object from last " << j << endl;
                    expAngle = objectsBehindRobot[i].getAngleWithLine(currentViewInMFIS[j]);
                    expDist = shortestDistanceBtwTwoObjects(objectsBehindRobot[i], currentViewInMFIS[j]);
                    if (abs(expAngle) > 300)
                        similarity = (360.0 - abs(expAngle)) * expDist;
                    else
                        similarity = abs(expAngle) * expDist;
                    cout << "Angle: " << expAngle << " dist: " << expDist << " similarity: " << similarity << " Pos: " << currentViewInMFIS[j].getPos() << endl;
                    if ((abs(expAngle) < 10.0 || abs(expAngle) > 349.0) && expDist < 500.0
                            && currentViewInMFIS[j].getPos() == 1) {
                        //if ((abs(expAngle) < 7.5 || abs(expAngle) > 352.5) && expDist < 500 
                        //&& currentViewInMFIS[j].getPos() == 1) {
                        cout << "will be expanded as " << currentViewInMFIS[j].getID() << endl;
                        //new kind of extension. just extend old surface as it is. not averaging.
                        Point p1 = objectsBehindRobot[i].ppCordOfPoint(currentViewInMFIS[j].X1(),currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setP1(p1.X(),p1.Y());
                        //objectsBehindRobot[i].setP1(currentViewInMFIS[j].X1(), currentViewInMFIS[j].Y1());
                        objectsBehindRobot[i].setOoPV(true);
                        objectsBehindRobot[i].setASRNo(currentViewInMFIS[j].getASRNo());

                        //objectsBehindRobot[i].setLimitingPoint(viewNumber);
                        for (unsigned int le = 0; le < currentViewInMFIS[j].getLocalEnvID().size(); le++)
                            if (isThisIDAdded(currentViewInMFIS[j].getLocalEnvID()[le], objectsBehindRobot[i].getLocalEnvID()) == false)
                                objectsBehindRobot[i].setLocalEnvID(currentViewInMFIS[j].getLocalEnvID()[le]);

                        objectsBehindRobot[i].setPEP1(currentViewInMFIS[j].getPEP1());
                        objectsBehindRobot[i].setPEP2(currentViewInMFIS[j].getPEP2());

                        insertTo = currentViewInMFIS[j].getID();

                        //checking whether this one is a target Object
                        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
                            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                                targetObjectsForNextStep.back().setID(objectsBehindRobot[i].getID());
                                break;
                            }
                        }

                        break;
                    }
                }
                cout << "Hello" << endl;
                break;
            }
        }
    }

    //wiping based on limiting point
    vector<Object> tempAfterWiping;
    //  if (viewNumber >= 143) {
    cout << "Wiping @" << viewNumber << " based on limiting point" << endl;
    displayObjects(objectsInSideCV);
    vector<int> needToDeleteLPs;
    vector<Object> objectsProvidingLPToDelete;
    
    //Algorithm 0: Spatial boundary. It doesn't need to check time stamp
    //Algorithm 1: Spatial and Temp. 
    //Algorithm 2: 0 to tx
    
    
    
    vector<double> boundariesOfCV = findBoundariesOfCV(cView, 500.0);

    //for printing only
    vector<Object> boundaryLines;
    boundaryLines.push_back(Object(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2]));
    boundaryLines.push_back(Object(boundariesOfCV[1], boundariesOfCV[2], boundariesOfCV[1], 0));
    boundaryLines.push_back(Object(boundariesOfCV[1], 0, boundariesOfCV[0], 0));
    
    vector<Surface> rectangle = convertObjectToSurface(boundaryLines);
    
    //for demonastration only
    vector<Object> cvBoundaryOnMFIS; //for demonstration of wiping out algorithm
    
    cout<<"Algorithm: " << DELETING_ALGORITHM << "is going to execute. " << endl;
    //finding timestamp/local space identity
    if (DELETING_ALGORITHM == 1) {
        //waitHere();
        for (unsigned int i = 0; i < objectsInSideCV.size(); i++) {
            //if (objectsInSideCV[i].getLimitingPoint() != lastLimitingPoint) {
                needToDeleteLPs.push_back(objectsInSideCV[i].getLimitingPoint());
                objectsProvidingLPToDelete.push_back(objectsInSideCV[i]);
           // }
        }
    } else if (DELETING_ALGORITHM == 2) {
        for (unsigned int i = 0; i < objectsInSideCV.size(); i++) {
            if (objectsInSideCV[i].getLimitingPoint() != lastLimitingPoint) {
                needToDeleteLPs.push_back(objectsInSideCV[i].getLimitingPoint());
                objectsProvidingLPToDelete.push_back(objectsInSideCV[i]);
            }
        }
    }
    std::sort(needToDeleteLPs.begin(), needToDeleteLPs.end(), sortIntAscendingOrder);
    for (unsigned int i = 0; i < needToDeleteLPs.size(); i++) {
        cout << " " << needToDeleteLPs[i];
    }

    vector<Object> deleteByTimeStamp;
    if (DELETING_ALGORITHM == 0) {
        for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {

            if (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0
                    && (pointInPolygon(PointXY(objectsBehindRobotOnCV[i].X1(), objectsBehindRobotOnCV[i].Y1()), rectangle) == true ||
                    pointInPolygon(PointXY(objectsBehindRobotOnCV[i].X2(), objectsBehindRobotOnCV[i].Y2()), rectangle) == true)) {
                deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                //cout<<endl;objectsBehindRobotOnCV[i].display();
            } else {

                tempAfterWiping.push_back(objectsBehindRobot[i]);
            }
        }
        //for demonastration only
        for (unsigned int i = 0; i < boundaryLines.size(); i++) {
            temp = remakeLineP2(refobjects[0], refobjects[1], boundaryLines[i], i + 1, 0, refobjects[0].getKP());
            cvBoundaryOnMFIS.push_back(temp);
        }
        polygonObjectsOnMFIS = cvBoundaryOnMFIS;//for demonastration only
    }

    
    if (needToDeleteLPs.size() > 0) {
        int thisObjectsLP;

        int deleteFrom1;
        int deleteTo1;

        deleteFrom1 = needToDeleteLPs[0];
        deleteTo1 = needToDeleteLPs.back();

        if (DELETING_ALGORITHM == 1) {
            cout << "Forgetting algorithm 1" << endl;
            int deleteFrom2;
            int deleteTo2;
            //faking first.
            //if necessary then these values will be changed in next step.
            deleteFrom2 = needToDeleteLPs[0];
            deleteTo2 = needToDeleteLPs.back();

            int deleteFrom3;
            int deleteTo3;
            //faking first.
            //if necessary then these values will be changed in next step.
            deleteFrom3 = needToDeleteLPs[0];
            deleteTo3 = needToDeleteLPs.back();
            cout<<"Finding time range. Will split the range if necessary."<<endl;
            cout << endl << "Going to delete btw: " << deleteTo1 << " & " << deleteFrom1 << endl;
            for (unsigned int i = 0; i < needToDeleteLPs.size() - 1; i++) {
                if ((needToDeleteLPs[i + 1] - needToDeleteLPs[i]) > 20) {
                    deleteTo1 = needToDeleteLPs[i];
                    deleteFrom2 = needToDeleteLPs[i + 1];
                    deleteFrom3 = needToDeleteLPs[i + 1]; //upto now last two ranges are same. going to check for next split.
                    for (unsigned int j = i + 1; j < needToDeleteLPs.size() - 1; j++) {
                        if ((needToDeleteLPs[j + 1] - needToDeleteLPs[j]) > 20) {
                            deleteTo2 = needToDeleteLPs[j];
                            deleteFrom3 = needToDeleteLPs[j + 1];
                        }
                    }
                    cout << endl << "No" << endl;
                    cout << "Going to delete btw: " << deleteFrom1 << " & " << deleteTo1 << endl;
                    cout << "Going to delete btw: " << deleteFrom2 << " & " << deleteTo2 << endl;
                    cout << "Going to delete btw: " << deleteFrom3 << " & " << deleteTo3 << endl;
                    //waitHere();
                    break;
                }
            }
            for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
                thisObjectsLP = objectsBehindRobot[i].getLimitingPoint();
                if (thisObjectsLP >= deleteFrom1 && thisObjectsLP <= deleteTo1
                        && (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0)) {
                    cout << thisObjectsLP << " ";
                    deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                    //cout<<endl;objectsBehindRobotOnCV[i].display();
                } else if (thisObjectsLP >= deleteFrom2 && thisObjectsLP <= deleteTo2
                        && (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0)) {
                    cout << thisObjectsLP << " ";
                    deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                    //cout<<endl;objectsBehindRobotOnCV[i].display();
                } else if (thisObjectsLP >= deleteFrom3 && thisObjectsLP <= deleteTo3
                        && (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0)) {
                    cout << thisObjectsLP << " ";
                    deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                    //cout<<endl;objectsBehindRobotOnCV[i].display();
                } else {

                    tempAfterWiping.push_back(objectsBehindRobot[i]);
                }
            }
        } else if (DELETING_ALGORITHM == 2) {
            cout << "Forgetting Algorithm2" << endl;
            deleteFrom1 = 1;
            deleteTo1 = needToDeleteLPs.back();
            cout << "Going to delete btw: " << deleteFrom1 << " & " << deleteTo1 << endl;

            for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
                thisObjectsLP = objectsBehindRobot[i].getLimitingPoint();
                if (thisObjectsLP >= deleteFrom1 && thisObjectsLP <= deleteTo1
                        && (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0)) {
                    cout << thisObjectsLP << " ";
                    deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                    //cout<<endl;objectsBehindRobotOnCV[i].display();
                } else if (thisObjectsLP == lastLimitingPoint
                        && (objectsBehindRobotOnCV[i].Y1() > 0 && objectsBehindRobotOnCV[i].Y2() > 0)) {
                    cout << thisObjectsLP << " ";
                    deleteByTimeStamp.push_back(objectsBehindRobot[i]);
                    //cout<<endl;objectsBehindRobotOnCV[i].display();
                } else {

                    tempAfterWiping.push_back(objectsBehindRobot[i]);
                }
            }
        }
        //        if (viewNumber == 125)
        //            waitHere();
    }

 
    

    if (PRINT_UPDATING_STAGES == true) {
        sprintf(mfisFileName, "%s%d%s", "Maps/view-", viewNumber, ".png");
        
        if(DELETING_ALGORITHM == 0)
            plotObjectsOf3Kinds(mfisFileName,myrobot.getRobot(), boundaryLines,cView);
        else
            plotObjectsOf3Kinds(mfisFileName,myrobot.getRobot(), polygonObjects,cView);
        
        
        
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "a.png");
        plotObjects(mfisFileName, addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS), 
                            perceptualMap,limitingPoints);
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "b-cvBoundary.png");
        //        plotObjectsOf3Kinds(mfisFileName, perceptualMap, 
        //                addTwoVectorsOfObjects(objectsInSideCV,deleteByTimeStamp),addTwoVectorsOfObjects(polygonObjectsOnMFIS,addTwoVectorsOfObjects(cRobotPositionInMFIS,objectsProvidingLPToDelete)));
        plotObjectsOf3Kinds(mfisFileName, perceptualMap, polygonObjectsOnMFIS,
                addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS), limitingPoints);
        //    plotObjectsOf3Kinds(mfisFileName, objectsBehindRobot,polygonObjectsOnMFIS,
        //            addTwoVectorsOfObjects(addTwoVectorsOfObjects(cRobotPositionInMFIS,cvBoundaryOnMFIS),objectsProvidingLPToDelete));
        //    
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "c-SptlDlt.png");
        plotObjectsOf3Kinds(mfisFileName, objectsBehindRobot, polygonObjectsOnMFIS,
                addTwoVectorsOfObjects(deleteByTimeStamp, addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS)),
                limitingPoints);
    }
    if (tempAfterWiping.size() > 0) //means some surfaces has been deleted.
        objectsBehindRobot = tempAfterWiping;

    if (PRINT_UPDATING_STAGES == true) {
        objectsInSideCV.clear();
        deleteByTimeStamp.clear();
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "d-TmpDlt.png");
        plotObjectsOf3Kinds(mfisFileName, objectsBehindRobot, polygonObjectsOnMFIS,
                addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS), limitingPoints);
    }


    //inserting LEFT side's objects(old)
    for (int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == -1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }

    //inserting NEW objects
    cout << "InsertFrom " << insertFrom << endl;
    cout << "InsertTo: " << insertTo << endl;
    cout << "size: " << objectsBehindRobot.size() << endl;
    cout << "last id: " << cView[cView.size() - 1].getID() << endl;
    if (viewNumber == 87) {
        // insertTo = 14;
        //waitHere();
    }
    vector<Object> newObjectsFromCV; //for thesis only
    for (unsigned int j = insertFrom; j < insertTo - 1; j++) {
        updatedPM.push_back(currentViewInMFIS[j]);
        updatedPM.back().setID(lastObjectID + 1);
        //updatedPM.back().setASRNo(ASRNumber);
        newObjectsFromCV.push_back(currentViewInMFIS[j]);

        //making target(i.e changing id as they have in MFIS) object for next step
        for (unsigned int k = 0; k < targetObjectsInCV.size(); k++) {
            if (targetObjectsInCV[k].getID() == currentViewInMFIS[j].getID()) {
                targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                targetObjectsForNextStep.back().setID(lastObjectID + 1);
                break;
            }
        }

        lastObjectID++;
    }
    cout << endl << endl << "TargetObjects for Next step " << targetObjectsForNextStep.size() << endl;
    //displayObjects(targetObjectsForNextStep);

    //inserting RIGHT side's objects(old)
    for (unsigned int i = 0; i < objectsBehindRobot.size(); i++) {
        if (objectsBehindRobot[i].getPos() == 1) {
            updatedPM.push_back(objectsBehindRobot[i]);
        }
    }
    updatedPM.back().setID(lastObjectID + 1);
    cout << endl << endl << "UpdatedMFIS " << endl;
    //displayObjects(updatedPM);
    //     plotObjectsOf3Kinds("Maps/updatedMFIS.png", allRobotPositions,newObjectsFromCV,objectsBehindRobot);
    //     waitHere();

    if (PRINT_UPDATING_STAGES == true) {
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "e-UpdMap.png");
        plotObjectsOf3Kinds(mfisFileName, objectsBehindRobot, polygonObjectsOnMFIS,
                addTwoVectorsOfObjects(newObjectsFromCV, addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS)),
                limitingPoints);
        
        
        polygonObjectsOnMFIS.clear();
        sprintf(mfisFileName, "%s%d%s", "Maps/PM-", viewNumber, "f-fMap.png");
        plotObjectsOf3Kinds(mfisFileName, objectsBehindRobot, polygonObjectsOnMFIS,
                addTwoVectorsOfObjects(newObjectsFromCV, addTwoVectorsOfObjects(myrobot.getRobot(),cRobotPositionInMFIS)),
                limitingPoints);
    }

    Transporter package;
    package.setView(updatedPM);
    package.setTargetObjects(targetObjectsForNextStep);
    package.setASRs(places);
    
    //14-10-15 to send back CVonMFIS
    package.setReferenceObjectsForLoopClosing(currentViewInMFIS);
    
    return package;
}

void updateMapAtLastStep(Transporter lastStepInfo) {
    //project all target objects of last view onto MFIS to recognize
    vector<Object> targetObjectsInMFIS = projectingTheView(lastStepInfo.getTargetObjects(), lastStepInfo.getReferenceObjects()[0], lastStepInfo.getReferenceObjects()[1], 1);

    vector<Minfo> recognizedObject = recognizeObjects(targetObjectsInMFIS, lastStepInfo.getMFIS());
    cout << "Display Matched info" << endl;
    displayMinfo(recognizedObject);

    vector<Object> recogObjectInCV, recogObjectInMFIS;
    Object s;
    if (recognizedObject.size() > 5) {
        for (unsigned int ii = 0; ii < recognizedObject.size(); ii++) {
            s = getObject(targetObjectsInMFIS, recognizedObject[ii].getCID());
            recogObjectInCV.push_back(s);
            s = getObject(lastStepInfo.getMFIS(), recognizedObject[ii].getPID());
            recogObjectInMFIS.push_back(s);
        }
    }
    if (recogObjectInCV.size() > 5) {
        plotObjects("Maps/RecognizedObjectsInCV.png", recogObjectInCV, recogObjectInCV);
        plotObjects("Maps/RecognizedObjectsInMFIS.png", recogObjectInMFIS, recogObjectInMFIS);
    }

    plotObjects("Maps/MFISatlastStep.png", lastStepInfo.getMFIS(), targetObjectsInMFIS);
    //waitHere();
}

bool isBelongToSameSpace(int limitingPoint, vector<int> spaceIDs) {
    for (unsigned int i = 0; i < spaceIDs.size(); i++) {
        if (limitingPoint == spaceIDs[i]) {
            return true;
        }
    }
    return false;
}

bool isBelongToSameSpace(vector<int> limitingPoint, vector<int> spaceIDs) {
    for (unsigned int j = 0; j < limitingPoint.size(); j++) {
        for (unsigned int i = 0; i < spaceIDs.size(); i++) {
            if (limitingPoint[j] == spaceIDs[i]) {
                return true;
            }
        }
    }
    return false;
}

bool isThisIDAdded(int newID, vector<int> previousIDs) {
    for (unsigned int i = 0; i < previousIDs.size(); i++) {
        if (newID == previousIDs[i]) {
            return true;
        }
    }
    return false;
}

//return true if thisObject close to current view

bool isThisCloseToCurrentView(Object thisObject, vector<Object> currentView) {
    for (unsigned int i = 0; i < currentView.size(); i++) {
        if (thisObject.shortestDistanceWithObject(currentView[i]) < 2000)
            return true;
    }

    return false;
}

bool isThisCloseToCVPolygonBoundary(Object thisObject, vector<Object> polygonObjects, double distTh) {
    double distance;
    bool p1IsClose = false, p2IsClose = false;
    bool bigObject = false;
    for (unsigned int i = 0; i < polygonObjects.size(); i++) {
        distance = polygonObjects[i].shortestDistFrom(thisObject.X1(), thisObject.Y1());
//        if(thisObject.getID() == 6980)
//        cout<<"p1Dist: "<<distance<<endl;
        if (distance < distTh) {
            p1IsClose = true;
        }
        distance = polygonObjects[i].shortestDistFrom(thisObject.X2(), thisObject.Y2());
//        if(thisObject.getID() == 6980)
//        cout<<"p2Dist: "<<distance<<endl;
        if (distance < distTh) {
            p2IsClose = true;
        }
        
    }
    if (checkForIntersection(Object(0, 0, 0, 3000), thisObject) == 1) {//to find big surf which maybe across cv
        bigObject = true;//actually this isn't necessary now. better heuristic is added in another condition.
    }

    if (p1IsClose == true && p2IsClose == true)
        return true;
    else if (bigObject == true)//for big across cv type surf
        return true;
    else
        return false;
}

bool findCrossedExit(vector<Object> & allCrossedExit, vector<Object> lastView,
        vector<Object> refObjects, vector<double> distanceAngle, int set) {
    cout<<"Finding crossed exit"<<endl;
    bool exitCrossed = false;
    vector<Exit> exitsInPV = findShortestExits(lastView);
    vector<Object> exitsAsObjects = convertExitToObject(exitsInPV);
    //exitsAsObjects = findExitsThesisVersion(lastView);

    Object temp;
    if (set == 501 && lastView.back().getVN() == 164) {//for set 501
        temp.set(lastView[lastView.size() - 5].X1(), lastView[lastView.size() - 5].Y1(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 501 && lastView.back().getVN() == 198) {//for set 501
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }

    if (set == 500 && lastView.back().getVN() == 852) {//for exit6 of set 500 
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 500 && lastView.back().getVN() == 1027) {//for exit8 or kitchen entrance of set 500
        temp.set(lastView[3].X2(), lastView[3].Y2(),
                lastView[lastView.size() - 1].X2(), lastView[lastView.size() - 1].Y2(), 1);
        exitsAsObjects.push_back(temp);
    }
    if (set == 500 && lastView.back().getVN() == 1224) {//for exit10 or return home of set 500 
        temp.set(lastView[0].X2(), lastView[0].Y2(),
                lastView[lastView.size() - 1].X1(), lastView[lastView.size() - 1].Y1(), 1);
        exitsAsObjects.push_back(temp);
    }

    Point cRobotPositionInPV; //current robot position in previous view
    double angle = (distanceAngle[1] / 180) * PI; //angle in radian
    double rpx = (distanceAngle[0] + 400) * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double rpy = (distanceAngle[0] + 400) * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
    cRobotPositionInPV.set(rpx, rpy);
    cout << endl << "v" << 1 + 1 << " dis(v12) " << distanceAngle[0] << " angle(v12) " << distanceAngle[1] << endl;
    cout << "robot position id-";
    //cRobotPositionInPV.display();


    Object robotPath;
    robotPath.set(0.0, 0.0, rpx, rpy, 1);

    vector<Object> dummy;
    dummy.push_back(robotPath);
    cout<<endl<<"Robot path found!!"<<endl;
    cout<<endl<<"No of probable exits: "<<exitsAsObjects.size()<<endl;
    //checking for intersection of last view exits n robotPath
    for (unsigned int i = 0; i < exitsAsObjects.size(); i++) {
        
        cout << "exit length: " << exitsAsObjects[i].length() << endl;
        if (checkForIntersection(exitsAsObjects[i], robotPath) == 1 &&
                exitsAsObjects[i].length() > 600 && exitsAsObjects[i].length() < 1200) {
            dummy.push_back(exitsAsObjects[i]);
            cout << "I just crossed an Exit " << i << endl;
            temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[i], i + 1, 0, refObjects[0].getKP());
            if (allCrossedExit.back().distMPToPoint(temp.mpX(), temp.mpY()) > 2500) {
                allCrossedExit.push_back(temp);
                exitCrossed = true;
            }
        }
    }
    if(exitCrossed == false)
        cout<<"Did not cross any exit !!"<<endl;

    if (set == 501 && exitCrossed == false && lastView.back().getVN() == 133) {//for set 501
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    if (set == 500 && exitCrossed == false && lastView.back().getVN() == 121) {//for exit1 of set 500
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    if (set == 500 && exitCrossed == false && lastView.back().getVN() == 934) {//for exit7 or exit1repeat of set 500
        temp = remakeLineP2(refObjects[0], refObjects[1], exitsAsObjects[0], 1, 0, refObjects[0].getKP());
        allCrossedExit.push_back(temp);
        exitCrossed = true;
    }

    char mfisFileName[100];
    sprintf(mfisFileName, "%s%d%s", "Maps/exitsCrossed-", lastView.back().getVN(), ".png");
    //plotObjectsOf3Kinds(mfisFileName,dummy,exitsAsObjects,lastView);
    if(exitCrossed == false)
        cout<<"Did not cross any exit !!"<<endl;
    return exitCrossed;
}

//void updatePerceptualMapUsingPlaceInformation(vector<ASR> & places, vector<Object> & MFIS,
//        vector<Object> cView, vector<Object> currentRobotPositionInMFIS, vector <Object> allRobotPositions,
//        vector<Object> refobjects, int viewNumber, int ASRNumber, bool exitCrossed,
//        vector<Object> crossedExit, vector<Object> & targetObjectsInPV, vector<Object> refObjectForLoopClosing,
//        int updatingASRNumber, vector<Object> routeMap4CPlace) {
//
//    plotPerceptualMap("Maps/PerceptualMapBeforeUpdating.png", currentRobotPositionInMFIS, MFIS, ASRNumber);
//
//    vector<Object> oldPlaceOnNew;
//    Object temp;
//    //projecting old place on Current place
//    for (unsigned int i = 0; i < MFIS.size(); i++) {
//        if (MFIS[i].getASRNo() == updatingASRNumber) {
//            temp = remakeLineP2(refObjectForLoopClosing[1], refObjectForLoopClosing[0], MFIS[i], MFIS[i].getID(), 0, 1);
//            //oldPlaceOnNew.push_back(temp);
//            MFIS[i].set(temp);
//
//        }
//    }
//    //oldPlaceOnNew = projectingTheView(oldPlaceOnNew,refObjectForLoopClosing[1], refObjectForLoopClosing[0], 1);
//    plotPerceptualMap("Maps/PerceptualMapAfterUpdating.png", currentRobotPositionInMFIS, MFIS, ASRNumber);
//    //plotPerceptualMap("Maps/PerceptualMapAfterUpdating.png", oldPlaceOnNew, MFIS, ASRNumber);
//    exitCrossed = false;
//    Transporter computedOutput;
//    computedOutput = updatePerceptualMapATPlace(places, MFIS, cView, currentRobotPositionInMFIS, allRobotPositions, refobjects,
//            viewNumber, ASRNumber, exitCrossed, crossedExit, routeMap4CPlace);
//    MFIS = computedOutput.getView();
//    places = computedOutput.getASRs();
//    targetObjectsInPV = computedOutput.getTargetObjects();
//    plotObjects("Maps/finalUpdatedMFIS.png", currentRobotPositionInMFIS, MFIS);
//    //waitHere();
//}

void findBestReferenceObjectsUsingOdometryInfo(vector<Object> & referenceObjects,
        vector< vector<Object> > allRPoseAtOneStep, vector<Object> currentRobotPositionInMFIS,
        vector<double> coordTransInfo) {
    MyRobot myrobot(0, 0);
    double distErrorTh = 100.0;
    double angleErrorTh = 1.0;
    double angleError = 0, distanceError = 0;
    vector<Object> odometricReferenceObject, odometricCRPositionInMFIS;
    //localization using odometer
    Object aLine = makeLineAtTwoPointsWithObject(coordTransInfo[1], coordTransInfo[0], coordTransInfo[1], coordTransInfo[0] + 500, currentRobotPositionInMFIS[6], 1);
    aLine.setKP(1);
    odometricReferenceObject.clear();
    odometricReferenceObject.push_back(aLine);
    odometricReferenceObject.push_back(Object(0, 0, 0, 500, 1));
    odometricCRPositionInMFIS = myrobot.inMFIS(odometricReferenceObject[0],
            odometricReferenceObject[1], odometricReferenceObject[0].getKP());

    //by default use odo
    referenceObjects = odometricReferenceObject;


    //this condition true means there are some recognized objects. so find best
    if (allRPoseAtOneStep.size() > 0) {
        distanceError = odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[0][6]);
        angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[0][6]));
        if (distanceError < distErrorTh && angleError < angleErrorTh) {//means first one good enough
            referenceObjects.clear();
            referenceObjects.push_back(allRPoseAtOneStep[0][9]); //ref from mfis
            referenceObjects.push_back(allRPoseAtOneStep[0][10]); //ref from cv
        }

        for (unsigned int i = 1; i < allRPoseAtOneStep.size(); i++) {
            distanceError = odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[i][6]);
            angleError = abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[i][6]));
            if (angleError > 350.0)
                angleError = 360.0 - angleError;
            //            if (angleError < angleErrorTh && distanceError < distErrorTh &&
            //                    angleError < abs(odometricCRPositionInMFIS[6].getAngleWithLine(allRPoseAtOneStep[i - 1][6]))) {
            if (angleError < angleErrorTh && distanceError < distErrorTh &&
                    distanceError < odometricCRPositionInMFIS[6].distP1ToP1(allRPoseAtOneStep[i - 1][6])) {
                referenceObjects.clear();
                referenceObjects.push_back(allRPoseAtOneStep[i][9]); //ref from mfis
                referenceObjects.push_back(allRPoseAtOneStep[i][10]); //ref from cv

            }
        }
    }

}

void findRouteMap(vector<Object> MFIS,vector<vector<Object> > allRobotPose) {
    
    vector<Object> landmarkSurfaces;
    vector<vector<Object> > robotPoseBesideLandmark;
    for(unsigned int i=0; i < MFIS.size(); i++) {
        if(MFIS[i].length() > 3000) {
            landmarkSurfaces.push_back(MFIS[i]);
            
        }
    }
    plotObjects("Maps/landmarkSurfaces.png",landmarkSurfaces);
}

void updateRouteMap(vector<Object> & entireRouteMap,Object cRobotPosition,vector<Object> MFIS) {
    Object probablePath;
    probablePath.set(entireRouteMap.back().X1(),entireRouteMap.back().Y1(),cRobotPosition.X1(),cRobotPosition.Y1(),1);
    
    for (unsigned int i = 0; i < MFIS.size(); i++) {
        if (checkForIntersection(probablePath, MFIS[i]) == 1) {//true means need to push new pathSegment.
            probablePath.set(entireRouteMap.back().X2(),entireRouteMap.back().Y2(),cRobotPosition.X1(),cRobotPosition.Y1(),1);
            entireRouteMap.push_back(probablePath);
            break;
        }
    }
    
    //comes here. that means cRobotPosition is still visible.
    //so, resent lastPoint.
    entireRouteMap.back().setP2(cRobotPosition.X1(),cRobotPosition.Y1());
}