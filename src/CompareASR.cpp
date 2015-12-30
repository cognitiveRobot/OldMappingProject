#include <iostream>
#include <vector>
#include <algorithm>

#include "asr.H"
#include "Object.H"

#include "CompareASR.H"
#include "Plotting.H"
#include "GeometricOp.H"
#include "CompareASRClass.H"
#include "mfisOp.H"

using namespace std;
#define PI 3.14159265

//vector<Object> findSameObjectsFromTwoASRs(ASRNetwork pm, ASR oldASR, ASR newASR, Object rPositionOfNewASR){
Transporter findSameObjectsFromTwoASRs(ASRNetwork pm, ASR oldASR, ASR newASR, Object rPositionOfNewASR, Point currentRobotPosition){
    cout<<"\033[1;31m-------------------Inside findSameObjectsFromTwoASRs Module---------------------\033[0m"<<endl;
    Object rPositionOfOldASR;
    rPositionOfOldASR.set(0,0,0,400,1);
    double distMargin,angleMargin;
    double errorDist,errorAngle;
    
    distMargin = 2*(rPositionOfOldASR.distP1ToP1(rPositionOfNewASR));
    angleMargin = 2*abs(rPositionOfOldASR.getAngleWithLine(rPositionOfNewASR));
    //this is because getAngleWithLine(will be used later to compare angleMargin 
    //gives angle from -180 to +180
    if(angleMargin > 180) { 
        angleMargin = 360 - angleMargin;
    }
    cout<<"dist margin "<<distMargin<<" angleMargin "<<angleMargin<<endl;
    
    vector<Object> oldASRObjects=oldASR.getASRObjects();
    vector<Object> newASRObjects = newASR.getASRObjects();
    //vector<Object> newASRObjects;
    
    plotObjects("MFIS/ASR1-7.png",oldASRObjects,newASRObjects);
   
    double angle1,dist1,angle2,dist2;
    Object tmpRPosition,possibleRPosition;
    PossibleRobotPosition possibleRobotPosition;
    vector<PossibleRobotPosition> allPossibleRobotPositions;
    
    //getting all possible robot position which are within angle and distance margin
    for(int i=0;i<int(newASRObjects.size());i++) {
       // if(newASRObjects[i].getP1OS() == 1 || newASRObjects[i].getP2OS() == 1) {
            //if(newASRObjects[i].getP1OS() == 1)
                tmpRPosition.set(newASRObjects[i].X1(),newASRObjects[i].Y1(),rPositionOfNewASR.X1(),rPositionOfNewASR.Y1(),1);
           // else
              //  tmpRPosition.set(newASRObjects[i].X2(),newASRObjects[i].Y2(),rPositionOfNewASR.X1(),rPositionOfNewASR.Y1(),1);
            angle1=newASRObjects[i].getAngleWithLine(tmpRPosition);
            dist1=newASRObjects[i].distP1ToP1(rPositionOfNewASR);
            //for p2
            tmpRPosition.set(newASRObjects[i].X1(),newASRObjects[i].Y1(),rPositionOfNewASR.X2(),rPositionOfNewASR.Y2(),1);
            angle2=newASRObjects[i].getAngleWithLine(tmpRPosition);
            dist2=newASRObjects[i].distP1ToP2(rPositionOfNewASR);
            //cout<<"Angle "<<angle<<" dist "<<dist<<endl;
            for(int j=0;j<int(oldASRObjects.size());j++) {
              //  if(oldASRObjects[j].getP1OS() == 1 || oldASRObjects[j].getP2OS() == 1) {
                   // if(oldASRObjects[j].getP1OS() == 1)
                        possibleRPosition = makeLineAtTwoPointsWithObject(angle1,dist1,angle2,dist2,oldASRObjects[j],1);
                  //  else
                   //     possibleRPosition = makeLineAtTwoPointsWithObject(angle1,dist1,angle2,dist2,oldASRObjects[j],2);
                    possibleRobotPosition.setPossibleRobotPosition(possibleRPosition);
                    possibleRobotPosition.setObjectOfNewASR(newASRObjects[i]);
                    possibleRobotPosition.setObjectOfOldASR(oldASRObjects[j]);
                    
                    errorDist = rPositionOfOldASR.distP1ToP1(possibleRobotPosition.getPossibleRobotPosition());
                    errorAngle = abs(rPositionOfOldASR.getAngleWithLine(possibleRobotPosition.getPossibleRobotPosition()));
                    if(errorDist < distMargin && errorAngle < angleMargin)
                        allPossibleRobotPositions.push_back(possibleRobotPosition);
               // }
            }
        //}
    }
   
    //getting neighbor all possible robot position of each one
    vector<vector<PossibleRobotPosition> > neighbors;
    vector<PossibleRobotPosition> neighbor4one;
    double distanceError,angleError;
    
    vector<PossibleRobotPosition> mostPossibleRobotPositions;
    
    for(int i=0;i<int(allPossibleRobotPositions.size());i++) {
        neighbor4one.push_back(allPossibleRobotPositions[i]);
        for(int j = 0;j<int(allPossibleRobotPositions.size());j++) {
            if(j != i) {
                distanceError = allPossibleRobotPositions[i].getSqDistanceFromP1ToP1(allPossibleRobotPositions[j]);
                if(distanceError < 10000) {
                    angleError = allPossibleRobotPositions[i].getAngleWith(allPossibleRobotPositions[j]);
                    if(abs(angleError) < 5) {
                        neighbor4one.push_back(allPossibleRobotPositions[j]);
                    }
                }
            }
        }
        //getting most possible robot position
        if(i == 0) {
                neighbors.push_back(neighbor4one);
                mostPossibleRobotPositions = neighbor4one;//first group by default
        }else {
            if(mostPossibleRobotPositions.size() < neighbor4one.size()) { //is this group larger?
                mostPossibleRobotPositions =neighbor4one;//so this group is better
            }
            neighbors.push_back(neighbor4one);
        }
        neighbor4one.clear();
    }
    
    //getting recognized objects
    vector<Object> sameObjectsFromOldASR,sameObjectsFromNewASR;
    for(int i = 0;i<int(mostPossibleRobotPositions.size());i++) {
        sameObjectsFromNewASR.push_back(mostPossibleRobotPositions[i].getObjectOfNewASR());
        sameObjectsFromOldASR.push_back(mostPossibleRobotPositions[i].getObjectOfOldASR());
    }
   cout<<"all possible Robot position "<<allPossibleRobotPositions.size()<<endl;
     
   if(sameObjectsFromNewASR.size() > 0) 
    plotObjects("MFIS/recognizedObjects.png", sameObjectsFromNewASR, sameObjectsFromOldASR);

   Transporter t;
    return t;
    //reorientation all the ASRs according to old position
    vector<ASR> allASRs = pm.getASRs();
   //plotPerceptualMapWithASRs("MFIS/newPerceptualMap.png", allASRs);
    
    //replace first ASR by new ASR and locate old Exit from old exit into new ASR
    vector<Object> referenceObjectsForAdjustment;
    referenceObjectsForAdjustment.push_back(sameObjectsFromOldASR[0]);
    referenceObjectsForAdjustment.push_back(sameObjectsFromNewASR[0]);
   // Transporter adjustedASRs=mapOldASRsIntoNewASRsAfterLoopClosing(referenceObjectsForAdjustment,allASRs);
    Transporter newPM=makeNewPerceptualMap(referenceObjectsForAdjustment,allASRs);
    Object oldExitInNewASR;
    vector<Object> tmp;
   // plotPerceptualMapWithASRs("MFIS/newPerceptualMap.png", allASRs);
    for (int j = 4; j<int(allASRs.size()); j++) {

        for (int i = 0; i<int(allASRs[j].getASRObjects().size()); i++) {
            tmp.push_back(remakeLineP2(sameObjectsFromOldASR[0], sameObjectsFromNewASR[0], allASRs[j].getASRObjects()[i], 1, 0, sameObjectsFromOldASR[0].getKP()));
        }
        allASRs[j].setASRObjects(tmp);
        oldExitInNewASR = remakeLineP2(sameObjectsFromOldASR[0], sameObjectsFromNewASR[0], allASRs[j].getASRExit1(), 1, 0, sameObjectsFromOldASR[0].getKP());
        allASRs[j].setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(sameObjectsFromOldASR[0], sameObjectsFromNewASR[0], allASRs[j].getASRExit2(), 1, 0, sameObjectsFromOldASR[0].getKP());
        allASRs[j].setASRExit2(oldExitInNewASR);

        tmp.clear();
        for (int i = 0; i<int(allASRs[j].getRoute().size()); i++) {
            tmp.push_back(remakeLineP2(sameObjectsFromOldASR[0], sameObjectsFromNewASR[0], allASRs[j].getRoute()[i], 1, 0, sameObjectsFromOldASR[0].getKP()));
        }
        allASRs[j].replaceTheWholeRoute(tmp);
        tmp.clear();
    }
    //plotPerceptualMapWithASRs("MFIS/newPerceptualMap.png", allASRs);
    
    //merging old and new ASRs
    oldASRObjects=oldASR.getASRObjects();
    newASRObjects = newASR.getASRObjects();
    //plotObjects("MFIS/oldNnewASRs.png", oldASRObjects, newASRObjects); //possibleRobotPosition.convertAllPossibleRobotPositions(mostPossibleRobotPositions));
    //while (true) {
        int reflineNum;
        cout << "specify the ref line ";
        //cin >> reflineNum;
        reflineNum =3;
        tmp.clear();
        for (int i = 0; i<int(oldASRObjects.size()); i++) {
            tmp.push_back(remakeLineP2(sameObjectsFromNewASR[reflineNum], sameObjectsFromOldASR[reflineNum], oldASRObjects[i], 1, 0, sameObjectsFromOldASR[reflineNum].getKP()));
        }
       // plotObjects("MFIS/oldASR-in-newASRs.png", tmp, newASRObjects);
  //  }
        Transporter comparedOutput = compareObjects(newASRObjects,tmp);
    Transporter package;
    package.setASRs(allASRs);
    //vector<Object> referenceObjectsForAdjustment;
    //referenceObjectsForAdjustment.push_back(sameObjectsFromOldASR[0]);
    //referenceObjectsForAdjustment.push_back(sameObjectsFromNewASR[0]);
    /*Object tmpRefObject;
    tmpRefObject.set(0,0,0,400,1);
    tmpRefObject.setKP(1);
    referenceObjectsForAdjustment.push_back(tmpRefObject);
    tmpRefObject.set(currentRobotPosition.X(),currentRobotPosition.Y(),currentRobotPosition.X(),currentRobotPosition.Y()+400,1);
    referenceObjectsForAdjustment.push_back(tmpRefObject);*/
    newPM.setReferenceObjects(referenceObjectsForAdjustment);
    
    return newPM;
}

vector<Object> findSameObjectsFromTwoViews(vector<Object> oldView, vector<Object> newView, double dist, double angle){
    cout<<"\033[1;31m-------------------Inside findSameObjectsFromTwoASRs Module---------------------\033[0m"<<endl;
    Object rPositionOfOldASR;
    rPositionOfOldASR.set(0,0,0,400,1);
    Object rPositionOfNewASR;
    rPositionOfNewASR.set(0,dist,0,dist+400,2);
    double distMargin,angleMargin;
    double errorDist,errorAngle;
    
    distMargin = 2*(rPositionOfOldASR.distP1ToP1(rPositionOfNewASR));
    angleMargin = 2*abs(rPositionOfOldASR.getAngleWithLine(rPositionOfNewASR));
    //this is because getAngleWithLine(will be used later to compare angleMargin 
    //gives angle from -180 to +180
    if(angleMargin > 180) { 
        angleMargin = 360 - angleMargin;
    }
    cout<<"dist margin "<<distMargin<<" angleMargin "<<angleMargin<<endl;
    
  angle = (angle / 180) * 3.14; //angle in radian
        double rpx = dist * sin(-angle);
        double rpy = dist * cos(-angle);
        Point robpos;
        robpos.set(rpx, rpy);
        
    
    vector<Object> oldASRObjects=oldView;
    vector<Object> newASRObjects = xformCVIntoPV(newView,robpos,angle);
    //vector<Object> newASRObjects;
    plotObjects("MFIS/view1-view2.png",oldView,newASRObjects);
   
    double angle1,dist1,angle2,dist2;
    Object tmpRPosition,possibleRPosition;
    PossibleRobotPosition possibleRobotPosition;
    vector<PossibleRobotPosition> allPossibleRobotPositions;
    
    //getting all possible robot position which are within angle and distance margin
    for(int i=0;i<int(newASRObjects.size());i++) {
       // if(newASRObjects[i].getP1OS() == 1 || newASRObjects[i].getP2OS() == 1) {
            tmpRPosition.set(newASRObjects[i].X1(),newASRObjects[i].Y1(),rPositionOfNewASR.X1(),rPositionOfNewASR.Y1(),1);
            angle1=newASRObjects[i].getAngleWithLine(tmpRPosition);
            dist1=newASRObjects[i].distP1ToP1(rPositionOfNewASR);
            //for p2
            tmpRPosition.set(newASRObjects[i].X1(),newASRObjects[i].Y1(),rPositionOfNewASR.X2(),rPositionOfNewASR.Y2(),1);
            angle2=newASRObjects[i].getAngleWithLine(tmpRPosition);
            dist2=newASRObjects[i].distP1ToP2(rPositionOfNewASR);
            //cout<<"Angle "<<angle<<" dist "<<dist<<endl;
            for(int j=0;j<int(oldASRObjects.size());j++) {
                //if(oldASRObjects[j].getP1OS() == 1 || oldASRObjects[j].getP2OS() == 1) {
                    possibleRPosition = makeLineAtTwoPointsWithObject(angle1,dist1,angle2,dist2,oldASRObjects[j],1);
                    possibleRobotPosition.setPossibleRobotPosition(possibleRPosition);
                    possibleRobotPosition.setObjectOfNewASR(newASRObjects[i]);
                    possibleRobotPosition.setObjectOfOldASR(oldASRObjects[j]);
                    
                    //errorDist = rPositionOfOldASR.distP1ToP1(possibleRobotPosition.getPossibleRobotPosition());
                    //errorAngle = abs(rPositionOfOldASR.getAngleWithLine(possibleRobotPosition.getPossibleRobotPosition()));
                    //if(errorDist < distMargin && errorAngle < angleMargin)
                        allPossibleRobotPositions.push_back(possibleRobotPosition);
               // }
            }
        //}
    }
   
    //getting neighbor all possible robot position of each one
    vector<vector<PossibleRobotPosition> > neighbors;
    vector<PossibleRobotPosition> neighbor4one;
    double distanceError,angleError;
    
    vector<PossibleRobotPosition> mostPossibleRobotPositions;
    
    for(int i=0;i<int(allPossibleRobotPositions.size());i++) {
        neighbor4one.push_back(allPossibleRobotPositions[i]);
        for(int j = 0;j<int(allPossibleRobotPositions.size());j++) {
            if(j != i) {
                distanceError = allPossibleRobotPositions[i].getSqDistanceFromP1ToP1(allPossibleRobotPositions[j]);
                if(distanceError < 10000) {
                    angleError = allPossibleRobotPositions[i].getAngleWith(allPossibleRobotPositions[j]);
                    if(abs(angleError) < 5) {
                        neighbor4one.push_back(allPossibleRobotPositions[j]);
                    }
                }
            }
        }
        //getting most possible robot position
        if(i == 0) {
                neighbors.push_back(neighbor4one);
                mostPossibleRobotPositions = neighbor4one;//first group by default
        }else {
            if(mostPossibleRobotPositions.size() < neighbor4one.size()) { //is this group larger?
                mostPossibleRobotPositions =neighbor4one;//so this group is better
            }
            neighbors.push_back(neighbor4one);
        }
        neighbor4one.clear();
    }
    
    //getting recognized objects
    vector<Object> sameObjectsFromOldASR,sameObjectsFromNewASR;
    for(int i = 0;i<int(mostPossibleRobotPositions.size());i++) {
        sameObjectsFromNewASR.push_back(mostPossibleRobotPositions[i].getObjectOfNewASR());
        sameObjectsFromOldASR.push_back(mostPossibleRobotPositions[i].getObjectOfOldASR());
    }
   cout<<"all possible Robot position "<<allPossibleRobotPositions.size()<<endl;
     
    plotObjects("MFIS/oldNnewASRs.png",oldASRObjects,possibleRobotPosition.convertAllPossibleRobotPositions(mostPossibleRobotPositions));
    plotObjects("MFIS/recognizedObjects.png",sameObjectsFromNewASR,sameObjectsFromOldASR);
     
    vector<Object> package;
    return package;
}

//first delete small objects amongst old Objects
//check angle difference with new objects if less than angle threshold then go to next step else push in the unknown objects
//if the shortest dist btw these two objects is less than threshold then they are same objects

Transporter compareObjects(vector<Object> newObjects, vector<Object> oldObjects) {
    newObjects = deleteSmallObjects(newObjects);
    oldObjects = deleteSmallObjects(oldObjects);
    vector<Object> sameObjects,unknownObjects; 
    double angleDiff,shortestDist;//p1Dist,p2Dist,
    bool thisObjectUnknown;
    for(int i=0;i<int(oldObjects.size());i++) {
        thisObjectUnknown = true;
        for(int j=0;j<int(newObjects.size());j++) {
            angleDiff = abs(oldObjects[i].getAngleWithLine(newObjects[j]));
            if(angleDiff > 350)
                angleDiff = 360 - angleDiff;
            if(angleDiff < 6) {
                //p1Dist = oldObjects[i].distP1ToP1(newObjects[j]);
                //p2Dist = oldObjects[i].distP1ToP1(newObjects[j]);
                shortestDist = shortestDistanceBtwTwoObjects(oldObjects[i],newObjects[j]);
                if(shortestDist < 400) {
                    thisObjectUnknown = false;
                    sameObjects.push_back(oldObjects[i]);
                    sameObjects.push_back(newObjects[j]);
                    break;
                }
            }
        }
        if(thisObjectUnknown == true) {
            unknownObjects.push_back(oldObjects[i]);            
        }
            
    }
    
   // plotObjectsOf3Kinds("MFIS/knowAndUknownObjects.png",newObjects,unknownObjects,sameObjects);
    cout<<"Num of Unknown Objects "<<unknownObjects.size()<<endl;
    cout<<"Num of Same Objects "<<sameObjects.size()<<endl;
    
    Transporter package;
    return package;
}

//whose length is less than threshold is considered as small objects
vector<Object> deleteSmallObjects(vector<Object> objects) {
    vector<Object> output;
    for(int i =0;i<int(objects.size());i++) {
        if(objects[i].length() > 400)
            output.push_back(objects[i]);
    }
    return output;
}

//calculate shortest dist from four end points
//find and return the shortest dist
double shortestDistanceBtwTwoObjects(Object old, Object new1) {
    vector<double> dists;
    dists.push_back(new1.shortestDistFrom(old.X1(),old.Y1()));
    dists.push_back(new1.shortestDistFrom(old.X2(),old.Y2()));
    dists.push_back(old.shortestDistFrom(new1.X1(),new1.Y1()));
    dists.push_back(old.shortestDistFrom(new1.X2(),new1.Y2()));
    double shortestDistance;
    shortestDistance = dists[0];
    for(int i =1;i<int(dists.size());i++) {
        if(shortestDistance > dists[i])
            shortestDistance = dists[i];
    }
    return shortestDistance;
}

Transporter adjustNewASRsAfterLoopClosing(vector<Object> refObjects,vector<ASR> allASRs) {
    vector<Object> tmp;
    Object oldExitInNewASR;
    for (int j = 4; j<int(allASRs.size()); j++) {

        for (int i = 0; i<int(allASRs[j].getASRObjects().size()); i++) {
            tmp.push_back(remakeLineP2(refObjects[0], refObjects[1], allASRs[j].getASRObjects()[i], 1, 0, refObjects[0].getKP()));
        }
        allASRs[j].setASRObjects(tmp);
        oldExitInNewASR = remakeLineP2(refObjects[0], refObjects[1], allASRs[j].getASRExit1(), 1, 0, refObjects[0].getKP());
        allASRs[j].setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(refObjects[0], refObjects[1], allASRs[j].getASRExit2(), 1, 0, refObjects[0].getKP());
        allASRs[j].setASRExit2(oldExitInNewASR);

        tmp.clear();
        for (int i = 0; i<int(allASRs[j].getRoute().size()); i++) {
            tmp.push_back(remakeLineP2(refObjects[0], refObjects[1], allASRs[j].getRoute()[i], 1, 0, refObjects[0].getKP()));
        }
        allASRs[j].replaceTheWholeRoute(tmp);
        tmp.clear();
    }
    plotPerceptualMapWithASRs("MFIS/finalPerceptualMap.png",allASRs);
    Transporter package;
    return package;
}

Transporter mapOldASRsIntoNewASRsAfterLoopClosing(vector<Object> refObjects,vector<ASR> allASRs) {
    vector<Object> tmp;
    Object oldExitInNewASR;
   /* for (int j = 4; j<int(allASRs.size()); j++) {

        for (int i = 0; i<int(allASRs[j].getASRObjects().size()); i++) {
            tmp.push_back(remakeLineP2(refObjects[1], refObjects[0], allASRs[j].getASRObjects()[i], 1, 0, refObjects[0].getKP()));
        }
        allASRs[j].setASRObjects(tmp);
        oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[j].getASRExit1(), 1, 0, refObjects[0].getKP());
        allASRs[j].setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[j].getASRExit2(), 1, 0, refObjects[0].getKP());
        allASRs[j].setASRExit2(oldExitInNewASR);

        tmp.clear();
        for (int i = 0; i<int(allASRs[j].getRoute().size()); i++) {
            tmp.push_back(remakeLineP2(refObjects[1], refObjects[0], allASRs[j].getRoute()[i], 1, 0, refObjects[0].getKP()));
        }
        allASRs[j].replaceTheWholeRoute(tmp);
        tmp.clear();
    }*/
    vector<ASR> ASRsForNewPM;
    for(int i=1;i<int(allASRs.size());i++)
    ASRsForNewPM.push_back(allASRs[i]);
    oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[0].getASRExit2(), 1, 0, refObjects[0].getKP());
    ASRsForNewPM.back().setASRExit2(oldExitInNewASR);
    plotPerceptualMapWithASRs("MFIS/finalPerceptualMap1.png",ASRsForNewPM);
   // vector<Object> tmp;
     for (int i = 0; i<int(allASRs[1].getASRObjects().size()); i++) {
            tmp.push_back(remakeLineP2(refObjects[1], refObjects[0], allASRs[1].getASRObjects()[i], 1, 0, refObjects[0].getKP()));
        }
    allASRs[1].setASRObjects(tmp);
    oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[1].getASRExit1(), 1, 0, refObjects[0].getKP());
        allASRs[1].setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[1].getASRExit2(), 1, 0, refObjects[0].getKP());
        allASRs[1].setASRExit2(oldExitInNewASR);
         for(int i=1;i<int(allASRs.size());i++)
    ASRsForNewPM.push_back(allASRs[i]);
         plotPerceptualMapWithASRs("MFIS/finalPerceptualMap2.png",ASRsForNewPM);
    cout<<"Total ASRs "<<allASRs.size()<<endl;
    Transporter package;
    return package;
}


//it replaces current ASR by new ASR
Transporter makeNewPerceptualMap(vector<Object> refObjects,vector<ASR> allASRs) {
    cout<<"\033[1;31m-------------------Inside makeNewPerceptualMap Module---------------------\033[0m"<<endl;
    vector<ASR> newAllASRs;
    vector<Object> tmp;
    
    ASR lastASR; //last ASR in new MFIS
    lastASR = allASRs.back();
    cout<<"Total num of ASR before closing the loop "<<allASRs.size()<<endl;
   // plotSingleASR("MFIS/lastASR.png",lastASR);
    
    //retrive the next exit from old ASR in current ASR
    Object oldExitInNewASR = remakeLineP2(refObjects[1], refObjects[0], allASRs[0].getASRExit2(), 1, 0, refObjects[0].getKP());
    lastASR.setASRExit2(oldExitInNewASR);
    
    //replace the first one
    allASRs[0].replaceASR(lastASR); 
    
    //delete the last one
    for(int i=0;i<int(allASRs.size()-1);i++)
        newAllASRs.push_back(allASRs[i]);
    cout<<"Total num of ASR after closing the loop "<<newAllASRs.size()<<endl;
   // plotPerceptualMapWithASRs("MFIS/newPerceptualMap.png", newAllASRs);
    //char wait[40];
    //cin>>wait;
    

    Transporter package;
    package.setASRs(newAllASRs);
    return package;
}

void makeFinalASR(ASRNetwork pm, vector<Object> referenceObjects) {
    vector<ASR> allASRs = pm.getASRs();
    ASR cASR =pm.getCurrentASR();
            vector<Object> cView = pm.getCurrentView();
            Object cRobotPosition = pm.getCurrentRobotPosition();//robot position in MFIS
   // plotPerceptualMapWithASRs("MFIS/PerceptualMapWithAllASRs.png",allASRs);
    cout<<"Total number of ASRs "<<allASRs.size()<<endl;
    vector<ASR> newPM;
    
    for(int i=0;i<int(allASRs.size());i++) {
        
        if(i == 6) {//means at home
           // plotSingleASR("MFIS/currentASRBeforeLoopClosing.png",cASR);
            //plotObjects("MFIS/currentView.png",cView,cView);
            
            double leftlimit = 0;
            double rightlimit = 0;
            double horizon = 0;
            for(int j=0;j<int(cView.size());j++){
                    if(cView[j].X1() < leftlimit)
                            leftlimit=cView[j].X1();
                    if(cView[j].X2() < leftlimit)
                            leftlimit=cView[j].X2();

                    if(cView[j].X1() > rightlimit)
                            rightlimit=cView[j].X1();
                    if(cView[j].X2() > rightlimit)
                            rightlimit=cView[j].X2();

                    if(cView[j].Y1() > horizon)
                            horizon=cView[j].Y1();
                    if(cView[j].Y2() > horizon)
                            horizon=cView[j].Y2();
            }
            cout<<"leftlimit "<<leftlimit<<" rightlimit "<<rightlimit<<" horizon "<<horizon<<endl;
            double angle = cRobotPosition.getAngleWithXaxis();
            angle = ((angle / 180) * PI);
           vector<Object> cASRIntoCV=xformPVIntoCV(allASRs[1].getASRObjects(),Point(cRobotPosition.X1(),cRobotPosition.Y1()),angle);
            //plotObjects("MFIS/currentView.png",cASRIntoCV,cView);
            
            vector<Object> tmp;
            for(int k=0;k<int(cASRIntoCV.size());k++) {
                if(cASRIntoCV[k].X1() < leftlimit and cASRIntoCV[k].X2() < leftlimit) 
                     tmp.push_back(allASRs[1].getASRObjects()[k]);
                else if(cASRIntoCV[k].X1() > rightlimit and cASRIntoCV[k].X2() > rightlimit) 
                     tmp.push_back(allASRs[1].getASRObjects()[k]);
            }
            cout<<"objects from asr2 "<<tmp.size()<<endl;
            //replace current old ASR
            newPM[0].replaceASR(cASR);
            //replace next ASR
            
            /* for (int i = 0; i<int(allASRs[1].getASRObjects().size()); i++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRObjects()[i], 1, 0, referenceObjects[0].getKP()));
             }*/
             ASR tmpASR;
             Object oldExitInNewASR = remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRExit1(), 1, 0, referenceObjects[0].getKP());
        tmpASR.setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRExit2(), 1, 0, referenceObjects[0].getKP());
        tmpASR.setASRExit2(oldExitInNewASR);
        tmpASR.setASRObjects(tmp);
        tmp.clear();
        for (int i = 0; i<int(allASRs[1].getRoute().size()); i++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getRoute()[i], 1, 0, referenceObjects[0].getKP()));
             }
        tmpASR.replaceTheWholeRoute(tmp);
        newPM[1].replaceASR(tmpASR);//replace by old one in new coordinate
            plotPerceptualMapWithASRs("MFIS/PerceptualMapAtHome.png",newPM);
            vector<Object> allObjectsInMFIS;
             for(int j =0;j<int(newPM.size());j++) {
            for(int k=0;k<int(newPM[j].getASRObjects().size());k++)
                allObjectsInMFIS.push_back(newPM[j].getASRObjects()[k]);
        }
             plotObjects("MFIS/MFISAtHome.png",allObjectsInMFIS,allObjectsInMFIS);
        }
        else if(i == 7) {
             vector<Object> tmp;
               ASR tmpASR;
               ASR mergedASR;
               
                //get old ASR1 in new coord 
             for (int j = 0; j<int(allASRs[0].getASRObjects().size()); j++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[0].getASRObjects()[j], 1, 0, referenceObjects[0].getKP()));
             }
            tmpASR.setASRObjects(tmp);
            //merge old ASR1 with new ASR1
            mergedASR=mergeOldandNewASRs(tmpASR,allASRs[6]);//arg: old in new coord and new ASR
            newPM[0].replaceASR(mergedASR);//replace by old one in new coordinate
            tmp.clear();  
               //get old ASR2 in new coord 
             for (int j = 0; j<int(allASRs[1].getASRObjects().size()); j++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRObjects()[j], 1, 0, referenceObjects[0].getKP()));
             }
           
             Object oldExitInNewASR = remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRExit1(), 1, 0, referenceObjects[0].getKP());
        tmpASR.setASRExit1(oldExitInNewASR);
        oldExitInNewASR = remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getASRExit2(), 1, 0, referenceObjects[0].getKP());
        tmpASR.setASRExit2(oldExitInNewASR);
        tmpASR.setASRObjects(tmp);
        
        tmp.clear();
        for (int j = 0; j<int(allASRs[1].getRoute().size()); j++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[1].getRoute()[j], 1, 0, referenceObjects[0].getKP()));
             }
        tmpASR.replaceTheWholeRoute(tmp);
        //merge old ASR2 with new ASR2
        mergedASR=mergeOldandNewASRs(tmpASR,allASRs[i]);//arg: old in new coord and new ASR
        //plotObjects("MFIS/ASR2OldandNew.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
         //plotObjects("MFIS/ASR2OldandNew2.png",tmpASR.getASRObjects(),allASRs[i].getASRObjects());
         cout<<" "<<tmpASR.getASRObjects().size()<<" "<<allASRs[i].getASRObjects().size()<<" "<<mergedASR.getASRObjects().size()<<endl;
         mergedASR.setASRExit1(tmpASR.getASRExit1());
         mergedASR.setASRExit2(tmpASR.getASRExit2());
         mergedASR.replaceTheWholeRoute(tmpASR.getRoute());
        newPM[1].replaceASR(mergedASR);//replace by old one in new coordinate
            //newPM[1].replaceASR(allASRs[i]); //replace by new one
        vector<Object> allObjectsInMFIS;
        for(int j =0;j<int(newPM.size());j++) {
            for(int k=0;k<int(newPM[j].getASRObjects().size());k++)
                allObjectsInMFIS.push_back(newPM[j].getASRObjects()[k]);
        }
            //plotPerceptualMapWithASRs("MFIS/PerceptualMapAtASR2MergedASR.png",newPM);
       plotObjects("MFIS/MFISAtASR2.png",allObjectsInMFIS,allObjectsInMFIS);
        }
        else if(i == 8){
            //get old asr in new view
            vector<Object> tmp;
             for (int j = 0; j<int(newPM[5].getASRObjects().size()); j++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], newPM[5].getASRObjects()[j], 1, 0, referenceObjects[0].getKP()));
             }
             ASR tmpASR;
             tmpASR.setASRObjects(tmp);
             ASR mergedASR=mergeOldandNewASRs(tmpASR,allASRs[i]);//arg: old in new coord and new ASR
            newPM[5].replaceASR(mergedASR);
            vector<Object> allObjectsInMFIS;
        for(int j =0;j<int(newPM.size());j++) {
            for(int k=0;k<int(newPM[j].getASRObjects().size());k++)
                allObjectsInMFIS.push_back(newPM[j].getASRObjects()[k]);
        }
            //plotPerceptualMapWithASRs("MFIS/PerceptualMapAtASR2MergedASR.png",newPM);
        plotObjects("MFIS/MFISAtASR5.png",allObjectsInMFIS,allObjectsInMFIS);
            //plotPerceptualMapWithASRs("MFIS/PerceptualMapAtASR6.png",newPM);
        
                 tmp.clear();      
                //get old ASR5 in new coord 
             for (int j = 0; j<int(allASRs[4].getASRObjects().size()); j++) {
                tmp.push_back(remakeLineP2(referenceObjects[1], referenceObjects[0], allASRs[4].getASRObjects()[j], 1, 0, referenceObjects[0].getKP()));
             }
            tmpASR.setASRObjects(tmp);
            //merge old ASR5 with new ASR5
            mergedASR=mergeOldandNewASRs(tmpASR,allASRs[10]);//arg: old in new coord and new ASR
            newPM[4].replaceASR(mergedASR);//replace by old one in new coordinate
            allObjectsInMFIS.clear();
            for(int j =0;j<int(newPM.size());j++) {
            for(int k=0;k<int(newPM[j].getASRObjects().size());k++)
                allObjectsInMFIS.push_back(newPM[j].getASRObjects()[k]);
        }
             plotObjects("MFIS/MFISAtASR4.png",allObjectsInMFIS,allObjectsInMFIS);
        }
        else
        newPM.push_back(allASRs[i]);
    }
    
}

ASR mergeOldandNewASRs(ASR oldASR, ASR newASR) {
    vector<Object> mergedASR = newASR.getASRObjects();
    vector<Object> output=mergedASR;
    double angle,distance;
    bool deleteThis = false;
    bool deleteThisFromNewASR = false;
//    plotSingleASR("MFIS/NewASRInMergedASR.png",newASR);
    for(int j=0;j<int(oldASR.getASRObjects().size());j++) {
        deleteThis = false;
        for(int i=0;i<int(mergedASR.size());i++) {
//            deleteThisFromNewASR = false;
            angle=mergedASR[i].getAngleWithLine(oldASR.getASRObjects()[j]);
            if (abs(angle) < 8 || abs(angle) > 352 || (abs(angle) > 172 && abs(angle) < 188)) {
                distance = shortestDistanceBtwTwoObjects(mergedASR[i],oldASR.getASRObjects()[j]);
                if(distance < 800 ) {
                    deleteThis = true;
                    if(abs(mergedASR[i].length()-oldASR.getASRObjects()[j].length()) > 1500)
                        if(oldASR.getASRObjects()[j].length() > mergedASR[i].length()) {
                            deleteThis = false;
//                            deleteThisFromNewASR = true;
                        }
                    //cout<<"deleted"<<endl;
                    break;
                }
            }
//            if(deleteThisFromNewASR == false)
//                output.push_back(mergedASR[i]);
                
        }
        if(deleteThis == false) {
           output.push_back(oldASR.getASRObjects()[j]);
        }
    }
    ASR outputASR;
    outputASR.setASRObjects(output);
    outputASR.setASRExit1(newASR.getASRExit1());
    outputASR.setASRExit2(newASR.getASRExit2());
    outputASR.replaceTheWholeRoute(newASR.getRoute());
    outputASR.setLineOfSitePoints(newASR.getLineOfSitePoints());
    return outputASR;
}

ASR mergeOldandNewASRsUsingAngleAndLoS(ASR oldASR, ASR newASR) {
    vector<Object> mergedASR = newASR.getASRObjects();
    vector<Object> lineOfSitePoints = newASR.getLineOfSitePoints();
    Object selectedLineOfSitePoint,newObjectToLoS,oldObjectToLoS;
    double distanceToLoSPoints;
    vector<Object> output;//=mergedASR;
    double angle,distance;
    bool deleteThis = false;
    vector<int> deleteFromNewASR,deleteFromOldASR;
    for(int j=0;j<int(oldASR.getASRObjects().size());j++) {
        deleteThis = false;
        for(int i=0;i<int(mergedASR.size());i++) {
            angle=mergedASR[i].getAngleWithLine(oldASR.getASRObjects()[j]);           
            
            if ((abs(angle) < 10 || abs(angle) > 350) || (abs(angle) > 170 && abs(angle) < 190)) {
                distance = shortestDistanceBtwTwoObjects(mergedASR[i],oldASR.getASRObjects()[j]);
                if(mergedASR[i].length() > 5000 && mergedASR[i].length() < 6000 && oldASR.getASRObjects()[j].length() > 4000) {
                cout<<"angle "<<angle<<" distance "<<distance<<" lengthNewLine "<<mergedASR[i].length()<<" oldLine "<<oldASR.getASRObjects()[j].length()<<endl;
            }
                if(distance < 400 ) {
                   
                    deleteThis = true;
//                    cout<<"angle "<<angle<<" dist "<<distance<<" length "<<mergedASR[i].length()<<endl;
                    if(abs(mergedASR[i].length()-oldASR.getASRObjects()[j].length()) > 1000) {
                        
                        if(oldASR.getASRObjects()[j].length() > mergedASR[i].length()) {
                            deleteThis = false;//means this jth oldObject staying so we have to compare this jth oldObject with other newObject
                                                                //that's why no break
                            deleteFromNewASR.push_back(i);
                        }
                        else
                            break;
                    }
                    else  //means we need to delete this jth oldObject so no need to come back again for this oldObject
                        break;
                }
                else if(distance < 1000) {
                    selectedLineOfSitePoint = lineOfSitePoints[0];
                    distanceToLoSPoints = mergedASR[i].distMPToPoint(lineOfSitePoints[0].X1(),lineOfSitePoints[0].Y1());
                    for(int k=0;k<int(lineOfSitePoints.size());k++){
                        if(distanceToLoSPoints > mergedASR[i].distMPToPoint(lineOfSitePoints[k].X1(),lineOfSitePoints[k].Y1())) {
                            selectedLineOfSitePoint = lineOfSitePoints[k];
                            distanceToLoSPoints = mergedASR[i].distMPToPoint(lineOfSitePoints[k].X1(),lineOfSitePoints[k].Y1());
                        }
                    }
                    newObjectToLoS.set(mergedASR[i].mpX(),mergedASR[i].mpY(),selectedLineOfSitePoint.X1(),selectedLineOfSitePoint.Y1(),1);
                    oldObjectToLoS.set(oldASR.getASRObjects()[j].mpX(),oldASR.getASRObjects()[j].mpY(),selectedLineOfSitePoint.X1(),selectedLineOfSitePoint.Y1(),1);
                    if((checkForIntersection(newObjectToLoS,oldASR.getASRObjects()[j]) == 1) or checkForIntersection(oldObjectToLoS,mergedASR[i]) == 1) {
                        deleteThis = true;
                        if(abs(mergedASR[i].length()-oldASR.getASRObjects()[j].length()) > 1000) {
                        
                            if(oldASR.getASRObjects()[j].length() > mergedASR[i].length()) {
                                deleteThis = false;//means this jth oldObject staying so we have to compare this jth oldObject with other newObject
                                                                //that's why no break
                                deleteFromNewASR.push_back(i);
                            }
                            else
                                break;
                        }
                        else //means we need to delete this jth oldObject so no need to come back again for this oldObject
                            break;
                    }
                   
                }
            }
        }
        if(deleteThis == false) {
           output.push_back(oldASR.getASRObjects()[j]);
        }
    }

    vector<Object> tmpForTesting;
    for(int i=0;i<int(mergedASR.size());i++){
        deleteThis = false;
        for(int j=0;j<int(deleteFromNewASR.size());j++){
            if(i == deleteFromNewASR[j]){
                deleteThis = true;
//                cout<<"delete this "<<endl;
//                cout<<"len ++ "<<mergedASR[i].length()<<endl;
            }
        }
        if(deleteThis == false){
            output.push_back(mergedASR[i]);
//            cout<<"len "<<mergedASR[i].length()<<endl;
        }
        
        //testing purpose
        if(mergedASR[i].length() > 4000){
//            cout<<"Length "<<mergedASR[i].length()<<endl;
            tmpForTesting.push_back(mergedASR[i]);
        }
        
    }
//    plotObjects("MFIS/BigLinesFromASR-14.png",tmpForTesting,tmpForTesting);
//    char wait[10];
//    cin>>wait;
    ASR outputASR;
    outputASR.setASRObjects(output);
    outputASR.setASRExit1(newASR.getASRExit1());
    outputASR.setASRExit2(newASR.getASRExit2());
    outputASR.replaceTheWholeRoute(newASR.getRoute());
    outputASR.setLineOfSitePoints(newASR.getLineOfSitePoints());
    return outputASR;
}

void makeFinalPMUsingOldASRs(ASRNetwork pm, vector<Object> fAndLRP) {
    vector<ASR> allASRs,mergedAllASRs;
    allASRs = pm.getASRs();
    mergedAllASRs = pm.getAllASRs();
    
    vector<ASR> tmpFinalASRs;
    vector<ASR> finalASRs;
    for(int j=0;j<6;j++) {
        finalASRs.push_back(pm.getASRs()[j]);
    }
    vector<Object> allObjectsInMFIS;
    for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("Maps/PMat6.png", allObjectsInMFIS, allASRs[6].getASRObjects());
    cout<<"7 ASRs printed"<<endl;
    waitHere();
//    perceptualMapUptoCurrentView(pm,finalASRs);
    
     Object referenceObjectOld, referenceObjectNew;
     vector<Object> tmp,rotatedObjects;
     ASR tmpASR;
     Object oldExitInNewASR;
     
     //computing ASR1 on top of new ASR1(7)     
     cout<<"Merging ASR-1 and ASR-7"<<endl;
     referenceObjectOld.set(pm.getASRs()[0].getASRExit2());
     referenceObjectNew.set(pm.getASRs()[6].getASRExit2());     
     for (int j = 0; j<int(pm.getASRs()[0].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[0].getASRObjects()[j], 1, 0, 1));
     }     
     //for rotation
     vector<Object> objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[6].getASRObjects(),referenceObjectNew);
     //plotObjects("MFIS/selected-1-7.png",objects4Rotating,objects4Rotating);
     referenceObjectNew.set(objects4Rotating[1]);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[0]);
     //tmp.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     tmpASR.setASRObjects(rotatedObjects);
     tmpASR.replaceTheWholeRoute(allASRs[6].getRoute());
     allASRs[0].replaceASR(tmpASR);//replace by old one in new coordinate
     plotObjects("Maps/RotatedASR1andASR7.png",pm.getASRs()[6].getASRObjects(),rotatedObjects);
//     ASR mergedASR=mergeOldandNewASRs(tmpASR,pm.getASRs()[6]);//merging old and new ASR
     ASR mergedASR = mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getASRs()[6]);
//     plotObjects("MFIS/MergedASR1-7.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
//     plotSingleASR("MFIS/ASR7.png",pm.getASRs()[6]);
     plotSingleASR("Maps/MergedASR1-7.png",mergedASR);
     mergedASR.setASRExit1(allASRs[6].getASRExit1());
     mergedASR.setASRExit2(allASRs[6].getASRExit2());
//     mergedAllASRs[0].replaceASR(mergedASR);
     finalASRs[0].replaceASR(mergedASR);
     //printing pm at ASR7
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjectsOf3Kinds("Maps/PMat1.png", allObjectsInMFIS,mergedASR.getASRObjects(), fAndLRP);
    
    waitHere();
//    perceptualMapUptoCurrentView(pm,finalASRs);
     
     //computing ASR2 on top of new ASR2(8)
     cout<<"Merging ASR-2 and ASR-8"<<endl;
     tmp.clear();
     referenceObjectOld.set(pm.getASRs()[1].getASRExit1());
     referenceObjectNew.set(pm.getASRs()[7].getASRExit1());
     
     for (int j = 0; j<int(pm.getASRs()[1].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[1].getASRObjects()[j], 1, 0, 1));
     }     
     //for rotating
     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[7].getASRObjects(),referenceObjectNew);
     //plotObjects("MFIS/selected-2-8.png",objects4Rotating,objects4Rotating);
      referenceObjectNew.set(objects4Rotating[1]);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[0]);
     rotatedObjects.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     
     tmpASR.setASRObjects(rotatedObjects);
     
     allASRs[1].replaceASR(tmpASR);//replace by old one in new coordinate
     plotObjects("MFIS/RotatedASR2andASR8.png",pm.getASRs()[7].getASRObjects(),rotatedObjects);
//     mergedASR=mergeOldandNewASRs(tmpASR,pm.getASRs()[7]);//merging old and new ASR
     mergedASR = mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getASRs()[7]);
//     plotObjects("MFIS/MargedASR2-8.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR2-8.png",mergedASR);
    mergedASR.setASRExit1(allASRs[7].getASRExit1());
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[1].getASRExit2(), 1, 0, 1);
     mergedASR.setASRExit2(oldExitInNewASR);
     mergedASR.replaceTheWholeRoute(allASRs[7].getRoute());
     mergedAllASRs[1].replaceASR(mergedASR);
     //     tmpFinalASRs.push_back(tmpASR);
   //  tmpFinalASRs.push_back(allASRs[7]);
     finalASRs[1].replaceASR(mergedASR);
     //printing pm at ASR7
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat2.png", allObjectsInMFIS, allObjectsInMFIS);
//    perceptualMapUptoCurrentView(pm,finalASRs);
    
    
    //inserting ASR 9
     cout<<"Merging ASR-9"<<endl;
    vector<Object> tmpforASR9;
    for(int j=0;j<int(pm.getASRs()[8].getASRObjects().size());j++) {
        if(pm.getASRs()[8].getASRObjects()[j].length() < 3000)
            tmpforASR9.push_back(pm.getASRs()[8].getASRObjects()[j]);
    }
    tmpASR.setASRObjects(tmpforASR9);
     finalASRs.push_back(tmpASR);
//     plotObjects("MFIS/ASR9.png",tmpforASR9,tmpforASR9);
     //printing pm at ASR7
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat7.png", allObjectsInMFIS, allObjectsInMFIS);
     
     //computing ASR6 on top of new ASR6(10)
     cout<<"Merging ASR-6 and ASR-10"<<endl;
     tmp.clear();
     referenceObjectOld.set(allASRs[5].getASRExit1().X2(),allASRs[5].getASRExit1().Y2(),allASRs[5].getASRExit1().X1(),allASRs[5].getASRExit1().Y1(),1);
     referenceObjectNew.set(pm.getASRs()[9].getASRExit2());
     referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
     for (int j = 0; j<int(pm.getASRs()[5].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[5].getASRObjects()[j], 1, 0, 1));
     }                
     //rotating
     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[9].getASRObjects(),referenceObjectNew);
     //plotObjects("MFIS/selected-6-10.png",objects4Rotating,objects4Rotating);
     referenceObjectNew.set(objects4Rotating[1]);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[0]);
     rotatedObjects.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     
     tmpASR.setASRObjects(rotatedObjects);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[5].getASRExit1(), 1, 0, 1);
     tmpASR.setASRExit1(oldExitInNewASR);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[5].getASRExit2(), 1, 0, 1);
     tmpASR.setASRExit2(oldExitInNewASR);
     tmpASR.replaceTheWholeRoute(allASRs[9].getRoute());
     allASRs[5].replaceASR(tmpASR);//replace by old one in new coordinate
     plotObjects("MFIS/RotatedASR6andASR10.png",pm.getASRs()[9].getASRObjects(),rotatedObjects);
      mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getASRs()[9]);//merging old and new ASR
//      plotObjects("MFIS/MergedASR6-10.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
      plotSingleASR("MFIS/MergedASR6-10.png",mergedASR);
//      mergedAllASRs[5].replaceASR(mergedASR);
//           tmpFinalASRs.push_back(tmpASR);
//      tmpFinalASRs.push_back(allASRs[9]);
      finalASRs[5].replaceASR(mergedASR);
     //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat6Revisiting.png", allObjectsInMFIS, allObjectsInMFIS);
//     perceptualMapUptoCurrentView(pm,finalASRs);
         
      //computing ASR5 on top of new ASR5(11)
     cout<<"Merging ASR-5 and ASR-11"<<endl;
     tmp.clear();
     referenceObjectOld.set(allASRs[4].getASRExit2().X2(),allASRs[4].getASRExit2().Y2(),allASRs[4].getASRExit2().X1(),allASRs[4].getASRExit2().Y1(),1);
     referenceObjectNew.set(pm.getASRs()[10].getASRExit1());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
//     cout<<"ErrorAngle of refObjects for ASR5 "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     for (int j = 0; j<int(pm.getASRs()[4].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[4].getASRObjects()[j], 1, 0, 1));
     }
     plotObjects("MFIS/ASR5andASR11.png",pm.getASRs()[10].getASRObjects(),tmp);
     //for rotating
     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[10].getASRObjects(),referenceObjectNew);
     plotObjects("MFIS/RotatingObjects5-11.png",objects4Rotating,objects4Rotating);
//      referenceObjectNew.set(objects4Rotating[1].X2(),objects4Rotating[1].Y2(),objects4Rotating[1].X1(),objects4Rotating[1].Y1(),1);
      referenceObjectNew.set(objects4Rotating[3]);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[2]);
//     cout<<" length of rotating objects "<<endl;
//     for(int j=0;j<int(objects4Rotating.size());j++)
//         cout<<" "<<objects4Rotating[j].length();
//     cout<<"Error angle btw rotating objects "<<objects4Rotating[1].getAngleWithLine(objects4Rotating[0]);
     rotatedObjects.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     
     tmpASR.setASRObjects(rotatedObjects);
     
//     allASRs[4].replaceASR(tmpASR);//replace by old one in new coordinate
     plotObjects("MFIS/RotatedASR5-11.png",pm.getASRs()[10].getASRObjects(),rotatedObjects);
     mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getASRs()[10]);//merging old and new ASR
//     plotObjects("MFIS/MergedASR5-11.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR5-11.png",mergedASR);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[4].getASRExit1(), 1, 0, 1);
     mergedASR.setASRExit1(oldExitInNewASR);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[4].getASRExit2(), 1, 0, 1);
     mergedASR.setASRExit2(oldExitInNewASR);
//    mergedASR.replaceTheWholeRoute(allASRs[11].getRoute());
//     mergedAllASRs[4].replaceASR(mergedASR);
     //tmpASR.setASRObjects(tmp);
    // tmpFinalASRs.push_back(tmpASR);
 //    tmpFinalASRs.push_back(allASRs[6]);
     finalASRs[4].replaceASR(mergedASR);
     //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat5Revisiting.png", allObjectsInMFIS, allObjectsInMFIS);
    
    
    
    
    
    
    
    
    
//     perceptualMapUptoCurrentView(pm,finalASRs);
     
//          //computing ASR5 on top of new ASR5(13)
//     tmp.clear();
//     referenceObjectOld.set(finalASRs[4].getASRExit1().X2(),finalASRs[4].getASRExit1().Y2(),finalASRs[4].getASRExit1().X1(),finalASRs[4].getASRExit1().Y1(),1);
////     referenceObjectOld.set(allASRs[4].getASRExit1().X2(),allASRs[4].getASRExit1().Y2(),allASRs[4].getASRExit1().X1(),allASRs[4].getASRExit1().Y1(),1);
//     referenceObjectNew.set(pm.getASRs()[12].getASRExit2());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
//     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
//     for (int j = 0; j<int(finalASRs[4].getASRObjects().size()); j++) {
//        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, finalASRs[4].getASRObjects()[j], 1, 0, 1));
//     }     
//     //for rotating
//     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[12].getASRObjects(),referenceObjectNew);
//     cout<<"objects for rotating "<<objects4Rotating.size()<<endl;
////     plotObjects("MFIS/selected-5-13.png",objects4Rotating,objects4Rotating);
//      referenceObjectNew.set(objects4Rotating[1]);
//     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
//     referenceObjectOld.set(objects4Rotating[0]);
//     rotatedObjects.clear();
//     for (int j = 0; j<int(tmp.size()); j++) {
//        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
//     }
//     
//     tmpASR.setASRObjects(rotatedObjects);
//     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, mergedAllASRs[4].getASRExit1(), 1, 0, 1);
//     tmpASR.setASRExit1(oldExitInNewASR);
//     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, mergedAllASRs[4].getASRExit2(), 1, 0, 1);
//     tmpASR.setASRExit2(oldExitInNewASR);
//     tmpASR.replaceTheWholeRoute(allASRs[12].getRoute());
//     allASRs[4].replaceASR(tmpASR);//replace by old one in new coordinate
////     plotObjects("MFIS/RotatedASR5-13.png",pm.getASRs()[12].getASRObjects(),rotatedObjects);
//     mergedASR=mergeOldandNewASRs(tmpASR,allASRs[12]);//merging old and new ASR
//     //plotObjects("MFIS/ASR5-12.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
////     mergedAllASRs[4].replaceASR(mergedASR);
////          tmpFinalASRs.push_back(tmpASR);
////     tmpFinalASRs.push_back(allASRs[12]);
//     finalASRs[4].replaceASR(mergedASR);
//     //printing pm at ASR6
//     allObjectsInMFIS.clear();
//     for(int j =0;j<int(finalASRs.size());j++) {
//        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
//            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
//    }
////    plotObjects("MFIS/PMat5RevisitingAgain.png", allObjectsInMFIS, allObjectsInMFIS);
////     perceptualMapUptoCurrentView(pm,finalASRs);
     
     //computing ASR4 on top of new ASR4(12)
     cout<<"Merging ASR-4 and ASR-12"<<endl;
     tmp.clear();
     referenceObjectOld.set(allASRs[3].getASRExit2().X2(),allASRs[3].getASRExit2().Y2(),allASRs[3].getASRExit2().X1(),allASRs[3].getASRExit2().Y1(),1);
     referenceObjectNew.set(pm.getASRs()[11].getASRExit1());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     for (int j = 0; j<int(pm.getASRs()[3].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[3].getASRObjects()[j], 1, 0, 1));
     }
//     plotObjects("MFIS/ASR4andASR14.png",pm.getASRs()[13].getASRObjects(),tmp);
      //for rotating
     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[11].getASRObjects(),referenceObjectNew);
     cout<<"objects for rotating "<<objects4Rotating.size()<<endl;
//     plotObjects("MFIS/selected-4-14.png",objects4Rotating,objects4Rotating);
      referenceObjectNew.set(objects4Rotating[1]);//.X2(),objects4Rotating[1].Y2(),objects4Rotating[1].X1(),objects4Rotating[1].Y1(),1);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[0]);
     rotatedObjects.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     
     tmpASR.setASRObjects(rotatedObjects);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, allASRs[3].getASRExit1(), 1, 0, 1);
     tmpASR.setASRExit1(oldExitInNewASR);
     oldExitInNewASR = remakeLineP2(referenceObjectNew,referenceObjectOld, allASRs[3].getASRExit2(), 1, 0, 1);
     tmpASR.setASRExit2(oldExitInNewASR);
     tmpASR.replaceTheWholeRoute(allASRs[11].getRoute());
     allASRs[4].replaceASR(tmpASR);//replace by old one in new coordinate
     plotObjects("MFIS/RotatedASR4-12.png",pm.getASRs()[11].getASRObjects(),rotatedObjects);
     mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,allASRs[11]);//merging old and new ASR
//     plotObjects("MFIS/MergedASR4-12.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR4-12.png",mergedASR);
//     //tmpASR.setASRObjects(tmp);
//          tmpFinalASRs.push_back(tmpASR);
//     tmpFinalASRs.push_back(allASRs[13]);
     finalASRs[3].replaceASR(mergedASR);
     //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat4.png", allObjectsInMFIS, allObjectsInMFIS);
//     perceptualMapUptoCurrentView(pm,finalASRs);
     
     //computing ASR3 on top of new ASR3(13)
     cout<<"Merging ASR-3 and ASR-13"<<endl;
     tmp.clear();
     referenceObjectOld.set(pm.getASRs()[2].getASRExit2().X2(),pm.getASRs()[2].getASRExit2().Y2(),pm.getASRs()[2].getASRExit2().X1(),pm.getASRs()[2].getASRExit2().Y1(),1);
     referenceObjectNew.set(pm.getASRs()[12].getASRExit1());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     for (int j = 0; j<int(pm.getASRs()[2].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, pm.getASRs()[2].getASRObjects()[j], 1, 0, 1));
     }
     plotObjects("MFIS/ASR3andASR13.png",pm.getASRs()[12].getASRObjects(),tmp);
     tmpASR.setASRObjects(tmp);
     mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,allASRs[12]);//merging old and new ASR
//     plotObjects("MFIS/MergedASR3-13.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR3-13.png",mergedASR);
//     tmpFinalASRs.push_back(tmpASR);
//     tmpFinalASRs.push_back(allASRs[14]);
     finalASRs[2].replaceASR(mergedASR);
     //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
        plotObjects("MFIS/PMat3.png", allObjectsInMFIS, allObjectsInMFIS);
//     perceptualMapUptoCurrentView(pm,finalASRs);
     
     //computing ASR2 on top of new ASR2(14)
     cout<<"Merging ASR-2 and ASR-14"<<endl;
     tmp.clear();
     referenceObjectOld.set(finalASRs[1].getASRExit1().X2(),finalASRs[1].getASRExit1().Y2(),finalASRs[1].getASRExit1().X1(),finalASRs[1].getASRExit1().Y1(),1);
     referenceObjectNew.set(pm.getASRs()[13].getASRExit2());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     for (int j = 0; j<int(finalASRs[1].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, finalASRs[1].getASRObjects()[j], 1, 0, 1));
     }
//     plotObjects("MFIS/ASR2andASR16.png",pm.getASRs()[15].getASRObjects(),tmp);
     
     objects4Rotating = findObjectsForRotatingTwoViews(tmp,pm.getASRs()[13].getASRObjects(),referenceObjectNew);
//     plotObjects("MFIS/selected-2-16.png",objects4Rotating,objects4Rotating);
     referenceObjectNew.set(objects4Rotating[1]);//.X2(),objects4Rotating[1].Y2(),objects4Rotating[1].X1(),objects4Rotating[1].Y1(),1);
     //referenceObjectOld.set(makeLineAtPointWithObject(abs(referenceObjectNew.getAngleWithLine(objects4Rotating[0])),0,objects4Rotating[0]));
     referenceObjectOld.set(objects4Rotating[0]);
     rotatedObjects.clear();
     for (int j = 0; j<int(tmp.size()); j++) {
        rotatedObjects.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, tmp[j], 1, 0, 1));
     }
     plotObjects("MFIS/RotatedASR2-14.png",pm.getASRs()[13].getASRObjects(),rotatedObjects);
     tmpASR.setASRObjects(rotatedObjects);
     mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getASRs()[13]);//merging old and new ASR     
//      plotObjects("MFIS/MergedASR2-14.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR2-14.png",mergedASR);
      finalASRs[1].replaceASR(mergedASR);
      //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMat2.png", allObjectsInMFIS, allObjectsInMFIS);
//perceptualMapUptoCurrentView(pm,finalASRs);
     
     //computing ASR1 on top of new ASR1(15)
     cout<<"No of ASRS "<<pm.getASRs().size()<<endl;
     cout<<"Merging ASR-1 and ASR-15"<<endl;
     tmp.clear();
     cout<<"tmp size "<<tmp.size()<<endl;
     referenceObjectOld.set(finalASRs[0].getASRExit2().X2(),finalASRs[0].getASRExit2().Y2(),finalASRs[0].getASRExit2().X1(),finalASRs[0].getASRExit2().Y1(),1);
     referenceObjectNew.set(pm.getCurrentASR().getASRExit1());
//     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     referenceObjectOld.set(makeLineAtPointWithObject(- (referenceObjectNew.getAngleWithLine(referenceObjectOld)),0,referenceObjectOld));
     cout<<"ErrorAngle of refObjects "<<referenceObjectNew.getAngleWithLine(referenceObjectOld)<<endl;
     cout<<"ASR  1 size "<<finalASRs[0].getASRObjects().size()<<endl;
//     referenceObjectNew.display();
//     referenceObjectOld.display();
     cout<<referenceObjectNew.getGID()<<endl;
     for (int j = 0; j<int(finalASRs[0].getASRObjects().size()); j++) {
        tmp.push_back(remakeLineP2(referenceObjectNew,referenceObjectOld, finalASRs[0].getASRObjects()[j], 1, 0, 1));
     }
     //cout<<"tmp size "<<tmp.size()<<" asr 15 size "<<pm.getASRs()[14].getASRObjects().size()<<endl;
//    tmp=finalASRs[0].getASRObjects();
     plotObjects("MFIS/ASR1andASR15.png",pm.getCurrentASR().getASRObjects(),tmp);
     tmpASR.setASRObjects(tmp);
     mergedASR=mergeOldandNewASRsUsingAngleAndLoS(tmpASR,pm.getCurrentASR());//merging old and new ASR
//     //tmpASR.setASRObjects(rotatedObjects);
//      plotObjects("MFIS/MergedASR1-15.png",mergedASR.getASRObjects(),mergedASR.getASRObjects());
     plotSingleASR("MFIS/MergedASR1-15.png",mergedASR);
      finalASRs[0].replaceASR(mergedASR);
      //printing pm at ASR6
     allObjectsInMFIS.clear();
     for(int j =0;j<int(finalASRs.size());j++) {
        for(int k=0;k<int(finalASRs[j].getASRObjects().size());k++)
            allObjectsInMFIS.push_back(finalASRs[j].getASRObjects()[k]);
    }
    plotObjects("MFIS/PMHome.png", allObjectsInMFIS, allObjectsInMFIS);


}

vector<Object> findObjectsForRotatingTwoViews(vector<Object> oldView, vector<Object> newView, Object exit) {
    double shortestDistance, angleOldLine, angleNewLine, angleDifference;
    vector<Object> selected4orientation;
    double length = 2000;
    int loop =1;
    while(loop <3){
        for (int i = 0; i<int(oldView.size()); i++) {
            if (oldView[i].length() > length) {
                angleOldLine = exit.getAngleWithLine(oldView[i]);
                for (int j = 0; j<int(newView.size()); j++) {
                    if (newView[j].length() > length) {
                        angleNewLine = exit.getAngleWithLine(newView[j]);
                        if (abs(angleDifference) < 40) {
                            shortestDistance = shortestDistanceBtwTwoObjects(oldView[i], newView[j]);
                            if (shortestDistance < 500) {
                                selected4orientation.push_back(oldView[i]);
                                selected4orientation.push_back(newView[j]);
//                                cout<<"oldObject Length "<<oldView[i].length()<<" "<<newView[j].length()<<endl;
                                //return selected4orientation;
                            }
                        }
                    }
                }
            }
        }
        if(selected4orientation.size() == 0) {
            length = 1000;
        }
        else 
            break;
        loop++;
    }
    
    cout << "angleDifference " << angleDifference << endl;
    return selected4orientation;
}

void   perceptualMapUptoCurrentView(ASRNetwork pm,vector<ASR> ASRs) {
       cout<<"---------- INSIDE -------------"<<endl;
       cout<<"Total num of ASRs "<<pm.getAllCurrentViews().size()<<endl;
       cout<<"Total Robot Positions "<<pm.getAllRobotPositions().size()<<endl;
       int viewNum =221;
       vector<Object> cView = pm.getAllCurrentViews()[viewNum];
       Object cRobotPositionInMFIS = pm.getAllRobotPositions()[viewNum][7];//x-axis at current robot position
        double leftlimit = 0;
        double rightlimit = 0;
        double horizon = 0;
        for (int j = 0; j<int(cView.size()); j++) {
            if (cView[j].X1() < leftlimit)
                leftlimit = cView[j].X1();
            if (cView[j].X2() < leftlimit)
                leftlimit = cView[j].X2();

            if (cView[j].X1() > rightlimit)
                rightlimit = cView[j].X1();
            if (cView[j].X2() > rightlimit)
                rightlimit = cView[j].X2();

            if (cView[j].Y1() > horizon)
                horizon = cView[j].Y1();
            if (cView[j].Y2() > horizon)
                horizon = cView[j].Y2();
        }
        cout << "leftlimit " << leftlimit << " rightlimit " << rightlimit << " horizon " << horizon << endl;
        double angle = cRobotPositionInMFIS.getAngleWithXaxis();
        angle = ((angle / 180) * PI);
        cout<<"angle "<<angle<<endl;
        vector<Object> oneASRObjects = ASRs[2].getASRObjects();
        vector<Object> cASRIntoCV = xformPVIntoCV(oneASRObjects, Point(cRobotPositionInMFIS.X1(),cRobotPositionInMFIS.Y1()), angle);

        //deleting lines from current ASR which are inside current view
        vector<Object> tmpForPrint;
        for (int k = 0; k<int(cASRIntoCV.size()); k++) {
            if (cASRIntoCV[k].X1() < leftlimit and cASRIntoCV[k].X2() < leftlimit)
                tmpForPrint.push_back(oneASRObjects[k]);
            else if (cASRIntoCV[k].X1() > rightlimit and cASRIntoCV[k].X2() > rightlimit)
                tmpForPrint.push_back(oneASRObjects[k]);
            else if (cASRIntoCV[k].Y1() > horizon and cASRIntoCV[k].Y2() > horizon)
                tmpForPrint.push_back(oneASRObjects[k]);
        }
        //getting all lines from ASR except overlapped ASR(with cv)
        for(int j =0;j<int(ASRs.size());j++) {
            if(j != 2)
            for(int k=0;k<int(ASRs[j].getASRObjects().size());k++)
                tmpForPrint.push_back(ASRs[j].getASRObjects()[k]);
        }
        //getting cv into MFIS.
        vector<Object> newCV;
        for(int i=0;i<int(cView.size());i++) {
            tmpForPrint.push_back(remakeLineP2(pm.getAllReferenceObjects()[viewNum][0], pm.getAllReferenceObjects()[viewNum][1], cView[i], 1, 0, pm.getAllReferenceObjects()[viewNum][0].getKP()));
        }
        
        plotObjects("MFIS/MFISat268-1.png" ,pm.getAllRobotPositions()[221], tmpForPrint);//pm.getAllRobotPositions()[102]);
    }