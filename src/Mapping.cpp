/*
 *  Functions for Mapping like MFIS computation...
 *
 *  Hossain
 *	29 December, 2011
 */

#include "asr.H"


#include "Minfo.H"


#include "Mapping.H"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <valarray>  // Needed for sort() method
#include <vector>
#include <complex>

#include "Object.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "asrOp.H"
#include "mfisOp.H"
#include "readAndwriteASCII.H"


#include "CompareASRClass.H"
#include "CompareASR.H"

#include "Laser2Surface.H"
#include "generalFunctions.h"

/*#include <mrpt/base.h>
#include <mrpt/slam.h>
#include <mrpt/gui.h>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::random;*/
using namespace std;


//mrpt::gui::CDisplayWindow3DPtr  win;
#define PI 3.14159265

using namespace std;


Transporter recognizeTargetObjects(vector<Object> mfis, vector<Object> po_pv, vector<Object> pobjects_cv, 
        vector<double> ctda, int viewNumber, int lastUS) {
    cout << "\033[1;32m-------------------Inside recognizeTargetObjects module<>---------------------\033[0m" << endl;
    Transporter output;
    Object rp(0, 0, 0, 300, 1); //current robot position
    
    MyRobot myrobot(0, 0);
    vector<Object> robloc = myrobot.getRobot();

    vector<Object> tmpmfis = mfis;

    Point rpos;
    double angle = (ctda[1] / 180) * PI; //angle in radian
    double rpx = ctda[0] * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
    double rpy = ctda[0] * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
    rpos.set(rpx, rpy);
    cout << endl << "v" << 1 + 1 << " dis(v12) " << ctda[0] << " angle(v12) " << ctda[1] << endl;
    cout << "robot position id-";
    rpos.display();

    vector<Object> po_into_cv = xformPObjectsIntoCV(po_pv, rpos, angle);
    cout << "pv into cv (potential objects)" << endl;
    //for thesis
    char viewFileName[80];
    sprintf(viewFileName, "%s%d%s", "Maps/PTSandCTS-", viewNumber, ".png");
    if(viewNumber == 47) {
        plotObjects("Maps/PT-MFIS.png",myrobot.getRobot(),mfis,878);
    plotObjectsOf3Kinds(viewFileName,myrobot.getRobot(),po_into_cv,pobjects_cv);
    //waitHere();
    }
    //displayObjects(po_into_cv);

    //    if (viewNumber == 146) {
    //        vector<Object> view1 = findTargetObjects(readASCII("bin/surfaces-145"));
    //        vector<Object> view2 = findTargetObjects(readASCII("bin/surfaces-146"));
    //        view1 = xformPObjectsIntoCV(view1,rpos,angle);
    //        plotObjects("Maps/TOFromTwoViews.png", view1,view2); 
    //    }

    vector<Object> pview_into_cv = xformPObjectsIntoCV(mfis, rpos, angle);


    vector<Object> tmp_po_for_mfis;
    for (int i = 0; i<int(po_into_cv.size()); i++) {
        tmp_po_for_mfis.push_back(getObject(mfis, po_into_cv[i].getID()));
    }

    //vector<Object> pobjects_cv = findTargetObjects(cv);
    cout << "cv's potential objects" << endl;
    //    displayObjects(pobjects_cv);
    char mfisFileName[100];

//    if (viewNumber > 229) {
//        sprintf(mfisFileName, "%s%d%s", "Maps/PVAndCV-", viewNumber, ".png");
//        plotObjects(mfisFileName, po_into_cv, pobjects_cv);
//        //waitHere();
//    }

    //    if(viewNumber == 19)


    //find pobjects in current view
    cout << "********looking for potential objects " << endl;
    double angleth, p1d, p2d;
    double ath = 5;
    double dth = 400;
    vector<Minfo> reflines;

    vector<int> l2binserted;
    Object rmfis, rcv;
    //int lid=mfis[mfis.size()-1].getID();
    vector<int> new_pobjects;

    do {
        for (int i = 0; i<int(pobjects_cv.size()); i++) {
            bool new_po = true;
            for (int j = 0; j<int(po_into_cv.size()); j++) {
                angleth = abs(pobjects_cv[i].getAngleWithLine(po_into_cv[j]));

                if (angleth >= 345)
                    angleth = 360 - angleth;

                p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);
                //if(
                cout<<"pv id "<<po_into_cv[j].getID()<<" cv id "<<pobjects_cv[i].getID();
                cout<<" angle "<<angleth<<"p1d "<<p1d<<" p2d "<<p2d<<endl;

                if (angleth < ath) {
                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);



                    int key_point = po_into_cv[j].getKP();
                    //                    int key_point = pobjects_cv[i].getKP();
                    if (key_point == 1) {
                        if (p1d < dth) {
                            cout << po_into_cv[j].getID() << " cv id " << pobjects_cv[i].getID();
                            cout << " angle " << angleth << " p1d " << p1d << " p2d " << p2d << endl;
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                            refline.setDistFromRobot(fdist);
                            double fitness;
                            if (fdist < 2000.0) {
                                fdist = 2000.0;
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
                                // fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
                                //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
                                //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                            } else {
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
                                // fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
                                //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
                            }
                            refline.setDist(fitness);
                            refline.setAngleDiff(angleth);
                            refline.setLength(pobjects_cv[i].length());
                            refline.setDistDiff(p1d);

                            //check for double match
                            if (reflines.size() > 0) {
                                int pid = refline.getPID();
                                int cid = refline.getCID();
                                bool newmatch = true;
                                for (int k = 0; k<int(reflines.size()); k++) {
                                    if (reflines[k].getPID() == pid || reflines[k].getCID() == cid) {
                                        if (reflines[k].getDist() < refline.getDist()) {
                                            reflines[k].setPID(pid);
                                            reflines[k].setCID(cid);
                                            reflines[k].setDist(fitness);
                                            reflines[k].setRP(1);
                                        }
                                        newmatch = false;
                                    }
                                }
                                if (newmatch == true) {
                                    reflines.push_back(refline);
                                }
                            } else //for first one
                                reflines.push_back(refline);


                            cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 1 << " fitness: " << fitness << endl;

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                        }
                    }
                    if (key_point == 2) {
                        if (p2d < dth) {
                            cout << po_into_cv[j].getID() << " cv id " << pobjects_cv[i].getID();
                            cout << " angle " << angleth << " p1d " << p1d << " p2d " << p2d << endl;
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 2);
                            double fdist = pobjects_cv[i].distP2ToPoint(0, 0);
                            double fitness;
                            refline.setDistFromRobot(fdist);
                            if (fdist < 2000.0) {
                                fdist = 2000.0;
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
                                // fitness = ( pobjects_cv[i].length() /(fdist*p2d*angleth));
                                //fitness = ((dth - p2d)*(ath - angleth)*(1 / fdist));
                            } else {
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
                                //fitness = ( pobjects_cv[i].length() /(fdist*p2d*angleth));
                                //fitness = ((dth - p2d)*(ath - angleth)*(1 / fdist));
                            }
                            refline.setDist(fitness);
                            refline.setAngleDiff(angleth);
                            refline.setLength(pobjects_cv[i].length());
                            refline.setDistDiff(p2d);
                            //check for double match
                            if (reflines.size() > 0) {
                                int pid = refline.getPID();
                                int cid = refline.getCID();
                                bool newmatch = true;
                                for (int k = 0; k<int(reflines.size()); k++) {
                                    if (reflines[k].getPID() == pid || reflines[k].getCID() == cid) {
                                        if (reflines[k].getDist() < refline.getDist()) {
                                            reflines[k].setPID(pid);
                                            reflines[k].setCID(cid);
                                            reflines[k].setDist(fitness);
                                            reflines[k].setRP(2);
                                        }
                                        newmatch = false;
                                    }
                                }
                                if (newmatch == true) {
                                    reflines.push_back(refline);
                                }
                            } else //for first one
                                reflines.push_back(refline);

                            //reflines.push_back(refline);
                            cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 2 << " fitness: " << fitness << endl;

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                        }
                    }

                    if (key_point == 3) {
                        if (p1d < dth || p2d < dth) {
                            cout << po_into_cv[j].getID() << " cv id " << pobjects_cv[i].getID();
                            cout << " angle " << angleth << " p1d " << p1d << " p2d " << p2d << endl;
                            if (p1d < p2d) {
                                Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                                double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                                double fitness;
                                refline.setDistFromRobot(fdist);
                                if (fdist < 2000.0) {
                                    fdist = 2000.0;
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
                                    // fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
                                    //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
                                    //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                                } else {
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
                                    //fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
                                    //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
                                }
                                refline.setDist(fitness);
                                refline.setAngleDiff(angleth);
                                refline.setLength(pobjects_cv[i].length());
                                refline.setDistDiff(p1d);

                                //check for double match
                                if (reflines.size() > 0) {
                                    int pid = refline.getPID();
                                    int cid = refline.getCID();
                                    bool newmatch = true;
                                    for (int k = 0; k<int(reflines.size()); k++) {
                                        if (reflines[k].getPID() == pid || reflines[k].getCID() == cid) {
                                            if (reflines[k].getDist() < refline.getDist()) {
                                                reflines[k].setPID(pid);
                                                reflines[k].setCID(cid);
                                                reflines[k].setDist(fitness);
                                                reflines[k].setRP(1);
                                            }
                                            newmatch = false;
                                        }
                                    }
                                    if (newmatch == true) {
                                        reflines.push_back(refline);
                                    }
                                } else //for first one
                                    reflines.push_back(refline);

                                //reflines.push_back(refline);
                                cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 1 << " fitness: " << fitness << endl;
                            } else {
                                //if(po_into_cv[j].getID() == 144)
                                //	break;
                                Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 2);
                                double fdist = pobjects_cv[i].distP2ToPoint(0, 0);
                                double fitness;
                                refline.setDistFromRobot(fdist);
                                if (fdist < 2000.0) {
                                    fdist = 2000.0;
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
                                    //fitness = ( pobjects_cv[i].length() /(fdist*p2d*angleth));
                                    //fitness = ((dth - p2d)*(ath - angleth)*(1 / fdist));
                                } else {
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
                                    //fitness = ( pobjects_cv[i].length() /(fdist*p2d*angleth));
                                    //fitness = ((dth - p2d)*(ath - angleth)*(1 / fdist));
                                }
                                refline.setDist(fitness);
                                refline.setAngleDiff(angleth);
                                refline.setLength(pobjects_cv[i].length());
                                refline.setDistDiff(p2d);

                                //check for double match
                                if (reflines.size() > 0) {
                                    int pid = refline.getPID();
                                    int cid = refline.getCID();
                                    bool newmatch = true;
                                    for (int k = 0; k<int(reflines.size()); k++) {
                                        if (reflines[k].getPID() == pid || reflines[k].getCID() == cid) {
                                            if (reflines[k].getDist() < refline.getDist()) {
                                                reflines[k].setPID(pid);
                                                reflines[k].setCID(cid);
                                                reflines[k].setDist(fitness);
                                                reflines[k].setRP(2);
                                            }
                                            newmatch = false;
                                        }
                                    }
                                    if (newmatch == true) {
                                        reflines.push_back(refline);
                                    }
                                } else //for first one
                                    reflines.push_back(refline);

                                //reflines.push_back(refline);
                                cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 2 << " fitness: " << fitness << endl;
                            }

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                            new_po = false;

                        }//if p1d or p2d
                    }//if keypoint 3
                }//if angleth
            }//for j

            //            if (new_po == true && pobjects_cv[i].distP1ToPoint(0, 0) > 3000) {//bug
            //                new_pobjects.push_back(pobjects_cv[i].getID());
            //            }

        }

        //resetting angle and distance threshold
        if (reflines.size() < 1) {
            /*ath=ath+2;
            if(ath > 10) {
                    ath=10;
                    dth=dth+100;
            }
    }
    if(ath == 10 && dth == 800){*/
            cout << "no reference found" << endl;
            sprintf(mfisFileName, "%s%d%s", "Maps/PVnCV-no-ref-found-", viewNumber, ".png");
            plotObjects(mfisFileName, po_into_cv, pobjects_cv);
            
//            if((viewNumber - lastUS) == 1 && ath < 9.0) {//going to be lost situation. make condition flexible
//                cout<<"Changing threshold once "<<endl;
//                ath = ath * 2.0;
//                dth = dth * 2.0;
//            } else {
//                cout<<"ath "<<ath<<" dth "<<dth<<" still couldn't find ref."<<endl;
                return output;
//            }
        }
    } while (reflines.size() < 1);
    
    //preparing potential objects for next step
    vector<Object> targetObjectsForNextStep;
    for (int i = 0; i<int(reflines.size()); i++) {
        Object s = getObject(pobjects_cv, reflines[i].getCID());
        //s.display();
        s.setID(reflines[i].getPID());
        //s.setKP(getObject(po_pv, reflines[i].getPID()).getKP());
        s.setKP(reflines[i].getRP());
        targetObjectsForNextStep.push_back(s);
    }
    cout<<"Num of recognized target: "<<reflines.size()<<endl;
    //waitHere();


    //ploting all robot position for aaai paper
    //finding deviation 
    //setting low fitness value which localizes far away
    vector<vector< Object> > allRPosition4ThisStep;
    vector<Object> oneRPosition4ThisStep;
    Object refFromMFIS, refFromCV;
    Object surfFromMFIS, surfFromCV, surfFromCVinMFIS;
    double angleDistortion, distDistortion;
    double totalAngleDistortion = 0, totalDistDistortion = 0, totalDistortion = 0;
    double goodness = 0;
    double distFromRobot = 0;
    double meanX = 0.0, meanY = 0.0;
    //    for(unsigned int i=0; i<reflines.size();i++) {
    //        refFromMFIS.set(getObject(mfis, reflines[i].getPID()));
    //        refFromCV.set(getObject(pobjects_cv, reflines[i].getCID()));
    //        totalDistortion = 0;
    //        goodness = 0;
    //        for(unsigned int j=0; j<reflines.size();j++) {
    //            if(j != i) {
    //                surfFromMFIS.set(getObject(mfis, reflines[j].getPID()));
    //                surfFromCV.set(getObject(pobjects_cv, reflines[j].getCID()));
    //                surfFromCVinMFIS = remakeLineP2(refFromMFIS,refFromCV,surfFromCV,1,0, reflines[i].getRP());
    //                angleDistortion = abs(surfFromMFIS.getAngleWithLine(surfFromCVinMFIS));
    //                if(angleDistortion > 340)
    //                    angleDistortion = 360 - angleDistortion;
    //                if(reflines[j].getRP() == 1) {
    //                    distDistortion = surfFromMFIS.distP1ToP1(surfFromCVinMFIS);
    //                } else
    //                    distDistortion = surfFromMFIS.distP2ToP2(surfFromCVinMFIS);
    //                
    ////                if(distFromRobot < 2000)
    ////            distFromRobot = 2000.0;
    ////                if(reflines[j].getRP() == 1)
    ////             distFromRobot = surfFromCV.distP1ToPoint(0,0);
    ////         else
    ////             distFromRobot = surfFromCV.distP2ToPoint(0,0);
    ////                goodness +=surfFromCV.length() * (1.0/( angleDistortion* distDistortion*distFromRobot));
    //                totalDistortion += (distDistortion * angleDistortion);
    ////                if(reflines[i].getRP() == 1)
    ////             distFromRobot = refFromCV.distP1ToPoint(0,0);
    ////         else
    ////             distFromRobot = refFromCV.distP2ToPoint(0,0);
    ////         
    ////                goodness +=refFromCV.length() * (1.0/( angleDistortion* distDistortion*distFromRobot));
    //            }
    //        }
    //         if(reflines[i].getRP() == 1)
    //             distFromRobot = refFromCV.distP1ToPoint(0,0);
    //         else
    //             distFromRobot = refFromCV.distP2ToPoiif (viewNumber > 1) {
//        sprintf(mfisFileName, "%s%d%s", "Maps/PVAndCV-", viewNumber, ".png");
//        plotObjects(mfisFileName, po_into_cv, pobjects_cv,reflines.back().getCID());
//        //waitHere();
//    }nt(0,0);
    ////         
    //////        if(distFromRobot < 2000)
    //////            distFromRobot = 2000.0;
    ////        //fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
    //        goodness  = refFromCV.length() *(1.0/ (distFromRobot *totalDistortion));
    //       // goodness  += refFromCV.length()  *(1.0/ distFromRobot);
    //        reflines[i].setDist(goodness);
    //    }
    for (unsigned int i = 0; i < reflines.size(); i++) {
        refFromMFIS.set(getObject(mfis, reflines[i].getPID()));
        refFromCV.set(getObject(pobjects_cv, reflines[i].getCID()));
        oneRPosition4ThisStep = myrobot.inMFIS(refFromMFIS, refFromCV, reflines[i].getRP());
        meanX += oneRPosition4ThisStep[6].X1();
        meanY += oneRPosition4ThisStep[6].Y1();
        refFromMFIS.setKP(reflines[i].getRP());
        refFromCV.setKP(reflines[i].getRP());
        oneRPosition4ThisStep.push_back(refFromMFIS);
        oneRPosition4ThisStep.push_back(refFromCV);
        allRPosition4ThisStep.push_back(oneRPosition4ThisStep);
    }
    meanX = meanX / reflines.size();
    meanY = meanY / reflines.size();
    double distFromMean = 0.0;
    double variance = 0.0;
    for (unsigned int i = 0; i < allRPosition4ThisStep.size(); i++) {//calculate dist from mean
        distFromMean = allRPosition4ThisStep[i][6].distP1ToPoint(meanX, meanY);
        reflines[i].setDistFromMean(distFromMean);
        variance += distFromMean * distFromMean;
    }
    variance = variance / reflines.size();
    double standardDeviation = sqrt(variance);
    cout << "Standard Deviation: " << standardDeviation << endl;
    for(unsigned int i=0;i<reflines.size();i++) {
        if((reflines[i].getDistFromMean() - standardDeviation) > 0.001) {
            reflines[i].setDist(0.0);//reducing fitness for far away localization
        }
    }

    //finding ref using mean of robot position
//    if (reflines.size() > 2) {
//
//        Object meanRobotPoseInPM, cvRobotPosition;
//        meanRobotPoseInPM.setP1(meanX, meanY);
//        double meanX4P1 = 0.0;
//        double meanY4P1 = 0.0;
//        double meanX4P2 = 0.0;
//        double meanY4P2 = 0.0;
//        for (unsigned int i = 0; i < reflines.size(); i++) {
//            if ((reflines[i].getDistFromMean() - standardDeviation) > 0.001) {//if we use reflines[i].getDistFromMean() > standardDeviation. then we get sometimes true, sometimes false
//                //reflines[i].setDist(0.0); //reducing fitness for far away localization
//                reflines[i].setGoodnessValue(0.0);
//            }
//        }
//        for (unsigned int i = 0; i < reflines.size(); i++) {
//            if (reflines[i].getGoodnessValue() > 0.0) {
//                meanX4P1 = allRPosition4ThisStep[i][6].X1();
//                meanY4P1 = allRPosition4ThisStep[i][6].Y1();
//                meanX4P2 = allRPosition4ThisStep[i][6].X2();
//                meanY4P2 = allRPosition4ThisStep[i][6].Y2();
//
//            }
//        }
//        meanX4P1 = meanX4P1 / reflines.size();
//        meanY4P1 = meanY4P1 / reflines.size();
//        meanX4P2 = meanX4P2 / reflines.size();
//        meanY4P2 = meanY4P2 / reflines.size();
//        meanRobotPoseInPM.set(meanX4P1, meanY4P1, meanX4P2, meanY4P2, 1);
//        cvRobotPosition.set(0, 0, 0, 250, 1);
//        meanRobotPoseInPM.setKP(1);
//        cvRobotPosition.setKP(1);
//
//        if(meanX4P1 != 0.0) {
//        output.setTargetObjects(targetObjectsForNextStep);
//        vector<Object> refObjects;
//        refObjects.push_back(meanRobotPoseInPM);
//        refObjects.push_back(cvRobotPosition);
//        output.setReferenceObjects(refObjects);
//        output.setViews(allRPosition4ThisStep);
//        //    if(viewNumber == 260)
//        //       plotObjects("Maps/1reference.png",robloc,);
//        return output;
//        }
//    }
    
//        std::sort(reflines.begin(), reflines.end(), DataSortMethod);
//        if (reflines.back().getDist() < 80) {
//            cout << "Returning Null" << endl;
//            output.setViews(allRPosition4ThisStep);
//            return output;
//        } else {
//            Object refObjectFromMFIS, refObjectFromCV;
//            cout << "Found a suitable ref:)" << endl;
//            //waitHere();
//            refObjectFromMFIS.set(getObject(mfis, reflines.back().getPID()));
//            if (refObjectFromMFIS.X1() == 0 && refObjectFromMFIS.X2() == 0) //means ref isn't found in MFIS
//                return output;
//            refObjectFromMFIS.setKP(reflines.back().getRP());
//            refObjectFromCV.set(getObject(pobjects_cv, reflines.back().getCID()));
//            //  }
//
//            refObjectFromMFIS.display();
//            refObjectFromCV.display();
//
//            //waitHere();
//
//            output.setTargetObjects(targetObjectsForNextStep);
//            vector<Object> refObjects;
//            refObjects.push_back(refObjectFromMFIS);
//            refObjects.push_back(refObjectFromCV);
//            output.setReferenceObjects(refObjects);
//            output.setViews(allRPosition4ThisStep);
//            //    if(viewNumber == 260)
//            //       plotObjects("Maps/1reference.png",robloc,);
//            return output;
//        }
    


    //setting final goodness value
//    for (unsigned int i = 0; i < reflines.size(); i++) {
//        if ((reflines[i].getDistFromMean() - standardDeviation) > 0.001) {//if we use reflines[i].getDistFromMean() > standardDeviation. then we get sometimes true, sometimes false
//            //reflines[i].setDist(0.0); //reducing fitness for far away localization
//            reflines[i].setGoodnessValue(0.0);
//        } else {
//            refFromMFIS.set(getObject(mfis, reflines[i].getPID()));
//            refFromCV.set(getObject(pobjects_cv, reflines[i].getCID()));
//            surfFromCVinMFIS = remakeLineP2(refFromMFIS, refFromCV, refFromCV, 1, 0, reflines[i].getRP());
//            //            totalDistortion = 0;
//            //            goodness = 0;
//            //            for (unsigned int j = 0; j < reflines.size(); j++) {
//            //                if (j != i) {
//            //                    surfFromMFIS.set(getObject(mfis, reflines[j].getPID()));
//            //                    surfFromCV.set(getObject(pobjects_cv, reflines[j].getCID()));
//            //                    surfFromCVinMFIS = remakeLineP2(refFromMFIS, refFromCV, surfFromCV, 1, 0, reflines[i].getRP());
//            //                    angleDistortion = abs(surfFromMFIS.getAngleWithLine(surfFromCVinMFIS));
//            //                    if (angleDistortion > 340)
//            //                        angleDistortion = 360 - angleDistortion;
//            //                    if (reflines[j].getRP() == 1) {
//            //                        distDistortion = surfFromMFIS.distP1ToP1(surfFromCVinMFIS);
//            //                    } else
//            //                        distDistortion = surfFromMFIS.distP2ToP2(surfFromCVinMFIS);
//            //                    
//            ////fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1.0 / fdist));
//            //                    totalDistortion += (500.0 - distDistortion) * (10.0 - angleDistortion);
//            //
//            //                }
//            //            }
//            if (reflines[i].getRP() == 1)
//                distFromRobot = refFromCV.distP1ToPoint(0, 0);
//            else
//                distFromRobot = refFromCV.distP2ToPoint(0, 0);
//            //
//            //            if(totalDistortion == 0.0) //
//            //                goodness = 0.0;
//            //            else {
//            //                //goodness = refFromCV.length() * (1.0 / (distFromRobot * totalDistortion));
//            //                goodness = totalDistortion;
//            //            }
//
//            angleDistortion = abs(refFromMFIS.getAngleWithLine(surfFromCVinMFIS));
//            if (angleDistortion > 340)
//                angleDistortion = 360 - angleDistortion;
//            if (reflines[i].getRP() == 1) {
//                distDistortion = refFromMFIS.distP1ToP1(surfFromCVinMFIS);
//            } else
//                distDistortion = refFromMFIS.distP2ToP2(surfFromCVinMFIS);
//            goodness = refFromCV.length() * (500.0 - distDistortion) * (10.0 - angleDistortion)* (1.0 / (distFromRobot));
//            //goodness = (500.0 - distDistortion) * (10.0 - angleDistortion)* (1.0 / (distFromRobot));
//            //goodness = refFromCV.length() *  (500.0 - distDistortion) * (10.0 - angleDistortion);
//            if (goodness < 0) {
//                cout << "Bad goodness value bcz of dist: " << (500.0 - distDistortion) << " angle: " << (10.0 - angleDistortion) << endl;
//                waitHere();
//            }
//
//            reflines[i].setGoodnessValue(goodness);
//        }
//    }
    cout << "View No. " << viewNumber << " No of ref: " << allRPosition4ThisStep.size() << endl;
    //plotObjects("Maps/test.png",allRPosition4ThisStep,mfis);
    //waitHere();
    cout << "Target object for Next Step: " << endl;
    //displayObjects(targetObjectsForNextStep);
    //std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    //std::sort(reflines.begin(), reflines.end(),SortBasedOnAngleDiff);
    //std::sort(reflines.begin(), reflines.end(), SortBasedOnLength);
    //std::sort(reflines.begin(), reflines.end(), SortBasedOnDistDiff);
    cout << "sorted target Objects" << endl;
    //displayMinfo(reflines);
    //waitHere();
    //    if (reflines.back().getDist() < 80 )//or viewNumber == 149)//means very bad reference object
    //        return output;
    //    if(reflines.back().getDist() < 120) {
    //        cout<<"fittness: "<<reflines.back().getDist()<<endl;
    //        waitHere();
    //    }


    Object refObjectFromMFIS, refObjectFromCV;
    cout << endl << "********** Getting Reference Objects*************** " << endl;
    //    bool gapRefFound = false;
    //    //std::sort(reflines.begin(), reflines.end(), SortBasedOnDistDiff);
    //    std::sort(reflines.begin(), reflines.end(), SortBasedOnDistFromRobot);
    //    if (reflines.size() > 1) {//constructing ref surfaces from two occuluding points
    //        cout<<"From gap "<<endl;
    //        Object firstObject,secondObj;
    //        //get MFIS ref
    //        firstObject.set(getObject(mfis, reflines[0].getPID()));
    //        if(reflines[0].getRP() == 1) {//setting first point
    //            refObjectFromMFIS.setP1(firstObject.X1(),firstObject.Y1());
    //            firstObject.set(getObject(pobjects_cv, reflines[0].getCID()));
    //            refObjectFromCV.setP1(firstObject.X1(),firstObject.Y1());//for cv
    //        } else {
    //            refObjectFromMFIS.setP1(firstObject.X2(),firstObject.Y2());
    //            firstObject.set(getObject(pobjects_cv, reflines[0].getCID()));
    //            refObjectFromCV.setP1(firstObject.X2(),firstObject.Y2());//for cv
    //        }
    //        refObjectFromMFIS.setKP(1);
    //        refObjectFromCV.setKP(1);
    //        for(unsigned int i=1;i<reflines.size();i++) {//searching for suitable second point
    //                secondObj.set(getObject(mfis, reflines[i].getPID()));
    //                if(reflines[i].getRP() == 1) {
    //                    if(refObjectFromMFIS.distP1ToP1(secondObj) > 500.0) {//two ref could have same point
    //                        refObjectFromMFIS.setP2(secondObj.X1(),secondObj.Y1());
    //                        secondObj.set(getObject(pobjects_cv, reflines[i].getCID()));//for cv
    //                        refObjectFromCV.setP2(secondObj.X1(),secondObj.Y1());
    //                        gapRefFound = true;
    //                        refObjectFromCV.setColorID(100);
    //                        break;
    //                    }
    //                } else {
    //                    if(refObjectFromMFIS.distP1ToP2(secondObj) > 500.0) {
    //                        refObjectFromMFIS.setP2(secondObj.X2(),secondObj.Y2());
    //                        secondObj.set(getObject(pobjects_cv, reflines[i].getCID()));//for cv
    //                        refObjectFromCV.setP2(secondObj.X2(),secondObj.Y2());
    //                        gapRefFound = true;
    //                        refObjectFromCV.setColorID(100);
    //                        break;
    //                    }
    //                }
    //        }        
    //    }
    //    
    // gapRefFound = false;
    //if(gapRefFound == false) {//coudln't find two suitable points so use one surface
    cout << "view - " << viewNumber << " From surface " << endl;
    std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    //std::sort(reflines.begin(), reflines.end(), SortBasedOnGoodnessValue);
    displayMinfo(reflines);
    //        if(viewNumber > 145) {
    //            cout<<viewNumber<<endl;
    //            waitHere();
    //        }

        //if (reflines.back().getDist() < 80  or //for four figues for Yeap (contineous mapping)
               // viewNumber == 157 or (viewNumber > 200 && viewNumber < 220) or (viewNumber > 280 && viewNumber < 302) ){//or (viewNumber > 58 && viewNumber < 80) or (viewNumber > 983 && viewNumber < 991) or (viewNumber > 1000 && viewNumber < 1067)) {//or viewNumber == 149)//means very bad reference object
        //if (reflines.back().getDist() < 80  ){//or (viewNumber > 58 && viewNumber < 80) or (viewNumber > 983 && viewNumber < 991) or (viewNumber > 1000 && viewNumber < 1067)) {//or viewNumber == 149)//means very bad reference object    
        //if (reflines.back().getDist() < 80  or (viewNumber > 219 && viewNumber < 261) ){//for set601
         if (reflines.back().getDist() < 80  ){//for set600
            cout << "Returning Null" << endl;
            output.setViews(allRPosition4ThisStep);
            return output;
        }
//    if (reflines.back().getGoodnessValue() == 0.0 or reflines.size() < 3) {//or viewNumber == 149)//means very bad reference object
//        //then look how is normal fitness with previous view
//        std::sort(reflines.begin(), reflines.end(), DataSortMethod);
//        if (reflines.back().getDist() < 80) {
//            cout << "Returning Null" << endl;
//            output.setViews(allRPosition4ThisStep);
//            return output;
//        }
//        //waitHere();        
//    }
    //          std::sort(reflines.begin(), reflines.end(), SortBasedOnLength);
    //          std::sort(reflines.begin(), reflines.end(),SortBasedOnAngleDiff);
    //            std::sort(reflines.begin(), reflines.end(), SortBasedOnDistDiff);
    //        std::sort(reflines.begin(), reflines.end(), SortBasedOnDistFromMean);
    //        if(reflines.size() == 2)
    //                std::sort(reflines.begin(), reflines.end(),SortBasedOnAngleDiff);

    cout << "Found a suitable ref:)" << endl;
    //waitHere();
    refObjectFromMFIS.set(getObject(mfis, reflines.back().getPID()));
    if (refObjectFromMFIS.X1() == 0 && refObjectFromMFIS.X2() == 0) //means ref isn't found in MFIS
        return output;
    refObjectFromMFIS.setKP(reflines.back().getRP());
    refObjectFromCV.set(getObject(pobjects_cv, reflines.back().getCID()));
    //  }

    refObjectFromMFIS.display();
    refObjectFromCV.display();
    
//    if (viewNumber > 229 && viewNumber < 300) {
//        sprintf(mfisFileName, "%s%d%s", "Maps/PVAndCV-", viewNumber, ".png");
//        plotObjects(mfisFileName, po_into_cv, pobjects_cv,reflines.back().getCID());
//        //waitHere();
//    }

    //waitHere();

    output.setTargetObjects(targetObjectsForNextStep);
    vector<Object> refObjects;
    refObjects.push_back(refObjectFromMFIS);
    refObjects.push_back(refObjectFromCV);
    output.setReferenceObjects(refObjects);
    output.setViews(allRPosition4ThisStep);
    //    if(viewNumber == 260)
    //       plotObjects("Maps/1reference.png",robloc,);
    cout<<"Num of targetObjectsForNextStep: "<<reflines.size()<<endl;
    //waitHere();
    
    
    
    return output;
}




vector<Minfo> recognizeObjects(vector<Object> newASRObjects, vector<Object> oldASRObjectsOnNew) {

    vector<Object> pobjects_cv = newASRObjects;
    vector<Object> po_into_cv = oldASRObjectsOnNew;
    double angleth, p1d, p2d;
    double ath = 10;
    double dth = 700;
    vector<Minfo> reflines;


    do {
        for (int i = 0; i<int(pobjects_cv.size()); i++) {

            for (int j = 0; j<int(po_into_cv.size()); j++) {
                angleth = abs(pobjects_cv[i].getAngleWithLine(po_into_cv[j]));


                if (angleth >= 345)
                    angleth = 360 - angleth;

                if (angleth < ath) {

                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);

                    //cout<<"Angle: "<<angleth<<" p1d: "<<p1d<<" p2d: "<<p2d<<endl;


                    double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                    double fitness;
                    //fitness based recognition
                    if (p1d < dth || p2d < dth) {
                        if (p1d < dth)
                            fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                        else
                            fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));

                        if (fitness > 50) {
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                            if (p1d < p2d)
                                refline.setRP(1);
                            else
                                refline.setRP(2);
                            if (p1d < dth && p2d < dth)
                                refline.setRP(3);


                            refline.setDist(fitness);
                            reflines.push_back(refline);
                        }
                    }


                }//if angleth
            }//for j


        }

        //resetting angle and distance threshold
        if (reflines.size() < 1) {
            cout << "no reference found" << endl;
            return reflines;
        }
    } while (reflines.size() < 1);

    std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    cout << "sorted target Objects" << endl;
    displayMinfo(reflines);

    return reflines;
}

vector<Minfo> recognizeObjectsFrom2ConsViews(vector<Object> newASRObjects, vector<Object> oldASRObjectsOnNew) {

    vector<Object> pobjects_cv = newASRObjects;
    vector<Object> po_into_cv = oldASRObjectsOnNew;
    double angleth, p1d, p2d, shortestDist;
    double ath = 5;
    double dth = 400;
    vector<Minfo> reflines;


    do {
        for (int i = 0; i<int(pobjects_cv.size()); i++) {

            for (int j = 0; j<int(po_into_cv.size()); j++) {
                angleth = abs(pobjects_cv[i].getAngleWithLine(po_into_cv[j]));

                if (angleth >= 345)
                    angleth = 360 - angleth;

                if (angleth < ath) {
                    //                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                    //                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);
                    shortestDist = shortestDistanceBtwTwoObjects(pobjects_cv[i], po_into_cv[j]);
                    if (shortestDist < dth) {

                        double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                        double fitness;
                        //fitness based recognition
                        //                    if(p1d < dth)
                        fitness = (pobjects_cv[i].length()*(dth - shortestDist)*(ath - angleth)*(1 / fdist));
                        //                    else
                        //                        fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));

                        //                    if(fitness > 50) {
                        Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                        refline.setDist(fitness);
                        reflines.push_back(refline);
                        //                    }
                    }


                }//if angleth
            }//for j


        }

        //resetting angle and distance threshold
        if (reflines.size() < 1) {
            cout << "no reference found" << endl;
            return reflines;
        }
    } while (reflines.size() < 1);

    std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    cout << "sorted target Objects" << endl;
    displayMinfo(reflines);

    return reflines;
}

vector<Object> discardObjectsPartBehindRobot(vector<Object> previousViewOnCV, vector<double> distAngle) {
    MyRobot myrobot(0, 0);
    vector<Object> currentRobotPosition = myrobot.getRobot();
    //    Object aLine = makeLineAtTwoPointsWithObject(distAngle[1], distAngle[0], distAngle[1], distAngle[0] + 500, previousRobotPosition[6], 1);
    //    vector<Object> currentRobotPosition = myrobot.inMFIS(aLine, previousRobotPosition[6], 1);
    vector<Object> output;
    vector<double> intersectingPoint;
    for (unsigned int i = 0; i < previousViewOnCV.size(); i++) {
        if (previousViewOnCV[i].Y1() < 0 && previousViewOnCV[i].Y2() > 0) {//object on left side
            intersectingPoint = getIntersectionPoint(currentRobotPosition[7], previousViewOnCV[i]);
            cout << "Need to discard object " << i << endl;
            cout << "intersection with x-axis " << intersectingPoint[0] << " " << intersectingPoint[1] << endl;
            previousViewOnCV[i].setP1(intersectingPoint[0], intersectingPoint[1]);
            output.push_back(previousViewOnCV[i]);
        } else if (previousViewOnCV[i].Y1() > 0 && previousViewOnCV[i].Y2() < 0) {//object on right side
            cout << "Need to discard object " << i << endl;
            intersectingPoint = getIntersectionPoint(currentRobotPosition[7], previousViewOnCV[i]);
            cout << "intersection with x-axis " << intersectingPoint[0] << " " << intersectingPoint[1] << endl;
            previousViewOnCV[i].setP2(intersectingPoint[0], intersectingPoint[1]);
            output.push_back(previousViewOnCV[i]);
        } else if (previousViewOnCV[i].Y1() > 0 && previousViewOnCV[i].Y2() > 0) {//objects in front of robot
            output.push_back(previousViewOnCV[i]);
        }
    }
    return output;
}

vector<Object> shapeBasedViewRecognition(vector<Object> previousView, vector<Object> currentView, vector<double> distAngle) {

    //Finding robot's position in x,y plane
    Point rpos;
    double angle = (distAngle[1] / 180) * PI; //angle in radian
    double rpx = distAngle[0] * sin(-angle);
    double rpy = distAngle[0] * cos(-angle);
    rpos.set(rpx, rpy);
    cout << endl << "v" << 1 + 1 << " dis(v12) " << distAngle[0] << " angle(v12) " << distAngle[1] << endl;
    cout << "robot position id-";
    rpos.display();

    vector<Object> previousViewOnCV = xformPVIntoCV(previousView, rpos, angle);
    vector<Object> oldASRObjects = discardObjectsPartBehindRobot(previousViewOnCV, distAngle);
    plotObjects("Maps/TwoConsViews.png", currentView, oldASRObjects);
    //    waitHere();


    vector<Object> newASRObjects = currentView;

    vector<Object> oldASRObjectsOnNew;
    vector<Object> probableSameObjects;

    vector<Object> recognizedObjectsFromOldASR;
    vector<Object> recognizedObjectsFromCV;

    vector<Minfo> recognizedObjects;
    char mfisFileName[80];

    Object oldObject, newObject;

    for (unsigned int i = 0; i < oldASRObjects.size(); i++) {
        if (oldASRObjects[i].length() > 400) {
            oldObject = oldASRObjects[i];
        }
        for (unsigned int j = 0; j < newASRObjects.size(); j++) {
            if (newASRObjects[j].length() > 400) {
                newObject = newASRObjects[j];
                oldASRObjectsOnNew.clear();
                for (unsigned int k = 0; k < oldASRObjects.size(); k++) {
                    oldASRObjectsOnNew.push_back(remakeLineP2(newObject, oldObject, oldASRObjects[k], oldASRObjects[k].getID(), 0, 1));
                }

                recognizedObjects = recognizeObjectsFrom2ConsViews(newASRObjects, oldASRObjectsOnNew);
                if (recognizedObjects.size() > 1) {
                    for (int ii = 0; ii<int(recognizedObjects.size()); ii++) {
                        Object s = getObject(newASRObjects, recognizedObjects[ii].getCID());
                        recognizedObjectsFromCV.push_back(s);
                        s = getObject(previousView, recognizedObjects[ii].getPID());
                        recognizedObjectsFromOldASR.push_back(s);

                    }
                    probableSameObjects.clear();
                    probableSameObjects.push_back(oldASRObjectsOnNew[i]);
                    probableSameObjects.push_back(newObject);
                    sprintf(mfisFileName, "%s%d%s%d%s", "Maps/oldOnNewASR-", i, "-", j, ".png");
                    //                    plotObjectsOf3Kinds(mfisFileName, oldASRObjectsOnNew, newASRObjects, probableSameObjects);
                    plotObjects(mfisFileName, recognizedObjectsFromCV, recognizedObjectsFromOldASR);
                    plotObjects("Maps/FromCV.png", recognizedObjectsFromCV, recognizedObjectsFromCV);
                    plotObjects("Maps/FromPV.png", recognizedObjectsFromOldASR, recognizedObjectsFromOldASR);
                    cout << "Recognized Objects: " << endl;
                    displayMinfo(recognizedObjects);
                    waitHere();
                }
                //                cout<<"OldObject ID: "<<i<<" NewObject ID: "<<j<<endl;

            }
        }
    }
    cout << "Old ASR" << endl;
    //displayObjects(oldASRObjects);
    plotObjects("Maps/oldAndNewASRs.png", oldASRObjects, newASRObjects);
    vector<Object> result;
    return result;
}

vector<Object> testingFunction() {
    //    Object mainObject(100,200,2000,3000,1);
    //    Object intersectingObject(1000,1800,2000,2000,2);
    //    vector<Object> tmp1,tmp2;
    //    tmp1.push_back(mainObject);
    //   tmp2.push_back(intersectingObject);
    //    vector<double> intersectingPoint = getIntersectionPoint(mainObject,intersectingObject);
    //    intersectingObject.setP2(intersectingPoint[0],intersectingPoint[1]);
    //     
    //    double angle = mainObject.getAngleWithLine(intersectingObject);
    //    cout<<"Angle: "<<angle<<endl;
    //    Object paralleOfIO=makeLineAtPointWithObject(180+angle,intersectingObject.length(),mainObject);
    //    paralleOfIO.setP1(mainObject.X1(),mainObject.Y1());
    //    tmp2.push_back(paralleOfIO);
    //    plotObjects("MFIS/test.png",tmp1,tmp2);


}

//place recognition based on shape

Transporter recognizeThisPlace(ASRNetwork pm, vector<Object> mfis, vector<Object> currentView, vector<Object> robotPositions, vector<Object> referenceObjects, int viewNumber) {

    vector<Object> allASRObjects;
    vector<Object> oldASRObjects;
    vector<Object> newASRObjects;
    vector<Object> oldASRObjectsOnNew;


    vector<Object> recognizedObjectsFromOldASR;
    vector<Object> recognizedObjectsFromCV;

    vector<Object> refObjectsForLoopClosing;
    vector<Object> refObjects;

    vector<Minfo> recognizedObjects;
    vector<Object> targetObjects;
    char mfisFileName[80];

    int maxRecObjects = 0;

    MyRobot myrobot(0, 0);

    cout << "Number of ASRs: " << pm.getASRs().size() << endl;
    vector<Object> currentViewOnMFIS;
    Object temp;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        temp = remakeLineP2(referenceObjects[0], referenceObjects[1], currentView[i], currentView[i].getID(), 0, referenceObjects[0].getKP());
        currentViewOnMFIS.push_back(temp);
    }

    oldASRObjects = pm.getASRs()[0].getASRObjects();
    newASRObjects = currentViewOnMFIS;
    //newASRObjects = pm.getCurrentASR().getASRObjects();

    //    cout<<"CurrentView"<<endl;
    //    displayObjects(newASRObjects);

    cout << "Old ASR" << endl;
    displayObjects(oldASRObjects);
    plotObjects("Maps/oldAndNewASRs.png", oldASRObjects, newASRObjects);

    //    plotObjects("Maps/OldASR.png", myrobot.getRobot(), oldASRObjects);
    //    plotObjects("Maps/NewASR.png", myrobot.getRobot(), newASRObjects);

    for (unsigned int i = 0; i < 1; i++) {
        allASRObjects = addTwoVectorsOfObjects(allASRObjects, pm.getASRs()[i].getASRObjects());
    }
    // plotObjectsOf3Kinds("Maps/PMat69.png",allASRObjects,currentViewOnMFIS,addTwoVectorsOfObjects(robotPositions,myrobot.getRobot()));

    //  plotObjects("Maps/ASR-1and7.png",oldASRObjects,newASRObjects);
    //waitHere();

    Object oldObject, newObject;



    for (unsigned int i = 1; i < oldASRObjects.size(); i++) {
        if (oldASRObjects[i].length() > 400) {
            oldObject = oldASRObjects[i];

            for (unsigned int j = 0; j < newASRObjects.size(); j++) {
                if (newASRObjects[j].length() > 400) {
                    newObject = newASRObjects[j];
                    oldASRObjectsOnNew.clear();
                    for (unsigned int k = 0; k < oldASRObjects.size(); k++) {
                        oldASRObjectsOnNew.push_back(remakeLineP2(newObject, oldObject, oldASRObjects[k], oldASRObjects[k].getID(), 0, 1));
                    }

                    recognizedObjects = recognizeObjects(newASRObjects, oldASRObjectsOnNew);
                    //to filter out some possibilities we have to use order of appearance of surfaces 
                    //e.g. corresponding recognizedObjectsFromCV and recognizedObjectsFromOldASR should be in same order
                    //for explanation see future works in thesis folder.
                    if (recognizedObjects.size() > 5 && recognizedObjects.size() >= maxRecObjects) {
                        maxRecObjects = recognizedObjects.size();

                        recognizedObjectsFromCV.clear();
                        recognizedObjectsFromOldASR.clear();
                        targetObjects.clear();

                        //extracting recognized objects from old and new ASR
                        for (int ii = 0; ii<int(recognizedObjects.size()); ii++) {
                            Object s = getObject(newASRObjects, recognizedObjects[ii].getCID()); //new ASR as in MFIS
                            recognizedObjectsFromCV.push_back(s);
                            s = getObject(oldASRObjects, recognizedObjects[ii].getPID()); //old ASR object
                            recognizedObjectsFromOldASR.push_back(s);

                        }
                        //reference object to shift old ASR at new position
                        refObjectsForLoopClosing.clear();
                        refObjectsForLoopClosing.push_back(recognizedObjectsFromOldASR.back());
                        refObjectsForLoopClosing.push_back(recognizedObjectsFromCV.back());
                        //refObjectsForLoopClosing.push_back(recognizedObjectsFromOldASR[recognizedObjectsFromOldASR.size()-3]);
                        //refObjectsForLoopClosing.push_back(recognizedObjectsFromCV[recognizedObjectsFromCV.size()-3]);

                        //reference objects for localization. i.e. refObject1 from new old ASR n refObject2 from current view
                        refObjects.clear();
                        refObjects.push_back(getObject(oldASRObjectsOnNew, recognizedObjects[recognizedObjects.size() - 1].getPID()));
                        refObjects.push_back(getObject(currentView, recognizedObjects[recognizedObjects.size() - 1].getCID()));

                        //  if(maxRecObjects > 6) {
                        cout << "Recognized Objects: " << endl;
                        displayMinfo(recognizedObjects);
                        //waitHere();
                        //   }


                        //setting up target objects for next step
                        std::sort(recognizedObjects.begin(), recognizedObjects.end(), sortAsCID);
                        cout << "Sorted Recognized Objects: " << endl;
                        displayMinfo(recognizedObjects);

                        Object s;
                        for (unsigned int ii = 0; ii < recognizedObjects.size() - 1; ii++) {
                            for (unsigned int iii = ii + 1; iii < recognizedObjects.size(); iii++) {
                                if (recognizedObjects[ii].getCID() == recognizedObjects[iii].getCID())
                                    ii = iii;
                            }

                            s = getObject(currentView, recognizedObjects[ii].getCID());
                            s.setKP(recognizedObjects[ii].getRP());
                            s.setID(recognizedObjects[ii].getPID());
                            targetObjects.push_back(s);
                        }



                        //printing recognized shape
                        sprintf(mfisFileName, "%s%d%s%d%s%d%s", "Maps/matched-old-", i, "-", j, "of ", viewNumber, ".png");
                        //                    plotObjectsOf3Kinds(mfisFileName, oldASRObjectsOnNew, newASRObjects, probableSameObjects);
                        plotObjects(mfisFileName, myrobot.getRobot(), recognizedObjectsFromOldASR);

                        sprintf(mfisFileName, "%s%d%s%d%s%d%s", "Maps/matched-new-", i, "-", j, "of ", viewNumber, ".png");
                        //                    plotObjectsOf3Kinds(mfisFileName, oldASRObjectsOnNew, newASRObjects, probableSameObjects);
                        plotObjects(mfisFileName, myrobot.getRobot(), recognizedObjectsFromCV);
                        // plotObjects("Maps/matched-situation.png", currentViewOnMFIS, oldASRObjectsOnNew);
                        // plotObjects("Maps/matched-surfaces.png", recognizedObjectsFromOldASR, recognizedObjectsFromCV);

                        cout << endl << endl << "I know this ASR::::at v-" << viewNumber << " " << i << " " << j << endl;
                        //waitHere();
                    }
                    //                cout<<"OldObject ID: "<<i<<" NewObject ID: "<<j<<endl;

                }
            }
        }
    }

    //    cout<<"Recognized Objects: "<<endl;
    //                    displayMinfo(recognizedObjects);

    //    cout<<"Recognized Objects in CV"<<endl;
    //                    displayObjects(recognizedObjectsFromCV);

    plotObjectsOf3Kinds("Maps/MFISAndCurrentView.png", mfis, newASRObjects, refObjectsForLoopClosing);

    cout << "TargetObject for next step" << endl;
    displayObjects(targetObjects);

    cout << "Reference Objects" << endl;
    displayObjects(refObjectsForLoopClosing);

    //plotObjects("Maps/referenceObjects.png", refObjectsForLoopClosing, myrobot.getRobot());
    cout << "LoopClosing is finished for this step. v: " << viewNumber << endl;
    //waitHere();
    //plotObjects("Maps/MFIS@111.png", myrobot.inMFIS(refObjects[0], refObjects[1], 1), mfis);
    //plotObjects("Maps/MFISBeforeLP.png", oldASRObjects, mfis);
    vector<Object> newOldASR = projectingTheView(pm.getASRs()[0].getASRObjects(), refObjectsForLoopClosing[1], refObjectsForLoopClosing[0], 1);
    plotObjects("Maps/MFISAfterLP.png", mfis, newOldASR);
    cout << "New Old ASR1" << endl;
    displayObjects(newOldASR);

    Transporter package;
    package.setReferenceObjects(refObjects);
    package.setReferenceObjectsForLoopClosing(refObjectsForLoopClosing);
    package.setTargetObjects(targetObjects);
    package.setView(projectingTheView(pm.getASRs()[1].getASRObjects(), refObjectsForLoopClosing[1], refObjectsForLoopClosing[0], 1));
    package.setMFIS(mfis);
    //waitHere();
    return package;
}

void recognizeViews(vector<Object> previousView, vector<Object> currentView) {

    vector<Object> oldASRObjects;
    vector<Object> newASRObjects;
    vector<Object> oldASRObjectsOnNew;
    vector<Object> probableSameObjects;

    vector<Object> recognizedObjectsFromOldASR;
    vector<Object> recognizedObjectsFromCV;

    vector<Minfo> recognizedObjects;
    char mfisFileName[80];

    oldASRObjects = previousView;
    newASRObjects = currentView; //pm.getASRs()[6].getASRObjects();

    Object oldObject, newObject;

    for (unsigned int i = 1; i < oldASRObjects.size(); i++) {
        if (oldASRObjects[i].length() > 400) {
            oldObject = oldASRObjects[i];

            for (unsigned int j = 0; j < newASRObjects.size(); j++) {
                if (newASRObjects[j].length() > 400) {
                    newObject = newASRObjects[j];
                    oldASRObjectsOnNew.clear();
                    for (unsigned int k = 0; k < oldASRObjects.size(); k++) {
                        oldASRObjectsOnNew.push_back(remakeLineP2(newObject, oldObject, oldASRObjects[k], oldASRObjects[k].getID(), 0, 1));
                    }
                    sprintf(mfisFileName, "%s%d%s%d%s", "Maps/oldOnNewASR-", i, "-", j, ".png");
                    //plotObjects(mfisFileName, oldASRObjectsOnNew, newASRObjects);

                    recognizedObjects = recognizeObjects(newASRObjects, oldASRObjectsOnNew);
                    if (recognizedObjects.size() > 4) {
                        for (int ii = 0; ii<int(recognizedObjects.size()); ii++) {
                            Object s = getObject(newASRObjects, recognizedObjects[ii].getCID());
                            recognizedObjectsFromCV.push_back(s);
                            s = getObject(oldASRObjects, recognizedObjects[ii].getPID());
                            recognizedObjectsFromOldASR.push_back(s);

                        }
                        probableSameObjects.clear();
                        probableSameObjects.push_back(oldASRObjectsOnNew[i]);
                        probableSameObjects.push_back(newObject);
                        sprintf(mfisFileName, "%s%d%s%d%s", "Maps/oldOnNewASR-", i, "-", j, ".png");
                        //plotObjectsOf3Kinds(mfisFileName, oldASRObjectsOnNew, newASRObjects, probableSameObjects);
                        //                    plotObjects(mfisFileName,recognizedObjectsFromCV,recognizedObjectsFromOldASR);
                        cout << "Recognized Objects: " << endl;
                        displayMinfo(recognizedObjects);
                        //waitHere();
                    }
                    //                cout<<"OldObject ID: "<<i<<" NewObject ID: "<<j<<endl;


                }
            }//for  j
        }
    }
    cout << "Old ASR" << endl;
    displayObjects(oldASRObjects);
    //plotObjects("Maps/oldAndNewASRs.png",oldASRObjects,newASRObjects);
    //waitHere();
}

//it takes current view surfaces
vector<double> findBoundariesOfCV(vector<Object> casr, double extension) {
    double leftBoundary = 0;
    double rightBoundary = 0;
    double frontBoundary = 0;
    for (int i = 0; i<int(casr.size()); i++) {
        if (casr[i].length() > 100) {
            if (casr[i].X1() < leftBoundary)
                leftBoundary = casr[i].X1();
            if (casr[i].X2() < leftBoundary)
                leftBoundary = casr[i].X2();

            if (casr[i].X1() > rightBoundary)
                rightBoundary = casr[i].X1();
            if (casr[i].X2() > rightBoundary)
                rightBoundary = casr[i].X2();

            if (casr[i].Y1() > frontBoundary)
                frontBoundary = casr[i].Y1();
            if (casr[i].Y2() > frontBoundary)
                frontBoundary = casr[i].Y2();
        }

    }
    vector<double> result;
    result.push_back(leftBoundary-extension);
    result.push_back(rightBoundary+extension);
    result.push_back(frontBoundary+extension);

    return result;
}

vector<Object> findBoundaryLinesOfCV(vector<Object> casr) {

    double filterLength = 100;

    double leftBoundary = 0;
    double rightBoundary = 0;
    double frontBoundary = 0;
    for (int i = 0; i<int(casr.size()); i++) {
        if (casr[i].length() > filterLength) {
            if (casr[i].X1() < leftBoundary)
                leftBoundary = casr[i].X1();
            if (casr[i].X2() < leftBoundary)
                leftBoundary = casr[i].X2();

            if (casr[i].X1() > rightBoundary)
                rightBoundary = casr[i].X1();
            if (casr[i].X2() > rightBoundary)
                rightBoundary = casr[i].X2();

            if (casr[i].Y1() > frontBoundary)
                frontBoundary = casr[i].Y1();
            if (casr[i].Y2() > frontBoundary)
                frontBoundary = casr[i].Y2();
        }

    }
    vector<Object> result;
    Object temp;
    temp.set(leftBoundary, 0, leftBoundary, frontBoundary, 1);
    result.push_back(temp);
    temp.set(leftBoundary, frontBoundary, rightBoundary, frontBoundary, 2);
    result.push_back(temp);
    temp.set(rightBoundary, frontBoundary, rightBoundary, 0, 3);
    result.push_back(temp);

    return result;
}

void keyInfoOfCompuetedPM(char *mappingInfoFileName, int ASRNumber, vector<int> exitPoints, vector<int> lostPoints,
        vector<int> limitingPoints, vector<int> badLocalization, vector<int> failedToRecognizeRef, int refNumberTh, int level, int set) {

    cout << "****Brief Information****" << endl << "ASRs : " << ASRNumber << endl;
    cout << "Exit Points : ";
    for (int i = 0; i<int(exitPoints.size()); i++)
        cout << " " << exitPoints[i];
    cout << endl << "Lost situation : " << lostPoints.size() << " times. MFIS updated: " << limitingPoints.size() << " times." << endl;
    cout << "Lost Point: ";
    for (int i = 0; i<int(lostPoints.size()); i++) {
        cout << " " << lostPoints[i];
    }
    cout << endl << "Limiting Points: ";
    for (int i = 0; i<int(limitingPoints.size()); i++) {
        cout << " " << limitingPoints[i];
    }
    cout << endl << "Odometric ref used: " << badLocalization.size();
    cout << endl << "Odometric ref used at: ";
    for (unsigned int i = 0; i < badLocalization.size(); i++) {
        cout << " " << badLocalization[i];
    }
    cout << endl << "Failed to recognize any ref: " << failedToRecognizeRef.size() << endl;
    for (unsigned int i = 0; i < failedToRecognizeRef.size(); i++) {
        cout << failedToRecognizeRef[i] << " ";
    }
    cout << endl;

    //writing everything in a txt file
    ofstream outFile(mappingInfoFileName, ios::out);
    outFile << "****Brief Information****" << endl;
    outFile << "DataSet: Level- " << level << " Set- " << set << endl;
    outFile << "ASRs : " << ASRNumber << endl;
    outFile << "Exit Points : ";
    for (int i = 0; i<int(exitPoints.size()); i++)
        outFile << " " << exitPoints[i];
    outFile << endl << "Lost situation : " << lostPoints.size() << " times. MFIS updated: " << limitingPoints.size() << " times." << endl;
    outFile << "Lost Point: ";
    for (int i = 0; i<int(lostPoints.size()); i++) {
        outFile << " " << lostPoints[i];
    }
    outFile << endl << "Limiting Points: ";
    for (int i = 0; i<int(limitingPoints.size()); i++) {
        outFile << " " << limitingPoints[i];
    }
    outFile << endl << "Odometric ref used: " << badLocalization.size();
    outFile << endl << "Odometric ref used at: ";
    for (unsigned int i = 0; i < badLocalization.size(); i++) {
        outFile << " " << badLocalization[i];
    }
    outFile << endl << "Failed to recognize any ref: " << failedToRecognizeRef.size() << endl;
    for (unsigned int i = 0; i < failedToRecognizeRef.size(); i++) {
        outFile << failedToRecognizeRef[i] << " ";
    }
    outFile << endl;
}

void keyInfoOfCompuetedPM(int ASRNumber, vector<int> exitPoints, vector<int> lostPoints,
        vector<int> usedRefFromMFIS, vector<int> limitingPoints, vector<int> badLocalization, 
        vector<int> failedToRecognizeRef, int refNumberTh, int level, int set) {

    cout << "****Brief Information****" << endl;
    cout << "ASRs : " << ASRNumber << endl;
    cout << "Exit Points : ";
    for (int i = 0; i<int(exitPoints.size()); i++)
        cout << " " << exitPoints[i];
    cout << endl << "Lost situation : " << lostPoints.size() <<" Reorientation situation : "<< usedRefFromMFIS.size()<< " MFIS updated: " << limitingPoints.size() << endl;
    cout << "Lost Point: ";
    for (int i = 0; i<int(lostPoints.size()); i++) {
        cout << " " << lostPoints[i];
    }
    cout << endl<<"Reorientation Point: ";
    for (int i = 0; i<int(usedRefFromMFIS.size()); i++) {
        cout << " " << usedRefFromMFIS[i];
    }
    cout << endl << "Limiting Points: ";
    for (int i = 0; i<int(limitingPoints.size()); i++) {
        cout << " " << limitingPoints[i];
    }
    cout << endl << "Odometric ref used: " << badLocalization.size();
    cout << endl << "Odometric ref used at: ";
    for (unsigned int i = 0; i < badLocalization.size(); i++) {
        cout << " " << badLocalization[i];
    }
    cout << endl << "Failed to recognize any ref: " << failedToRecognizeRef.size() << endl;
    for (unsigned int i = 0; i < failedToRecognizeRef.size(); i++) {
        cout << failedToRecognizeRef[i] << " ";
    }
    cout << endl;

    //writing everything in a txt file
    ofstream outFile("Maps/mappingLog.txt", ios::out | ios::app);

//    outFile << "Value of __LINE__ : " << __LINE__ << endl;
//    outFile << "File : " << __FILE__ << endl;
    outFile << "Date : " << __DATE__ << endl;
    outFile << "Time : " << __TIME__ << endl;

       
    outFile << endl << "****Output - Brief Information****" << endl;
    outFile << "DataSet: Level- " << level << " Set- " << set << endl;
    outFile << "ASRs : " << ASRNumber << endl;
    outFile << "Exit Points : ";
    for (int i = 0; i<int(exitPoints.size()); i++)
        outFile << " " << exitPoints[i];
    outFile << endl << "Lost situation : " << lostPoints.size() << " times. MFIS updated: " << limitingPoints.size() << " times." << endl;
    outFile << "Lost Point: ";
    for (int i = 0; i<int(lostPoints.size()); i++) {
        outFile << " " << lostPoints[i];
    }
    outFile << endl << "Limiting Points: ";
    for (int i = 0; i<int(limitingPoints.size()); i++) {
        outFile << " " << limitingPoints[i];
    }
    outFile << endl << "Odometric ref used: " << badLocalization.size();
    outFile << endl << "Odometric ref used at: ";
    for (unsigned int i = 0; i < badLocalization.size(); i++) {
        outFile << " " << badLocalization[i];
    }
    outFile << endl << "Failed to recognize any ref: " << failedToRecognizeRef.size() << endl;
    for (unsigned int i = 0; i < failedToRecognizeRef.size(); i++) {
        outFile << failedToRecognizeRef[i] << " ";
    }
    outFile << endl;
}

void pointingExperiment(vector<Object> MFIS, vector<Object> allRobotPositions, vector<Object> currentRobotPositionInMFIS) {
    MyRobot myrobot(0, 0);
    vector<Object> currentRobotPosition = myrobot.getRobot();

    double distToHome = currentRobotPositionInMFIS[6].distP1ToP1(currentRobotPosition[6]);
    //    currentRobotPositionInMFIS[6].setP2(0,1500);
    cout << "HomeDist: " << distToHome << endl;
    Object directionToHome;
    vector<Object> tempL;

//    int lineN;
//    while (true) {
//        cout << "GIve LineNumber: ";
//        cin >> lineN;
//        //directionToHome.set(MFIS[lineN].X1(),MFIS[lineN].Y1(),0,0, 1);
//        directionToHome.set(MFIS[lineN].X1(), MFIS[lineN].Y1(), currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), 1);
//        cout << "CP Dist: " << directionToHome.length() << " Angle: " << MFIS[lineN].getAngleWithLine(directionToHome);
//        cout << "Robot Pose: " << directionToHome.getAngleWithLine(currentRobotPositionInMFIS[6]) << endl;
//        tempL.push_back(directionToHome);
//        plotObjectsOf3Kinds("Maps/PointToHome.png", MFIS, tempL, currentRobotPositionInMFIS);
//        tempL.clear();
//    }

    directionToHome.set(currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), currentRobotPosition[6].X1(), currentRobotPosition[6].Y1(), 1);
    currentRobotPosition.push_back(directionToHome);
    double angleToHome = currentRobotPositionInMFIS[6].getAngleWithLine(directionToHome);
    cout << "Distance to HOME: " << distToHome << " Angle to HOME: " << angleToHome << endl;
    


    //direction lines
    Object tmp;
    tmp.set(directionToHome.mpX(), directionToHome.mpY(), directionToHome.X1(), directionToHome.Y1(), 1);
    tmp = makeLineAtPointWithObject(45, 0, 750, tmp);
    currentRobotPosition.push_back(tmp);
    tmp = makeLineAtPointWithObject(-90, 0, 750, tmp);
    currentRobotPosition.push_back(tmp);

    tmp = makeLineAtPointWithObject(0, 0, 1500, currentRobotPositionInMFIS[6]);
    currentRobotPositionInMFIS[6].setP2(tmp.X2(), tmp.Y2());
    tmp.set(currentRobotPositionInMFIS[6].X2(), currentRobotPositionInMFIS[6].Y2(), currentRobotPositionInMFIS[6].X1(), currentRobotPositionInMFIS[6].Y1(), 1);
    tmp = makeLineAtPointWithObject(45, 0, 750, tmp);
    currentRobotPosition.push_back(tmp);
    tmp = makeLineAtPointWithObject(-90, 0, 750, tmp);
    currentRobotPosition.push_back(tmp);

    plotObjects("Maps/PointToHome.png", addTwoVectorsOfObjects(currentRobotPosition, currentRobotPositionInMFIS), MFIS);
}

Transporter recognizeTargetExits(vector<Object> mfis, vector<Object> pvExitsAsObjects, vector<Object> cvExitsAsObjects, vector<double> ctda, int viewNumber) {
    cout << "\033[1;32m------------------- recognizeTargetExits module---------------------\033[0m" << endl;
    cout << "exitMFIS" << endl;
    displayObjects(mfis);
    //    cout<<"PV Exits"<<endl;
    //    displayExits(exitsFromPV);
    //    cout<<"CV Exits"<<endl;
    //    displayExits(exitsFromCV);
    //    

    // vector<Object> pvExitsAsObjects = convertExitToObject(exitsFromPV);
    // vector<Object> cvExitsAsObjects = convertExitToObject(exitsFromCV);

    //    cout<<"pv Exits"<<endl;
    //    displayObjects(pvExitsAsObjects);
    //    cout<<"cv exits"<<endl;
    //    displayObjects(cvExitsAsObjects);


    Object rp(0, 0, 0, 300, 1); //current robot position

    MyRobot myrobot(0, 0);

    Point rpos;
    double angle = (ctda[1] / 180) * PI; //angle in radian
    double rpx = ctda[0] * sin(-angle);
    double rpy = ctda[0] * cos(-angle);
    rpos.set(rpx, rpy);
    cout << endl << "v" << 1 + 1 << " dis(v12) " << ctda[0] << " angle(v12) " << ctda[1] << endl;
    cout << "robot position id-";
    rpos.display();

    vector<Object> pvExitsOnCV = xformPObjectsIntoCV(pvExitsAsObjects, rpos, angle);

    //plotObjects("Maps/exits.png", pvExitsAsObjects, cvExitsAsObjects);
    //plotObjects("Maps/exits1.png", pvExitsOnCV, cvExitsAsObjects);
    char viewFileName[100];
    sprintf(viewFileName, "%s%d%s%d%s", "Maps/view-", viewNumber - 1, " and ", viewNumber, ".png");
    //plotObjectsOf3Kinds(viewFileName,cvExitsAsObjects,pvExitsOnCV,myrobot.getRobot());
    //waitHere();
    vector<Minfo> recognizedExits = recognizeObjects(cvExitsAsObjects, pvExitsOnCV);

    if (recognizedExits.size() > 0) {
        cout << "\033[1;31m------------------- I recognize that exit....I know where I am.............:::)))\033[0m" << endl;
    } else {
        cout << "\033[1;31m------------------- I couldn't recognize any exit....I am lost.............:::(((\033[0m" << endl;
        Transporter result;
        return result;
    }

    //preparing potential objects for next step
    vector<Object> targetObjectsForNextStep;
    for (int i = 0; i<int(recognizedExits.size()); i++) {
        Object s = getObject(cvExitsAsObjects, recognizedExits[i].getCID());
        //s.display();
        s.setID(recognizedExits[i].getPID());
        //s.setKP(getObject(po_pv, reflines[i].getPID()).getKP());
        s.setKP(recognizedExits[i].getRP());
        targetObjectsForNextStep.push_back(s);
    }
    cout << "TargerExits for next step" << endl;
    displayObjects(targetObjectsForNextStep);

    Object refObjectFromMFIS, refObjectFromCV;
    cout << endl << "********** Getting Reference Objects*************** " << endl;
    refObjectFromMFIS.set(getObject(mfis, recognizedExits.back().getPID()));
    //    if(refObjectFromMFIS.X1() == 0 && refObjectFromMFIS.X2() == 0 ) //means ref isn't found in MFIS
    //        return output;
    refObjectFromMFIS.setKP(recognizedExits.back().getRP());
    refObjectFromCV.set(getObject(cvExitsAsObjects, recognizedExits.back().getCID()));

    refObjectFromMFIS.display();
    refObjectFromCV.display();

    vector<Object> refObjects;
    refObjects.push_back(refObjectFromMFIS);
    refObjects.push_back(refObjectFromCV);

    //waitHere();
    Transporter result;

    result.setReferenceObjects(refObjects);
    result.setTargetObjects(targetObjectsForNextStep);
    return result;
}

void computeRouteMap(ASRNetwork pm, vector<Object> routeMap, vector<Object> crossedExit) {
    plotObjects("Maps/5RouteMap.png", routeMap, crossedExit);
}

void useTopoMap(ASRNetwork perceptualMap, vector<Object> MFIS, vector<Object> crossedExit,
        vector<Object> refObjects) {
    vector<Object> exitFromTopoMap, tempExit, newCrossedExits;
    vector<Object> dottedExit;
    double angle, distance, distanceP1, distanceP2;
    cout << "Total Exits: " << crossedExit.size() << endl;
    vector<Object> oldExits, newOldExits;

    for (unsigned int i = 1; i < 7; i++) {
        oldExits.push_back(crossedExit[i]);
    }
    //for(unsigned int i=0;i<oldExits.size();i++) {
    newOldExits = projectingTheView(oldExits, refObjects[1], refObjects[0], 1);
    //}



    tempExit = makeSquare(perceptualMap.getASRs()[0].getASRExit2());
    tempExit = projectingTheView(tempExit, refObjects[1], refObjects[0], 1);
    //        for (unsigned int j = 0; j < tempExit.size(); j++) {
    //            dottedExit = breakTheLineInto(tempExit[j]);
    //            exitFromTopoMap = addTwoVectorsOfObjects(exitFromTopoMap, dottedExit);
    //        }
    exitFromTopoMap.push_back(tempExit[0]);

    for (unsigned int i = 7; i < crossedExit.size(); i++) {
        newCrossedExits.push_back(crossedExit[i]);
    }
    Object robotPose, robotPoseToNextExit;
    for (unsigned int i = 1; i < 5; i++) {
        refObjects.clear();
        refObjects.push_back(crossedExit[i]); //old exit
        refObjects.push_back(crossedExit[i + 6]); //new exit
        tempExit = makeSquare(perceptualMap.getASRs()[i].getASRExit2());
        tempExit = projectingTheView(tempExit, refObjects[1], refObjects[0], 1); //projected exit

        //to calculate angle and dist of next exit from just crossed exit
        Object temp;
        temp.set(crossedExit[i + 6].mpX(), crossedExit[i + 6].mpY(), crossedExit[i + 6].X2(), crossedExit[i + 6].Y2(), 1);
        robotPose = makeLineAtPointWithObject(90, 0, 1000, temp); //new exit is the robot position
        robotPoseToNextExit.set(robotPose.X1(), robotPose.Y1(), tempExit[0].X1(), tempExit[0].Y1(), 1);
        cout << "(P1)Angle: " << robotPose.getAngleWithLine(robotPoseToNextExit) << " Distance: " << robotPoseToNextExit.length() << endl;
        robotPoseToNextExit.set(robotPose.X1(), robotPose.Y1(), tempExit[0].X2(), tempExit[0].Y2(), 1);
        cout << "(P2)Angle: " << robotPose.getAngleWithLine(robotPoseToNextExit) << " Distance: " << robotPoseToNextExit.length() << endl;
        //oldExits.push_back(crossedExit[i]);
        //making dotted line of project exit
        //        for (unsigned int j = 0; j < tempExit.size(); j++) {
        //dottedExit = breakTheLineInto(tempExit[j]);
        //            exitFromTopoMap = addTwoVectorsOfObjects(exitFromTopoMap, dottedExit);
        //        }
        exitFromTopoMap.push_back(tempExit[0]);

    }

    vector<Object> original, projectedWithLPR, projectedWithExit;

    double xE = 2000;
    for (unsigned int i = 0; i < newCrossedExits.size(); i++) {

        original.push_back(Object(xE, 1000, xE + 1000, 1000, 1));

        xE = xE + 4000;

        angle = newCrossedExits[i].getAngleWithLine(exitFromTopoMap[i]);
        distance = newCrossedExits[i].distMPToPoint(exitFromTopoMap[i].mpX(), exitFromTopoMap[i].mpY());
        //distance = shortestDistanceBtwTwoObjects(newCrossedExits[i],exitFromTopoMap[i]);
        //distanceP1 = newCrossedExits[i].distP1ToP1(exitFromTopoMap[i]);
        //distanceP2 =newCrossedExits[i].distP2ToP2(exitFromTopoMap[i]);
        cout << "projected Exit " << ": Angle: " << angle << " Dist: " << distance << endl;
        projectedWithExit.push_back(makeLineAtPointWithObject(abs(angle), distance, 1000, original[i]));

        angle = newCrossedExits[i].getAngleWithLine(oldExits[i]);
        //distance = shortestDistanceBtwTwoObjects(newCrossedExits[i], newOldExits[i]);
        distance = newCrossedExits[i].distMPToPoint(oldExits[i].mpX(), oldExits[i].mpY());
        projectedWithLPR.push_back(makeLineAtPointWithObject(abs(angle), distance, 1000, original[i]));
        cout << "old Exit " << ": Angle: " << angle << " Dist: " << distance << endl;
    }




    //    for (unsigned int i = 0; i < crossedExit.size(); i++) {
    //        tempExit = makeSquare(crossedExit[i]);
    //        newCrossedExits = addTwoVectorsOfObjects(newCrossedExits, tempExit);
    //    }
    plotObjectsOf3Kinds("Maps/ProjectError.png", original, projectedWithExit, breakTheLinesInto(projectedWithLPR));
    plotObjectsOf4Kinds("Maps/Overley.png", MFIS,
            makeAllSquare(newCrossedExits),
            (makeAllRectangle(exitFromTopoMap)),
            (makeAllRectangle(oldExits)));

    //    cout<<"Perceived while traversing "<<endl;
    //    for(unsigned int i=0;i<newCrossedExits.size();i++) {
    //        cout<<newCrossedExits[i].X1()<<" "<<newCrossedExits[i].Y1()<<"  "<<newCrossedExits[i].X2()<<" "<<newCrossedExits[i].Y2()<<endl;
    //    }
    //    
    //     cout<<"Projected using exit "<<endl;
    //    for(unsigned int i=0;i<exitFromTopoMap.size();i++) {
    //        cout<<exitFromTopoMap[i].X1()<<" "<<exitFromTopoMap[i].Y1()<<" "<<exitFromTopoMap[i].X2()<<" "<<exitFromTopoMap[i].Y2()<<endl;
    //    }
    //     
    //      cout<<"Old  "<<endl;
    //    for(unsigned int i=0;i<newOldExits.size();i++) {
    //        cout<<newOldExits[i].X1()<<" "<<newOldExits[i].Y1()<<" "<<newOldExits[i].X2()<<" "<<newOldExits[i].Y2()<<endl;
    //    }

    //    cout << "Perceived while traversing " << endl;
    //    for (unsigned int i = 0; i < newCrossedExits.size(); i++) {
    //        cout << newCrossedExits[i].mpX() << "  " << newCrossedExits[i].mpY() << endl;
    //    }
    //
    //    cout << "Projected using exit " << endl;
    //    for (unsigned int i = 0; i < exitFromTopoMap.size(); i++) {
    //        cout << exitFromTopoMap[i].mpX() << " " << exitFromTopoMap[i].mpY() << endl;
    //    }
    //
    //    cout << "Old  " << endl;
    //    for (unsigned int i = 0; i < newOldExits.size(); i++) {
    //        cout << newOldExits[i].mpX() << " " << newOldExits[i].mpY() << endl;
    //    }

}

void abstractRouteMap(vector<Object> MFIS, vector<Object> robotPositionsAtLimitingPoints, vector<Point> updatingPoints, vector<Object> lastRP) {
    cout << endl << "Abstracting routeMap from PM" << endl << endl;
    vector<Object> routeMap;
    Object tempObject;
    bool combineNext = false;
    vector<Point> filteredUP;
    for (unsigned int i = 0; i < updatingPoints.size(); i++) {
        if (i == 0)
            filteredUP.push_back(updatingPoints[i]);
        else {
            if (getDistBtw2Points(updatingPoints[i - 1].X(), updatingPoints[i - 1].Y(), updatingPoints[i].X(), updatingPoints[i].Y()) < 500)
                filteredUP.back().set(updatingPoints[i].X(), updatingPoints[i].Y());
            else
                filteredUP.push_back(updatingPoints[i]);
        }
    }

    updatingPoints = filteredUP;
    for (unsigned int i = 0; i < updatingPoints.size() - 1; i++) {

        if (combineNext == false) {
            tempObject.set(updatingPoints[i].X(), updatingPoints[i].Y(), updatingPoints[i + 1].X(), updatingPoints[i + 1].Y(), 1);
            tempObject = makeLineAtPointWithObject(0, 500, tempObject.length() - 500, tempObject);
        } else {
            tempObject.setP2(updatingPoints[i + 1].X(), updatingPoints[i + 1].Y());
        }



        if (tempObject.length() < 2000) {
            combineNext = true;
        } else {
            routeMap = addTwoVectorsOfObjects(routeMap, makeArrow(tempObject));
            combineNext = false;
        }
    }
    tempObject.setP2(lastRP[6].X1(), lastRP[6].Y1());
    tempObject = makeLineAtPointWithObject(0, 500, tempObject.length() - 500, tempObject);
    routeMap = addTwoVectorsOfObjects(routeMap, makeArrow(tempObject));
    //plotObjects("Maps/1MFIS.png", robotPositionsAtLimitingPoints, MFIS);
    plotObjects("Maps/1RouteMap.png", routeMap, MFIS);
    plotObjects("Maps/1OnlyRouteMap.png", routeMap);

}

bool findLandmarkFromMemory(vector<Object> & referenceObjects, vector<Object> MFIS, vector<Object> cv,
        vector<Object> odometricRef) {
    cout<<endl<<endl<<"Inside findLandmarkFromMemory Module"<<endl;
    vector<Object> cvOnMFIS;
    Object temp;
    
    //transforming long cv objects on MFIS
    for (unsigned int i = 0; i < cv.size(); i++) {
        if (cv[i].length() > 500.0 && cv[i].distMPToPoint(0,0) < 10000) {
            temp = remakeLineP2(odometricRef[0], odometricRef[1], cv[i], cv[i].getID(), 0, odometricRef[0].getKP());
            cvOnMFIS.push_back(temp);
        }
    }
   // plotObjects("Maps/cvOnMFIS.png",MFIS,cvOnMFIS);
    
    vector<Minfo> matchedIDs;
    double distanceP1toP1, distanceP2toP2, angleDiff;
    double distanceTh = 500.0, angleTh = 6.0;
    //finding similar surfaces from cv and MFIS
    for(unsigned int i=0; i<MFIS.size(); i++) {
        if(MFIS[i].length() > 500.0) {
            for(unsigned int j=0; j<cvOnMFIS.size(); j++) {
                distanceP1toP1 = MFIS[i].distP1ToP1(cvOnMFIS[j]);
                distanceP2toP2 = MFIS[i].distP2ToP2(cvOnMFIS[j]);
                if(distanceP1toP1 < distanceTh or distanceP2toP2 < distanceTh) {
                    angleDiff = MFIS[i].getAngleWithLine(cvOnMFIS[j]);
                    cout<<"angle diff: "<<angleDiff<<endl;
                    if(abs(angleDiff) > 345)
                        angleDiff = 360.0 - angleDiff;
                    if(abs(angleDiff) < angleTh) {
                        Minfo oneMatch(MFIS[i].getID(), cvOnMFIS[j].getID(), 0, 1);
                        oneMatch.setAngleDiff(angleDiff);
                        oneMatch.setLength(MFIS[i].length());
                        if(distanceP1toP1 < distanceP2toP2) {
                            oneMatch.setDistDiff(distanceP1toP1);
                            oneMatch.setRP(1);
                        } else {
                            oneMatch.setDistDiff(distanceP2toP2);
                            oneMatch.setRP(2);
                        }
                        matchedIDs.push_back(oneMatch);
                    }
                }
            }
        }
    }
    
    bool success = false;
    if(matchedIDs.size() > 0) {
        vector<Object> newReferences;
        std::sort(matchedIDs.begin(), matchedIDs.end(), SortBasedOnLength);
        cout<<"matched ids after sorting"<<endl;
        displayMinfo(matchedIDs);
        
      //  plotObjects("Maps/cvOnMFISwithMatchedID.png",MFIS,cvOnMFIS,matchedIDs.back().getCID());
        
        temp = getObject(MFIS,matchedIDs.back().getPID());
        temp.setKP(matchedIDs.back().getRP());
        newReferences.push_back(temp);
        temp = getObject(cv,matchedIDs.back().getCID());
        temp.setKP(matchedIDs.back().getRP());
        newReferences.push_back(temp);
        referenceObjects = newReferences;
        cout<<"Landmark from Memory found "<<endl;
        success = true;
        //waitHere();
    }
    
    return success;
}
/*
vector<Object> makeViewFromPoints(int set, int v) {
    MyRobot myrobot(0,0);
    vector<Object> result;
    const char* levelName = "bin/level";
    //const char* surfaceName = "/surfaces-";
    const char* surfaceName = "/points-";
    char pointsName[80];
    vector<Point> points,xPoints;
    int firstP = v*3-2;
    double angle;//=40;
    for(unsigned int i=0;i<3;i++) {
    sprintf(pointsName, "%s%d%s%d%s%d", levelName, 1, "set", set, surfaceName, firstP+i);
    xPoints= readPoints(pointsName);
    if(i==1) {
        angle = -40;
    xPoints = xformPointsIntoCV(xPoints,Point(0,0),(angle/ 180) * PI);
    cout<<i<<" Angle: " <<(angle/ 180) * PI<<endl;
    }
    if(i==2) {
        
        angle = 40;
        xPoints = xformPointsIntoCV(xPoints,Point(0,0),(angle/ 180) * PI);
        cout<<i<<" Angle: " <<angle<<endl;
    }
    
    cout<<"Points: "<<points.size()<<endl;
    addTwoVectorsOfPoints(points,xPoints);
    sprintf(pointsName, "%s%d%s", "Maps/view-", firstP+i, ".png");
    //plotObjectsAndPoints(pointsName,myrobot.getRobot(),myrobot.getRobot(),points);
    }
    
    //arrange points according to their orientation angle
   Object _Xaxis(0,0,-300,0);//-ve x axis
    
    for(int i = 0;i<points.size();i++) {
        angle = _Xaxis.getAngleWithPoint(points[i].X(),points[i].Y());
        points[i].setOAngle(angle);
    }
   std::sort(points.begin(),points.end(),sortA2OrtAngle);
   
   vector<PointXY> laserPoints;
   for(int i = 0;i<points.size();i++) {
        laserPoints.push_back(PointXY(points[i].X(),points[i].Y()));
        
    }
    
   sprintf(pointsName, "%s%d%s", "Maps/Points-", v, ".png");
   plotObjectsAndPoints(pointsName,myrobot.getRobot(),myrobot.getRobot(),points);
   
   TestRANSACLines(points);
   
   
   
   vector<Surface> surfaces = Laser2Surface(laserPoints, 600, 200,150);
        char sname[50];
        sprintf(sname, "%s%d", "Maps/surfaces-",v);
        writeASCII(surfaces, sname);
        
        result =readASCII(sname);
    return result;
}

void TestRANSACLines(vector<Point> points)
{
    
        randomGenerator.randomize();

        // Generate random points in 2D
        // ------------------------------------
        const size_t N_LINES = 10;

        const size_t N_line = 30;
        const size_t N_noise = 50;

        const double LINE_EQ[N_LINES][3]={ 
                { 1,-1, -2 },
                { 1,+1.5, -1 },
                { 0,-1, +2 },
                { 0.5,-0.3, +1 } };

        vector_double xs,ys;
        for (size_t p=0;p<N_LINES;p++)
        {
                for (size_t i=0;i<N_line;i++)
                {
                        const double xx = randomGenerator.drawUniform(-10,10);
                        const double yy = randomGenerator.drawGaussian1D(0,0.05)  -(LINE_EQ[p][2]+LINE_EQ[p][0]*xx)/LINE_EQ[p][1];
                        xs.push_back(xx);
                        ys.push_back(yy);
                }
        }

        for (size_t i=0;i<N_noise;i++)
        {
                xs.push_back( randomGenerator.drawUniform(-15,15));
                ys.push_back( randomGenerator.drawUniform(-15,15));
        }

        // Run RANSAC
        // ------------------------------------
    //vector_double xs,ys;
        vector<pair<size_t,TLine2D > >   detectedLines;
        const double DIST_THRESHOLD = 0.2;
        
//        for(unsigned int i=0;i<points.size();i++) {
//            xs.push_back(points[i].X());
//            ys.push_back(points[i].Y());
//        }

        CTicTac	tictac;

        ransac_detect_2D_lines(xs,ys,detectedLines,DIST_THRESHOLD,5 );

        // Display output:
        cout << "RANSAC method: ransac_detect_2D_lines" << endl;
        cout << " Computation time: " << tictac.Tac()*1000.0 << " ms" << endl;
        cout << " " << detectedLines.size() << " lines detected." << endl;


        // Show GUI
        // --------------------------
        mrpt::gui::CDisplayWindowPlots  win2("Set of points", 500,500);

        win2.plot(xs,ys,".b4","points");

        unsigned int n=0;
        for (vector<pair<size_t,TLine2D> >::iterator p=detectedLines.begin();p!=detectedLines.end();++p)
        {
                vector_double lx(2),ly(2);
                lx[0] = -15;
                lx[1] = 15;
                for (vector_double::Index q=0;q<lx.size();q++)
                        ly[q] = -(p->second.coefs[2]+p->second.coefs[0]*lx[q])/p->second.coefs[1];
                win2.plot(lx,ly,"r-1",format("line_%u",n++));
        }

        win2.axis_fit();
        win2.axis_equal();

        win2.waitForKey();
}*/

Transporter recognizeTargetObjectsFromVision(vector<Object> mfis, vector<Object> po_pv, vector<Object> pobjects_cv, 
        vector<double> ctda, int viewNumber, int lastUS) {
//     cout << "\033[1;32m-------------------Inside recognizeTargetObjects For VISION module---------------------\033[0m" << endl;
//     waitHere();
    Transporter output;
//    Object rp(0, 0, 0, 300, 1); //current robot position
//
//    MyRobot myrobot(0, 0);
//    vector<Object> robloc = myrobot.getRobot();
//
//    vector<Object> tmpmfis = mfis;
//
//    Point rpos;
//    double angle = (ctda[1] / 180) * PI; //angle in radian
//    double rpx = ctda[0] * sin(-angle); //x= d*cos(th) = d*cos(90-angle) = d*sin(angle) //as aris give - value for right turn
//    double rpy = ctda[0] * cos(-angle); //y=d*sin(th)=d*sin(90-angle)=d*cos(angle)
//    rpos.set(rpx, rpy);
//    cout << endl << "v" << 1 + 1 << " dis(v12) " << ctda[0] << " angle(v12) " << ctda[1] << endl;
//    cout << "robot position id-";
//    rpos.display();
//    
//    cout<<"Num of landmarks in PV: "<<po_pv.size()<<endl;
//    cout<<"Num of landmarks in CV: "<<pobjects_cv.size()<<endl;
//
//    vector<Object> po_into_cv = xformPObjectsIntoCVForVision(po_pv, rpos, angle);
//    cout << "Num of pv landmarks onto cv: "<<po_into_cv.size() << endl;
//    //for thesis
//    char viewFileName[80];
//    sprintf(viewFileName, "%s%d%s", "Maps/PTSandCTS-", viewNumber, ".png");
//    plotObjectsOf3Kinds(viewFileName,myrobot.getRobot(),po_into_cv,pobjects_cv);
//    //waitHere();
//    
//     //find pobjects in current view
//    cout << "********looking for potential objects " << endl;
//    double angleth, p1d, p2d;
//    double ath = 10;
//    double dth = 400;
//    vector<Minfo> reflines;
//
//    vector<int> l2binserted;
//    Object rmfis, rcv;
//    //int lid=mfis[mfis.size()-1].getID();
//    vector<int> new_pobjects;
//
//       for (int i = 0; i<int(pobjects_cv.size()); i++) {
//            bool new_po = true;
//            for (int j = 0; j<int(po_into_cv.size()); j++) {
//                angleth = abs(pobjects_cv[i].getAngleWithLine(po_into_cv[j]));
//
//                if (angleth >= 345)
//                    angleth = 360 - angleth;
//
//                p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
//                p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);
//                //if(
//                cout<<"pv id "<<po_into_cv[j].getID()<<" cv id "<<pobjects_cv[i].getID();
//                cout<<" angle "<<angleth<<"p1d "<<p1d<<" p2d "<<p2d<<endl;
//
//                if (angleth < ath) {
//                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
//                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);
//
//
//
//                    int key_point = po_into_cv[j].getKP();
//                    //                    int key_point = pobjects_cv[i].getKP();
//                    
//                        if (p1d < dth or p2d < dth) {
//                            cout << po_into_cv[j].getID() << " cv id " << pobjects_cv[i].getID();
//                            cout << " angle " << angleth << " p1d " << p1d << " p2d " << p2d << endl;
//                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
//                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
//                            refline.setDistFromRobot(fdist);
//                            double fitness;
//                            if (fdist < 2000.0) {
//                                fdist = 2000.0;
//                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
//                                // fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
//                                //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
//                                //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
//                            } else {
//                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1.0 / fdist));
//                                // fitness = ( pobjects_cv[i].length() /(fdist*p1d*angleth));
//                                //fitness = ((dth - p1d)*(ath - angleth)*(1 / fdist));
//                            }
//                            refline.setDist(fitness);
//                            refline.setAngleDiff(angleth);
//                            refline.setLength(pobjects_cv[i].length());
//                            refline.setDistDiff(p1d);
//
//                            //check for double match
//                            if (reflines.size() > 0) {
//                                int pid = refline.getPID();
//                                int cid = refline.getCID();
//                                bool newmatch = true;
//                                for (int k = 0; k<int(reflines.size()); k++) {
//                                    if (reflines[k].getPID() == pid || reflines[k].getCID() == cid) {
//                                        if (reflines[k].getDist() < refline.getDist()) {
//                                            reflines[k].setPID(pid);
//                                            reflines[k].setCID(cid);
//                                            reflines[k].setDist(fitness);
//                                            if(p1d < p2d)
//                                                reflines[k].setRP(1);
//                                            else
//                                                reflines[k].setRP(2);
//                                        }
//                                        newmatch = false;
//                                    }
//                                }
//                                if (newmatch == true) {
//                                    reflines.push_back(refline);
//                                }
//                            } else //for first one
//                                reflines.push_back(refline);
//
//
//                            cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 1 << " fitness: " << fitness << endl;
//
//                            //next_po_pv.push_back(pobjects_cv[i]);
//                            //next_po_pv.back().setID(po_into_cv[j].getID());
//                        }
//                    }
//            }
//       }
//    
//    //display matching info
//    cout<<"Matched surfaces"<<endl;
//    displayMinfo(reflines);
//  //   waitHere();
//    if(reflines.size() < 1)
//        return output;
//    
//    
//    //preparing potential objects for next step
//    vector<Object> targetObjectsForNextStep;
//    for (int i = 0; i<int(reflines.size()); i++) {
//        Object s = getObject(pobjects_cv, reflines[i].getCID());
//        //s.display();
//        s.setID(reflines[i].getPID());
//        //s.setKP(getObject(po_pv, reflines[i].getPID()).getKP());
//        s.setKP(reflines[i].getRP());
//        targetObjectsForNextStep.push_back(s);
//    }
//
//    cout << "Target object for Next Step: " << endl;
//    cout << "sorted target Objects" << endl;
//
//
//    Object refObjectFromMFIS, refObjectFromCV;
//    cout << endl << "********** Getting Reference Objects*************** " << endl;
//    cout << "view - " << viewNumber << " From surface " << endl;
//    std::sort(reflines.begin(), reflines.end(), DataSortMethod);
//    //std::sort(reflines.begin(), reflines.end(), SortBasedOnGoodnessValue);
//    displayMinfo(reflines);
//
//
//    cout << "Found a suitable ref:)" << endl;
//    //waitHere();
//    refObjectFromMFIS.set(getObject(mfis, reflines.back().getPID()));
//    if (refObjectFromMFIS.X1() == 0 && refObjectFromMFIS.X2() == 0) //means ref isn't found in MFIS
//        return output;
//    refObjectFromMFIS.setKP(reflines.back().getRP());
//    refObjectFromCV.set(getObject(pobjects_cv, reflines.back().getCID()));
//    //  }
//
//    refObjectFromMFIS.display();
//    refObjectFromCV.display();
//
//
//    output.setTargetObjects(targetObjectsForNextStep);
//    vector<Object> refObjects;
//    refObjects.push_back(refObjectFromMFIS);
//    refObjects.push_back(refObjectFromCV);
//    output.setReferenceObjects(refObjects);
// //   output.setViews(allRPosition4ThisStep);
//    //    if(viewNumber == 260)
//    //       plotObjects("Maps/1reference.png",robloc,);
//    
       // waitHere();
    return output;
    

    
    
}