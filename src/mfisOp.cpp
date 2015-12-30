#include "mfisOp.H"
#include <vector>
#include <algorithm>  // Needed for sort() method

#include <cmath>
#include <iostream>
#include "Object.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "asrOp.H"

#include "readAndwriteASCII.H"

using namespace std;

#define PI 3.14159265
//vector<Minfo> reflines;

vector<Object> getLinesIFoR(vector<Object> view) 
{

    vector<Object> result;
    for (int i = 0; i<int(view.size()); i++) 
    {
        if (view[i].Y1() > 0 && view[i].Y2() > 0) 
        {
            result.push_back(view[i]);
        }
        if (view[i].Y1() < 0 && view[i].Y2() > 0) 
        {
            //Object x;
            if (view[i].X1() < view[i].X2()) {
                Object x(0, 0, view[i].X1(), 0);
                vector<double> in_sect = getIntersectionPoint(view[i], x);
                Object s(in_sect[0], in_sect[1], view[i].X2(), view[i].Y2(), view[i].getID(), view[i].nearness(), view[i].getP1OS(), view[i].getP2OS(), view[i].getGID());
                result.push_back(s);
            } else {//cout<<"hey guy: "<<endl;
                Object x(0, 0, view[i].X2(), 0);
                vector<double> in_sect = getIntersectionPoint(view[i], x);
                //cout<<in_sect[0]<<" "<<in_sect[1]<<endl;
                Object s(in_sect[0], in_sect[1], view[i].X2(), view[i].Y2(), view[i].getID(), view[i].nearness(), view[i].getP1OS(), view[i].getP2OS(), view[i].getGID());
                result.push_back(s);
            }

        }
        if (view[i].Y1() > 0 && view[i].Y2() < 0) {
            //Object x;
            if (view[i].X1() > view[i].X2()) {
                Object x(0, 0, view[i].X1(), 0);
                vector<double> in_sect = getIntersectionPoint(view[i], x);
                Object s(view[i].X1(), view[i].Y1(), in_sect[0], in_sect[1], view[i].getID(), view[i].nearness(), view[i].getP1OS(), view[i].getP2OS(), view[i].getGID());
                result.push_back(s);
            } else {
                Object x(0, 0, view[i].X2(), 0);
                vector<double> in_sect = getIntersectionPoint(view[i], x);
                Object s(view[i].X1(), view[i].Y1(), in_sect[0], in_sect[1], view[i].getID(), view[i].nearness(), view[i].getP1OS(), view[i].getP2OS(), view[i].getGID());
                result.push_back(s);
            }
        }
    }
    return result;

}


Object remakeLineP2(Object smfis, Object scv, Object rem, int id, double ang, int refpoint)
{

	double x1, y1, x2, y2;
	double angle;
	double dist;
	ang=0;

	if(refpoint==1)
	{
		angle=scv.getAngleWithPoint(rem.X1(), rem.Y1());
		dist=scv.distP1ToP1(rem);

		angle=(angle*(PI/180))-(ang*(PI/180));

		x1=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle)-((smfis.Y2()-smfis.Y1())/smfis.length())*sin(angle);
		y1=((smfis.X2()-smfis.X1())/smfis.length())*sin(angle)+((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x1=x1*dist+smfis.X1();
		y1=y1*dist+smfis.Y1();

		angle=scv.getAngleWithPoint(rem.X2(), rem.Y2());
		dist=scv.distP1ToP2(rem);
		angle=(angle*(PI/180))-(ang*(PI/180));

		x2=((smfis.X2()-smfis.X1())/smfis.length())*cos(angle)-((smfis.Y2()-smfis.Y1())/smfis.length())*sin(angle);
		y2=((smfis.X2()-smfis.X1())/smfis.length())*sin(angle)+((smfis.Y2()-smfis.Y1())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X1();
		y2=y2*dist+smfis.Y1();
	}
	else
	{
		angle=scv.getAngleWithPointFP1E(rem.X1(), rem.Y1());
		dist=scv.distP2ToP1(rem);
		angle=((angle)*(PI/180))+(ang*(PI/180));

		x1=((smfis.X1()-smfis.X2())/smfis.length())*cos(angle)-((smfis.Y1()-smfis.Y2())/smfis.length())*sin(angle);
		y1=((smfis.X1()-smfis.X2())/smfis.length())*sin(angle)+((smfis.Y1()-smfis.Y2())/smfis.length())*cos(angle);

		x1=x1*dist+smfis.X2();
		y1=y1*dist+smfis.Y2();

		angle=scv.getAngleWithPointFP1E(rem.X2(), rem.Y2());
		dist=scv.distP2ToP2(rem);
		angle=((angle)*(PI/180))+(ang*(PI/180));

		x2=((smfis.X1()-smfis.X2())/smfis.length())*cos(angle)-((smfis.Y1()-smfis.Y2())/smfis.length())*sin(angle);
		y2=((smfis.X1()-smfis.X2())/smfis.length())*sin(angle)+((smfis.Y1()-smfis.Y2())/smfis.length())*cos(angle);

		x2=x2*dist+smfis.X2();
		y2=y2*dist+smfis.Y2();
	}

	Object result(x1,y1,x2,y2, id, 0, rem.getP1OS(), rem.getP2OS(), 1);
        
        result.setPEP1(rem.getPEP1());
        result.setPEP2(rem.getPEP2());

	return result;

}

vector<Object> projectingTheView(vector<Object> currentView, Object refObjectOld, Object refObjectNew, int refPoint) {
    vector<Object> result;
    
    Object temp;
    for (unsigned int i = 0; i < currentView.size(); i++) {
        temp = remakeLineP2(refObjectOld, refObjectNew, currentView[i], currentView[i].getID(), 0,refPoint);
        result.push_back(temp);
    }
    
    return result;
}

// this functions is to find the landmark surfaces in the current view
vector<Object> findTargetObjects(vector<Object> & cv) 
{
    vector<Object> output;
    MyRobot myrobot(0, 0);
    Object leftSide, rightSide;
    double visibility = 0.0;
    
    int a = 0; //'0' means it considers frist right and left objects as Target Objects if they have at least one O.Point		
    for (int i = a; i<int(cv.size() - a); i++) 
    {
        leftSide.set(0,0,cv[i].X1(),cv[i].Y1(),i);
                rightSide.set(0,0,cv[i].X2(),cv[i].Y2(),i);
        visibility = abs(leftSide.getAngleWithLine(rightSide));        
        if (cv[i].length() > 400 ) // length of the surface ought to be longer than 400mm
        {
            if (cv[i].getP1OS() == 1 && cv[i].getP2OS() == 1) // occluding status of two points 
            {
                output.push_back(cv[i]);
                output.back().setKP(3);
                leftSide.set(0,0,cv[i].X1(),cv[i].Y1(),i);
                rightSide.set(0,0,cv[i].X2(),cv[i].Y2(),i);
                //cout<<"Angle with tObject"<<i<<" "<<myrobot.getRobot()[6].getAngleWithLine(cv[i])<<endl;
                 cout<<"Angle with tObject "<<i<<" "<<leftSide.getAngleWithLine(rightSide)<<endl;
            } else if (cv[i].getP1OS() == 1) 
            {
                
                output.push_back(cv[i]);
                output.back().setKP(1);
               // cout<<"Angle with tObject"<<i<<" "<<myrobot.getRobot()[6].getAngleWithLine(cv[i])<<endl;
                leftSide.set(0,0,cv[i].X1(),cv[i].Y1(),i);
                rightSide.set(0,0,cv[i].X2(),cv[i].Y2(),i);
                cout<<"Angle with tObject "<<i<<" "<<leftSide.getAngleWithLine(rightSide)<<endl;
            } else if (cv[i].getP2OS() == 1) 
            {
                output.push_back(cv[i]);
                output.back().setKP(2);
               // cout<<"Angle with tObject"<<i<<" "<<myrobot.getRobot()[6].getAngleWithLine(cv[i])<<endl;
                leftSide.set(0,0,cv[i].X1(),cv[i].Y1(),i);
                rightSide.set(0,0,cv[i].X2(),cv[i].Y2(),i);
                cout<<"Angle with tObject "<<i<<" "<<leftSide.getAngleWithLine(rightSide)<<endl;
            }
            
            if(cv[i].getP1OS() == 1 || cv[i].getP2OS() == 1)//added to tag target object
                cv[i].setPO(true);

        }
    }
    //plotObjectsOf3Kinds("Maps/targetObjects.png",myrobot.getRobot(),cv,output);
    //waitHere();
    return output;
}

vector<Object> findTargetObjectsFromMFIS(vector<Object>  cv) {
    vector<Object> output;
    //int a = 1; //'0' means it considers frist right and left objects as Target Objects if they have at least one O.Point		
    for (int i = 0; i<int(cv.size()); i++) {
        if (cv[i].length() > 400) {
            if (cv[i].getP1OS() == 1 && cv[i].getP2OS() == 1) {
                output.push_back(cv[i]);
                output.back().setKP(3);
            } else if (cv[i].getP1OS() == 1) {
                output.push_back(cv[i]);
                output.back().setKP(1);
            } else if (cv[i].getP2OS() == 1) {
                output.push_back(cv[i]);
                output.back().setKP(2);
            }           
            
        }
    }
    return output;
}

vector<Object> findPotentialObjects(vector<Object> cv) {
	vector<Object> output;

	//for first object
	if(cv[0].length() > 400){
		if(cv[0].getP2OS() == 1){
			if(cv[0].distP2ToP1(cv[1]) < 300) {
				output.push_back(cv[0]);
				output.back().setKP(2);
			}
		}
	}
	for(int i=1;i<int(cv.size()-1);i++) {
		if(cv[i].length() > 400) {
			if(cv[i].getP1OS() == 1 && cv[i].distP1ToP2(cv[i-1]) < 300) {
				output.push_back(cv[i]);
				if(cv[i].getP2OS() == 1 && cv[i].distP2ToP1(cv[i+1]) < 300)
					output.back().setKP(3);
				else
					output.back().setKP(1);
			}
			else if(cv[i].getP2OS() == 1 && cv[i].distP2ToP1(cv[i+1]) < 300) {
					output.push_back(cv[i]);
					output.back().setKP(2);
			}
			else if(cv[i].getP1OS() == 1 && cv[i].getP2OS() == 1) {//latest addition
				output.push_back(cv[i]);
					output.back().setKP(3);
			}
		}
	}


	//for last object
	if(cv[cv.size()-1].length() > 400){
		if(cv[cv.size()-1].getP1OS() == 1){
			if(cv[cv.size()-1].distP1ToP2(cv[cv.size()-2]) < 300) {
				output.push_back(cv[cv.size()-1]);
				output.back().setKP(1);
			}
		}
	}


	return output;
}

bool isUpdateEssential(vector<Object> tobjects, double angle, double dist) {
	bool essential;
	angle = (angle/180)*PI;//angle in radian
	double rpx=dist*sin(-angle);
	double rpy=dist*cos(-angle);
	Point robpos;
	robpos.set(rpx,rpy);

	//cout<<"coordTrans "<<angle<<" "<<dist<<endl;
	//cout<<"Robot Position "<<rpx<<" "<<rpy<<endl;

	vector<Object> tobjects_cv=xformPObjectsIntoCV(tobjects,robpos,angle);
	//cout<<"po into nextcv "<<po_into_next_cv.size()<<endl;
	//displayObjects(tobjects_cv);
	//cout<<"size of mfis "<<tmpmfis.size()<<endl;

	if(tobjects_cv.size() < 2 ) {
		essential = true;
	}
	else
		essential=false;

	return essential;
}


vector<vector<Object> > recognizeTargetObjects(vector<Object> mfis, vector<Object> cv, vector<Object> po_pv, vector<Object> pobjects_cv, Point rpos, double angle) {
	vector<vector<Object> > result;

    vector<Object> po_into_cv = xformPObjectsIntoCV(po_pv, rpos, angle);
    //cout<<"pvintocv potential objects"<<endl;
    //displayObjects(po_into_cv);
    //find pobjects in current view
    cout << "********looking for potential objects " << endl;
    double angleth, p1d, p2d;
    double ath = 5;
    double dth = 400;
    vector<Minfo> reflines;

    vector<int> l2binserted;

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
                //cout<<po_into_cv[j].getID()<<" cv id "<<pobjects_cv[i].getID();
                //cout<<" angle "<<angleth<<"p1d "<<p1d<<" p2d "<<p2d<<endl;

                if (angleth < ath) {
                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);



                    //int key_point=po_into_cv[j].getKP();
                    int key_point = pobjects_cv[i].getKP();
                    if (key_point == 1) {
                        if (p1d < dth) {
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                            double fitness;
                            if (fdist < 2000) {
                                fdist = 2000;
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                            } else
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                            refline.setDist(fitness);

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


                            cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 1 << " " << fitness << endl;

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                        }
                    }
                    if (key_point == 2) {
                        if (p2d < dth) {
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 2);
                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                            double fitness;
                            if (fdist < 2000) {
                                fdist = 2000;
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                            } else
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                            refline.setDist(fitness);

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
                            cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 2 << " " << fitness << endl;

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                        }
                    }

                    if (key_point == 3) {
                        if (p1d < dth || p2d < dth) {
                            cout << po_into_cv[j].getID() << " cv id " << pobjects_cv[i].getID();
                            cout << " angle " << angleth << "p1d " << p1d << " p2d " << p2d << endl;
                            if (p1d < p2d) {
                                Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                                double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                                double fitness;
                                if (fdist < 2000) {
                                    fdist = 2000;
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                    //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                                } else
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                refline.setDist(fitness);

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
                                cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 1 << " " << fitness << endl;
                            } else {
                                //if(po_into_cv[j].getID() == 144)
                                //	break;
                                Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 2);
                                double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                                double fitness;
                                if (fdist < 2000) {
                                    fdist = 2000;
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                                } else
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                                refline.setDist(fitness);

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
                                cout << po_into_cv[j].getID() << " " << pobjects_cv[i].getID() << " " << 2 << " " << fitness << endl;
                            }

                            //next_po_pv.push_back(pobjects_cv[i]);
                            //next_po_pv.back().setID(po_into_cv[j].getID());
                            new_po = false;

                        }//if p1d or p2d
                    }//if keypoint 3
                }//if angleth
            }//for j

            if (new_po == true && pobjects_cv[i].distP1ToPoint(0, 0) > 3000) {
                new_pobjects.push_back(pobjects_cv[i].getID());
            }

        }

        //resetting angle and distance threshold
        if (reflines.size() < 1) {
            ath = ath + 5;
            if (ath > 15) {
                ath = 15;
                dth = dth + 100;
            }
        }
        if (ath == 15 && dth == 1000) {
            cout << "no reference found" << endl;
            return result;
        }
    } while (reflines.size() < 1);

    //cout<<"po pv "<<endl;
    //displayObjects(po_pv);

    //preparing potential objects for next step
    //cout<<endl<<"testing "<<endl;
    vector<Object> next_po_pv;
    for (int i = 0; i<int(reflines.size()); i++) {
        Object s = getObject(cv, reflines[i].getCID());
        //s.display();
        s.setID(reflines[i].getPID());
        s.setKP(getObject(po_pv, reflines[i].getPID()).getKP());
        next_po_pv.push_back(s);
    }

    std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    //cout<<endl<<"after sorting"<<endl;
    //displayMinfo(reflines);

    Object rmfis, rcv;
    //vector<Object> robpositions;
    //cout<<endl<<"********** Getting Current robot position *************** "<<endl;
    //cout<<endl<<"using these ref lines "<<endl;
    rmfis.set(getObject(mfis, reflines.back().getPID()));
    rcv.set(getObject(cv, reflines.back().getCID()));
    //rmfis.display();
    //rcv.display();

    rmfis.setKP(reflines.back().getRP());
    rcv.setKP(reflines.back().getRP());

    vector<Object> robjects;
    robjects.push_back(rmfis);
    robjects.push_back(rcv);


    result.push_back(next_po_pv); //recognized target objects
    result.push_back(robjects); //reference objects

    return result;
}

vector<vector<Object> > updateForTurned(vector<Object> tmpmfis, vector<Object> cv, vector<Object> next_po_pv, vector<Object> pobjects_cv, vector<Object> ref_obj, vector<double> distang) {
    vector<vector<Object> > result;

    double angle = (distang[1] / 180) * PI; //angle in radian
    double rpx = distang[0] * sin(-angle);
    double rpy = distang[0] * cos(-angle);
    Point robpos;
    robpos.set(rpx, rpy);

    vector<Object> cv_into_pv = xformCVIntoPV(cv, robpos, angle); //angle in radian
    cout << endl << "cv into pv" << endl;
    //displayObjects(cv_into_pv);
    vector<Object> newlines;
    int lid = tmpmfis[tmpmfis.size() - 1].getID();
    for (int i = 0; i<int(cv_into_pv.size()); i++) {
        if (cv_into_pv[i].Y1() < 0 && cv_into_pv[i].Y2() < 0) {
            newlines.push_back(remakeLineP2(ref_obj[0], ref_obj[1], cv[i], 1, 0, ref_obj[0].getKP()));
            newlines.back().setID(lid + 1);
            tmpmfis.push_back(newlines.back());
            //update potential objects vector
            for (int j = 0; j<int(pobjects_cv.size()); j++) {
                if (pobjects_cv[j].getID() == cv[i].getID()) {
                    next_po_pv.push_back(cv[i]);
                    cout << " new po added to previous list " << cv[i].getID() << endl;
                    cout << pobjects_cv[j].getKP() << endl;
                    next_po_pv.back().setID(lid + 1);
                    next_po_pv.back().setKP(pobjects_cv[j].getKP());
                    break;
                }
            }
            lid++;
        }
    }

    result.push_back(tmpmfis);
    result.push_back(next_po_pv);
    return result;
}

vector<vector<Object> > update(vector<Object> tmpmfis, vector<Object> cv, vector<Object> pobjects_cv, vector<Object> refobjects) {
		cout<<"inside updating module "<<pobjects_cv.size()<<" "<<refobjects.size()<<" "<<cv.size()<<" "<<tmpmfis.size()<<endl;
		int lid=tmpmfis[tmpmfis.size()-1].getID();

		int refpoint=refobjects[0].getKP();

		vector<Object> tmp_po;
		Object rp;
		rp.set(0,0,300,0,1);
									//rmfis			rcv
		Object crp_x=remakeLineP2(refobjects[0],refobjects[1],rp,1,0, refpoint);

		rp.set(0,0,0,300,1);
		Object rpc=remakeLineP2(refobjects[0],refobjects[1],rp,1,0, refpoint);

		Point crp;
		crp.set(rpc.X1(),rpc.Y1());
		double aaa=crp_x.getAngleWithXaxis();
		aaa=(aaa/180)*PI;

		//cout<<endl<<"used angle "<<aaa<<endl;
		vector<Object> back_mfis=discardLinesIFoR(tmpmfis,crp,aaa,cv,refobjects[0],refobjects[1],refpoint);
		//cout<<"finding lines in front of robot "<<endl;

		//cout<<endl<<"mfis before discard "<<endl;
		//displayObjects(tmpmfis);
		//cout<<endl<<"mfis after discard "<<endl;
		//displayObjects(back_mfis);

		//ud_c.back().setID(ud_c.back().getID()+1);//to count update
		for(int k=1;k<int(cv.size()-1);k++) {
			Object new_line=remakeLineP2(refobjects[0],refobjects[1],cv[k],lid+1,0,refpoint);
			tmpmfis.push_back(new_line);
			back_mfis.push_back(new_line);

			for(int i=0;i<int(pobjects_cv.size());i++) {
				if(pobjects_cv[i].getID() == cv[k].getID()){
					Object tmp;
					tmp.set(pobjects_cv[i]);
					tmp.setID(lid+1);
					tmp.setKP(pobjects_cv[i].getKP());
					tmp_po.push_back(tmp);
				}
			}
			lid++;
		}
		cout<<"updating accomplished"<<endl;
		vector<Object> next_po_pv;

		for(int i=0;i<int(tmp_po.size());i++) {
			next_po_pv.push_back(tmp_po[i]);
		}
		//plotObjects("MFIS/back_mfis.png",crp_x,back_mfis);
		//cout<<endl<<"after adding new lines"<<endl;
		//displayObjects(back_mfis);
		tmpmfis.clear();
		tmpmfis=back_mfis;

	vector<vector<Object> > result;
	result.push_back(tmpmfis);//updated mfis
	result.push_back(next_po_pv);//reseted target objects

	return result;
}

vector<vector<Object> > updateMFIS(vector<Object> mfis, vector<Object> po_pv, vector<Object> cv, Point rpos, double angle, int v, vector<Object> ud_c) {
    cout << "\033[1;32m-------------------Inside updateMFIS module---------------------\033[0m" << endl;
    vector<vector<Object> > output;
    Object rp(0, 0, 0, 300, v); //current robot position

    MyRobot myrobot(0, 0);
    vector<Object> robloc = myrobot.getRobot();

    vector<Object> tmpmfis = mfis;
    //cout<<"cv "<<endl;
    //displayObjects(cv);
    //for(int i=0;i<int(cv.size());i++)
    //cout<<cv[i].getID()<<" length "<<cv[i].length()<<endl;
    //cout<<"pv's potential objects "<<endl;
    //displayObjects(po_pv);

    vector<Object> po_into_cv = xformPObjectsIntoCV(po_pv, rpos, angle);
    //cout<<"pvintocv potential objects"<<endl;
    //displayObjects(po_into_cv);
    vector<Object> pview_into_cv = xformPObjectsIntoCV(mfis, rpos, angle);
    //plotObjects("MFIS/pvAndcv.png",po_into_cv,cv);

    vector<Object> tmp_po_for_mfis;
    for (int i = 0; i<int(po_into_cv.size()); i++) {
        tmp_po_for_mfis.push_back(getObject(mfis, po_into_cv[i].getID()));
    }
    //cout<<tmp_po_for_mfis.size()<<endl;
    /*
            char tmpname[100];
            sprintf(tmpname, "%s%d", "MFIS/mfis_with_po_ifor-",v+1);
            plotObjects(tmpname,rp,mfis,tmp_po_for_mfis);
     */
    vector<Object> pobjects_cv = findTargetObjects(cv);
    cout << "cv's potential objects" << endl;
    displayObjects(pobjects_cv);
    //plotObjects("MFIS/pobjest", rp,cv);

    if (v == 57)
        plotObjects("MFIS/refobjects.png", po_into_cv, pobjects_cv);

    /*
    vector<Exit> exits=findExits(cv);

    char cfname[100];
    sprintf(cfname, "%s%d%s", "MFIS/view-",v+1,".png");
    plotObjectsAndPExits(cfname, robloc,cv,exits);
     */

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
                //cout<<po_into_cv[j].getID()<<" cv id "<<pobjects_cv[i].getID();
                //cout<<" angle "<<angleth<<"p1d "<<p1d<<" p2d "<<p2d<<endl;

                if (angleth < ath) {
                    p1d = pobjects_cv[i].distP1ToP1(po_into_cv[j]);
                    p2d = pobjects_cv[i].distP2ToP2(po_into_cv[j]);



                    //int key_point=po_into_cv[j].getKP();
                    int key_point = pobjects_cv[i].getKP();
                    if (key_point == 1) {
                        if (p1d < dth) {
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 1);
                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                            double fitness;
                            if (fdist < 2000) {
                                fdist = 2000;
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                            } else
                                fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                            refline.setDist(fitness);

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
                            Minfo refline(po_into_cv[j].getID(), pobjects_cv[i].getID(), 0, 2);
                            double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                            double fitness;
                            if (fdist < 2000) {
                                fdist = 2000;
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                            } else
                                fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                            refline.setDist(fitness);

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
                                if (fdist < 2000) {
                                    fdist = 2000;
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                    //cout<<"less t 2000 "<<fdist<<" "<<pobjects_cv[i].length()<<" "<<(400-p1d)<<" "<<(ath-angleth)<<" "<<1/fdist<<endl;
                                } else
                                    fitness = (pobjects_cv[i].length()*(dth - p1d)*(ath - angleth)*(1 / fdist));
                                refline.setDist(fitness);

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
                                double fdist = pobjects_cv[i].distP1ToPoint(0, 0);
                                double fitness;
                                if (fdist < 2000) {
                                    fdist = 2000;
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                                } else
                                    fitness = (pobjects_cv[i].length()*(dth - p2d)*(ath - angleth)*(1 / fdist));
                                refline.setDist(fitness);

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

            if (new_po == true && pobjects_cv[i].distP1ToPoint(0, 0) > 3000) {
                new_pobjects.push_back(pobjects_cv[i].getID());
            }

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
            return output;
        }
    } while (reflines.size() < 1);


    //preparing potential objects for next step
    //cout<<endl<<"testing "<<endl;
    vector<Object> next_po_pv;
    for (int i = 0; i<int(reflines.size()); i++) {
        Object s = getObject(cv, reflines[i].getCID());
        //s.display();
        s.setID(reflines[i].getPID());
        s.setKP(getObject(po_pv, reflines[i].getPID()).getKP());
        next_po_pv.push_back(s);
    }
    //cout<<"view no "<<v<<endl;
    if (v == 57) {

    } else if (v == 129)
        return output;
    else if (v == 138)
        return output;
    else if (v == 143)
        return output;
    else if (v == 145)
        return output;
    else if (v == 63) {

    } else
        std::sort(reflines.begin(), reflines.end(), DataSortMethod);
    cout << "ref lines " << " " << v << endl;
    displayMinfo(reflines);

    if (reflines.back().getDist() < 50)
        return output;

    vector<Object> robpositions;
    cout << endl << "********** Getting Current robot position *************** " << endl;
    cout << endl << "using these ref lines " << endl;
    if (v == 101) {
        rmfis.set(getObject(mfis, 579));
        rcv.set(getObject(cv, 9));
    } else {
        rmfis.set(getObject(mfis, reflines.back().getPID()));
        rcv.set(getObject(cv, reflines.back().getCID()));
    }
    rmfis.display();
    rcv.display();

    Object rpc = remakeLineP2(rmfis, rcv, rp, v + 1, 0, reflines.back().getRP());

    //adding new lines when robot took turn a certain angle
    cout << endl << endl << "turned angle " << angle << " " << angle * (180 / PI) << endl;

    bool update_occured = false;

    if (abs(angle * (180 / PI)) > 20) {
        vector<Object> cv_into_pv = xformCVIntoPV(cv, rpos, angle);
        cout << endl << "cv into pv" << endl;
        //displayObjects(cv_into_pv);

        vector<Object> newlines;
        int lid = tmpmfis[tmpmfis.size() - 1].getID();
        for (int i = 0; i<int(cv_into_pv.size()); i++) {
            if (cv_into_pv[i].Y1() < 0 && cv_into_pv[i].Y2() < 0) {
                newlines.push_back(remakeLineP2(rmfis, rcv, cv[i], v + 1, 0, reflines.back().getRP()));
                newlines.back().setID(lid + 1);
                tmpmfis.push_back(newlines.back());
                //update potential objects vector
                for (int j = 0; j<int(pobjects_cv.size()); j++) {
                    if (pobjects_cv[j].getID() == cv[i].getID()) {
                        next_po_pv.push_back(cv[i]);
                        cout << " new po added to previous list " << cv[i].getID() << endl;
                        cout << pobjects_cv[j].getKP() << endl;
                        next_po_pv.back().setID(lid + 1);
                        next_po_pv.back().setKP(pobjects_cv[j].getKP());
                        break;
                    }
                }
                lid++;
            }
        }
        //plotObjects("MFIS/mfis_n_nls",rp,tmpmfis,newlines);
        ud_c.back().setID(ud_c.back().getID() + 1); //to count update
        update_occured = true;
    }

    char ctfname[80];
    sprintf(ctfname, "%s%d%s%d", "bin/level", ud_c.back().getKP(), "/coordTrans-", v + 1 + 1);
    //sprintf(ctfname, "%s%d", "bin/level1/coordTrans-",v+1+1);
    vector <double> ctda = readCoordTrans(ctfname);

    double ang = (ctda[1] / 180) * PI; //angle in radian
    double rpx = ctda[0] * sin(-ang);
    double rpy = ctda[0] * cos(-ang);
    Point robpos;
    robpos.set(rpx, rpy);

    Object tmprp, next_rmfis, next_rcv;
    tmprp.set(ctda[0] * sin(-ang), ctda[0] * cos(-ang), (ctda[0] + 500) * sin(-ang), (ctda[0] + 500) * cos(-ang), 1);
    next_rmfis = remakeLineP2(rmfis, rcv, tmprp, v + 1, 0, reflines.back().getRP());
    next_rmfis.setKP(1);
    vector<Object> next_ref;
    next_ref.push_back(next_rmfis);
    tmprp.set(0, 0, 0, 500, 1);
    next_ref.push_back(tmprp);

    cout << v + 1 + 1 << " coordTrans " << ctda[0] << " " << ctda[1] << endl;
    cout << "Robot Position " << rpx << " " << rpy << endl;

    vector<Object> po_into_next_cv = xformPObjectsIntoCV(next_po_pv, robpos, ang);
    //cout<<"po into nextcv "<<po_into_next_cv.size()<<endl;
    displayObjects(po_into_next_cv);
    cout << "size of mfis " << tmpmfis.size() << endl;



    if (po_into_next_cv.size() < 2) { //|| nupdate==true){
        cout << "*************** updating(MUST) ***************" << endl;
        int lid = tmpmfis[tmpmfis.size() - 1].getID();
        //int i=0;
        vector<Object> tmp_po;


        rp.set(0, 0, 300, 0, 1);
        Object crp_x = remakeLineP2(rmfis, rcv, rp, v + 1, 0, reflines.back().getRP());


        Point crp;
        crp.set(rpc.X1(), rpc.Y1());
        double aaa = crp_x.getAngleWithXaxis();
        aaa = (aaa / 180) * PI;

        cout << endl << "used angle " << aaa << endl;
        vector<Object> back_mfis = discardLinesIFoR(tmpmfis, crp, aaa, cv, rmfis, rcv, reflines.back().getRP());
        cout << "finding lines in front of robot " << endl;

        ud_c.back().setID(ud_c.back().getID() + 1); //to count update
        for (int k = 1; k<int(cv.size() - 1); k++) {
            Object new_line = remakeLineP2(rmfis, rcv, cv[k], lid + 1, 0, reflines.back().getRP());
            tmpmfis.push_back(new_line);
            back_mfis.push_back(new_line);

            for (int i = 0; i<int(pobjects_cv.size()); i++) {
                if (pobjects_cv[i].getID() == cv[k].getID()) {
                    Object tmp;
                    tmp.set(pobjects_cv[i]);
                    tmp.setID(lid + 1);
                    tmp.setKP(pobjects_cv[i].getKP());
                    tmp_po.push_back(tmp);
                }
            }
            lid++;
        }
        cout << "done" << endl << "**updated MFIS" << endl;
        next_po_pv.clear();

        for (int i = 0; i<int(tmp_po.size()); i++) {
            next_po_pv.push_back(tmp_po[i]);
        }
        //plotObjects("MFIS/back_mfis.png",crp_x,back_mfis);
        cout << endl << "after adding new lines" << endl;
        //displayObjects(back_mfis);
        tmpmfis.clear();
        tmpmfis = back_mfis;

        update_occured = true;
    }

    vector<Object> robloc_in_mfis = myrobot.inMFIS(rmfis, rcv, reflines.back().getRP());

    vector<Object> refobjects;
    rmfis.setKP(reflines.back().getRP());
    refobjects.push_back(rmfis);
    refobjects.push_back(rcv);

    cout << "rmfis rcv" << reflines.back().getRP() << endl;

    rmfis.display();
    rcv.display(); 

    output.push_back(tmpmfis); //
    output.push_back(next_po_pv);
    output.push_back(robloc_in_mfis);
    output.push_back(ud_c);
    output.push_back(refobjects);

    vector<Object> updatedone;
    if (update_occured == true) {
        updatedone.push_back(rmfis);
    }
    output.push_back(updatedone);

    output.push_back(next_ref); //for future use
    cout << "updatedone size " << updatedone.size() << endl;
    return output;
}
//angel in degree

vector<vector<Object> > newUpdateMFIS(vector<Object> mfis, vector<Object> po_pv, vector<Object> cv, double dist, double angle, int v, vector<Object> ud_c) {
    vector<vector<Object> > output;

    angle = (angle / 180) * PI; //angle in radian
    double rpxx = dist * sin(-angle);
    double rpyy = dist * cos(-angle);
    Point rpos(rpxx, rpyy);

    vector<Object> tmpmfis = mfis;
    Object rp(0, 0, 0, 300, v); //current robot position
    vector<Object> pobjects_cv = findPotentialObjects(cv);
    vector<vector<Object> > recog_output = recognizeTargetObjects(mfis, cv, po_pv, pobjects_cv, rpos, angle); //angle in degree

    vector<Object> next_po_pv = recog_output.at(0);
    vector<Object> ref_objects = recog_output.at(1);
    int ref_point = ref_objects[0].getKP();

    cout << endl << "********** Getting Current robot position *************** " << endl;
    cout << endl << "using these ref lines " << endl;

    ref_objects[0].display();
    ref_objects[1].display();

    Object rpc = remakeLineP2(ref_objects[0], ref_objects[1], rp, 1, 0, ref_point);


    //cout<<" robots positions"<<endl;
    //displayObjects(robpositions);

    //adding new lines when robot took turn a certain angle
    cout << endl << endl << "turned angle " << angle << " " << angle * (180 / PI) << endl;

    Point robpos;
    angle = (angle / 180) * PI; //angle in radian
    double rpx = dist * sin(-angle);
    double rpy = dist * cos(-angle);
    robpos.set(rpx, rpy);

    if (abs(angle * (180 / PI)) > 20) {
        vector<Object> cv_into_pv = xformCVIntoPV(cv, robpos, angle);
        cout << endl << "cv into pv" << endl;
        //displayObjects(cv_into_pv);

        vector<Object> newlines;
        int lid = tmpmfis[tmpmfis.size() - 1].getID();
        for (int i = 0; i<int(cv_into_pv.size()); i++) {
            if (cv_into_pv[i].Y1() < 0 && cv_into_pv[i].Y2() < 0) {
                newlines.push_back(remakeLineP2(ref_objects[0], ref_objects[1], cv[i], 1, 0, ref_point)); //rmfis,rcv,cv[i],v+1,0, reflines.back().getRP()));
                newlines.back().setID(lid + 1);
                tmpmfis.push_back(newlines.back());
                //update potential objects vector
                for (int j = 0; j<int(pobjects_cv.size()); j++) {
                    if (pobjects_cv[j].getID() == cv[i].getID()) {
                        next_po_pv.push_back(cv[i]);
                        cout << " new po added to previous list " << cv[i].getID() << endl;
                        cout << pobjects_cv[j].getKP() << endl;
                        next_po_pv.back().setID(lid + 1);
                        next_po_pv.back().setKP(pobjects_cv[j].getKP());
                        break;
                    }
                }
                lid++;
            }
        }
        //plotObjects("MFIS/mfis_n_nls",rp,tmpmfis,newlines);
    }

    cout << "after adding po pv" << endl;
    displayObjects(next_po_pv);


    cout << endl << "next_po_pv" << endl;
    displayObjects(next_po_pv);

    char ctfname[20];
    sprintf(ctfname, "%s%d", "bin/coordTrans-", v + 1 + 1);
    vector <double> ctda = readCoordTrans(ctfname);

    double ang = (ctda[1] / 180) * PI; //angle in radian
    rpx = ctda[0] * sin(-ang);
    rpy = ctda[0] * cos(-ang);
    //Point robpos;
    robpos.set(rpx, rpy);

    cout << v + 1 + 1 << " coordTrans " << ctda[0] << " " << ctda[1] << endl;
    cout << "Robot Position " << rpx << " " << rpy << endl;

    vector<Object> po_into_next_cv = xformPObjectsIntoCV(next_po_pv, robpos, ang);
    cout << "po into nextcv " << po_into_next_cv.size() << endl;
    displayObjects(po_into_next_cv);
    cout << "size of mfis " << tmpmfis.size() << endl;

    if (po_into_next_cv.size() < 2) { //|| nupdate==true){
        cout << "*************** updating(MUST) ***************" << endl;
        vector<vector<Object> > up_output = update(tmpmfis, cv, pobjects_cv, ref_objects);
        tmpmfis = up_output.at(0);
        next_po_pv.clear();
        next_po_pv = up_output.at(1);
    }
    cout << endl << "next po pv after adding new pobjects from cv" << endl;
    displayObjects(next_po_pv);

    cout << "size of mfis " << tmpmfis.size() << endl;
    //displayObjects(tmpmfis);

    cout << " v " << v << endl;

    MyRobot myrobot(0, 0);
    vector<Object> robloc_in_mfis = myrobot.inMFIS(ref_objects[0], ref_objects[1], ref_point);

    output.push_back(tmpmfis);
    output.push_back(next_po_pv);
    output.push_back(robloc_in_mfis);
    output.push_back(ud_c);


    return output;
}

vector<Object> updateIfNecessary(vector<Object> mfis,vector<Object> cv,vector<Object> exits,vector<vector<Object> > allrpos,vector<Object> ref_objects) {
	cout<<"inside updateIfNecessary module"<<endl;
	vector<Object> result;
	Object robpath;
	robpath.set(allrpos[allrpos.size()-2][6].X1(),allrpos[allrpos.size()-2][6].Y1(),allrpos[allrpos.size()-1][6].X1(),allrpos[allrpos.size()-1][6].Y1(),1);//last travelled path

	bool update_necessary=false;

	//checking whether destination is crossed
	for(int i=0;i<int(exits.size());i++) {
		if(checkForIntersection(exits[i],robpath)== 1) {
			update_necessary=true;
		}
	}

	if(update_necessary == true) {
		//plotObjects("MFIS/ndview.png", allrpos[0], cv);
	}

	result.push_back(robpath);
	cout<<allrpos[0].size();

	//plotObjects("MFIS/newMFIS.png", allrpos[0], mfis);

	return result;
}

vector<vector<Object> > updateUsingRobotPosition(vector<Object> tmpmfis,vector<Object> cv,vector<Object> pobjects_cv,vector<Object> special_ref, vector<Object> ud_c) {
	cout<<"\033[1;32m-------------------Inside updateUsingRobotPosition module---------------------\033[0m"<<endl;
	int lid=tmpmfis[tmpmfis.size()-1].getID();
	//int i=0;
	vector<Object> tmp_po;

	MyRobot myrobot(0,0);
		vector<Object> robloc=myrobot.getRobot();

	Object rmfis,rcv;
	rmfis.set(special_ref[0]);
	rcv.set(special_ref[1]);


	Object rp;
	rp.set(0,0,300,0,1);
	Object crp_x=remakeLineP2(rmfis,rcv,rp,1,0, rmfis.getKP());

	Object rpc=remakeLineP2(rmfis,rcv,rp,1,0, rmfis.getKP());

	Point crp;
	crp.set(rpc.X1(),rpc.Y1());
	double aaa=crp_x.getAngleWithXaxis();
	aaa=(aaa/180)*PI;

	cout<<endl<<"used angle "<<aaa<<endl;
	vector<Object> back_mfis=discardLinesIFoR(tmpmfis,crp,aaa,cv,rmfis,rcv,rmfis.getKP());
	cout<<"finding lines in front of robot "<<endl;

	bool update_occured = false;
	ud_c.back().setID(ud_c.back().getID()+1);//to count update
	for(int k=1;k<int(cv.size()-1);k++) {
		Object new_line=remakeLineP2(rmfis,rcv,cv[k],lid+1,0,rmfis.getKP());
		tmpmfis.push_back(new_line);
		back_mfis.push_back(new_line);

		for(int i=0;i<int(pobjects_cv.size());i++) {
			if(pobjects_cv[i].getID() == cv[k].getID()){
				Object tmp;
				tmp.set(pobjects_cv[i]);
				tmp.setID(lid+1);
				tmp.setKP(pobjects_cv[i].getKP());
				tmp_po.push_back(tmp);
			}
		}
		lid++;
		update_occured = true;
	}
	cout<<"done"<<endl<<"**updated MFIS"<<endl;

	//plotObjects("MFIS/back_mfis.png",crp_x,back_mfis);
	cout<<endl<<"after adding new lines"<<endl;
	//displayObjects(back_mfis);
	tmpmfis.clear();
	tmpmfis=back_mfis;

	vector<Object> robloc_in_mfis=myrobot.inMFIS(rmfis,rcv,rmfis.getKP());
	ud_c.back().setID(ud_c.back().getID()+1);//to count update

	Object tmprp,next_rmfis,next_rcv;
	tmprp.set(0,0,0,500,1);
	next_rmfis=remakeLineP2(rmfis,rcv,tmprp,1,0, rmfis.getKP());
	next_rmfis.setKP(1);
	vector<Object> next_ref;
	next_ref.push_back(next_rmfis);
	next_ref.push_back(tmprp);

	vector<vector<Object> > output;

	output.push_back(tmpmfis);//
		output.push_back(tmp_po);
		output.push_back(robloc_in_mfis);
		output.push_back(ud_c);
		output.push_back(special_ref);

		vector<Object> updatedone;
		if(update_occured == true) {
			updatedone.push_back(rmfis);
		}
		output.push_back(updatedone);

		output.push_back(next_ref);//for future use

		return output;

	//update_occured=true;
}
