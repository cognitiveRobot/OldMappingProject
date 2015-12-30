#include "mfisOp.H"
#include <vector>
#include <algorithm>  // Needed for sort() method

#include <cmath>
#include <iostream>
#include <valarray>
#include "Object.H"
#include "asr.H"
#include "GeometricOp.H"
#include "Plotting.H"
#include "asrOp.H"
#include "Transporter.H"
#include "Map.H"
#include "CompareASR.H"
#include "Mapping.H"

using namespace std;

#ifdef ASR_NAMESPACE
using namespace asr; //for autocomplition of asr members
#endif

#define PI 3.14159265

vector<Object> getExitsInCV(vector<Object> nextpv) {

    Object es; //robpos(0,0) to minpoint of exit
    int elos;
    //double erang;
    vector<Object> exits;
    Object exitObject;
    cout << "Exits - a particular gap in a local space" << endl;
    for (int i = 0; i<int(nextpv.size()); i++) {
        if (nextpv[i].getPEP2() == true) {
            for (int j = i + 1; j<int(nextpv.size()); j++) {
                double ed = nextpv[i].distP2ToP1(nextpv[j]);

                if (ed > 600 && ed < 1000) // my exit condition if it's true then check how well robot can see through that exit
                {
                    es.set(0, 0, (nextpv[i].X2() + nextpv[j].X1()) / 2, (nextpv[i].Y2() + nextpv[j].Y1()) / 2, 1);
                    elos = 0;
                    exitObject.set(nextpv[i].X2(), nextpv[i].Y2(), nextpv[j].X1(), nextpv[j].Y1(), j);
                    for (int k = i; k <= j; k++) { // is this Object too far from exit
                        if (exitObject.distP1ToPoint((nextpv[k].X2() + nextpv[k].X1()) / 2, (nextpv[k].Y2() + nextpv[k].Y1()) / 2) < 4000) {
                            int exmp = checkForIntersection(es, nextpv[k]);

                            double edist = nextpv[k].whereIsExit(exitObject); //dist btw sur mp & los on exit
                            double surdist = exitObject.whereIsExit(nextpv[k]);

                            if (exmp == 1 || surdist == 0 || (surdist > 1 && surdist < 2000) || edist > 1) {//edist==0 || ){
                                elos = 1;
                            }//if check
                        }//first if
                    }// for k
                    double angle_exit_emp = abs(exitObject.getAngleWithLine(es));

                    if (angle_exit_emp > 180) //angle_exit_emp -------robot can see through
                        angle_exit_emp = 360 - angle_exit_emp;
                    if (elos == 0 && angle_exit_emp > 50 && angle_exit_emp < 130) {

                        //exit.set(nextpv[i].X2(),nextpv[i].Y2(),nextpv[j].X1(),nextpv[j].Y1(),j);
                        if (exits.size() == 0)
                            exits.push_back(exitObject);
                        else// check for same exit
                        {
                            //double ang_exits;
                            int pushexit = 1;
                            for (int l = 0; l<int(exits.size()); l++) {
                                if (exitObject.distP1ToP1(exits[l]) < 200 || exitObject.distP2ToP2(exits[l]) < 200) {
                                    double ang_exits = abs(exitObject.getAngleWithLine(exits[l]));
                                    if (ang_exits < 60 || ang_exits > 300) {
                                        pushexit = 0;
                                        break;
                                    }//if ang
                                }//if
                            }//for l
                            if (pushexit == 1) {
                                exits.push_back(exitObject); //cout<<ang_exits<<endl;
                            }
                        }//else

                    }// if elos

                }//if ed
            }// for j
        }//if nextpv
    }// for
    return exits;
}

vector<Object> deleteSameExits(vector<Object> pexits, vector<Object> tmpexits) {
    if (pexits.size() == 0) {
        pexits = tmpexits; // for first time
    } else {
        double ang_exits;
        for (int i = 0; i<int(tmpexits.size()); i++) {
            bool newexit = true;
            for (int j = 0; j<int(pexits.size()); j++) {
                if (tmpexits[i].distP1ToP1(pexits[j]) < 400 || tmpexits[i].distP2ToP2(pexits[j]) < 400) {
                    ang_exits = abs(tmpexits[i].getAngleWithLine(pexits[j]));
                    if (ang_exits < 70 || ang_exits > 300) {// these two conditions true means they are same
                        if (tmpexits[i].length() < pexits[j].length())
                            pexits[j].set(tmpexits[i]);

                        newexit = false;
                        break;
                    }//if ang
                }//if tmp
            }// for j
            if (newexit == true)
                pexits.push_back(tmpexits[i]);
        }//for i
    }//else
    return pexits;

}

// for ASR

vector<vector<Object> > sortA2OA(vector<Object> mfis, Object rp) {
    //Object rp=crp.at(0);
    //cout<<"RP X axis"<<endl;rp.display();
    /*
            double x21=rp.X1()-rp.X2();
            double angle=acos(x21/rp.length());
            double y21=rp.Y1()-rp.Y2();
            if(y21 < 0)
            angle = 2*PI - angle;
            //angle = ((180/PI)*angle);
     */
    //cout<<" angle "<<angle<<endl;
    Point cvrob;
    cvrob.set(rp.X1(), rp.Y1());
    double angle = rp.getAngleWithXaxis();

    rp.set(0, 0, 0, 300, 0);
    mfis.push_back(rp);

    if (angle > 180)
        angle = 360 - angle;
    angle = ((PI / 180) * angle);
    //cout<<"angle with xaxis "<<angle<<endl;
    vector<Object> mfis_in_cv = xformPVIntoCV(mfis, cvrob, angle);
    vector<Object> line_ifor = getLinesIFoR(mfis_in_cv);
    //plotObjects("MFIS/mfis_in_cv",rp,line_ifor);
    //displayObjects(mfis_in_cv);
    rp.set(0, 0, 300, 0, 1);

    //cout<<"Unsorted mfis "<<endl;displayObjects(mfis);

    for (int i = 0; i<int(mfis_in_cv.size()); i++) {
        //cout<<mfis[i].getID()<<" "<<rp.getAngleWithPointForASR(mfis[i].X2(),mfis[i].Y2())<<endl;
        mfis_in_cv[i].setOrt(rp.getAngleWithPoint(mfis_in_cv[i].X2(), mfis_in_cv[i].Y2()));
    }

    vector<vector<Object> > send;

    vector<Object> tmp; //=mfis;
    /*
            //set Orientation angle
            for(int i=0;i<int(tmp.size());i++) {
                    tmp[i].setP1(0,0);
                    tmp[i].setP2(mfis[i].X2(),mfis[i].Y2());
                    mfis[i].setOrt(tmp[i].getAngleWithXaxis());
            }
     */
    std::sort(mfis_in_cv.begin(), mfis_in_cv.end(), MyDataSortPredicate);
    vector<Object> tmpmfis;
    for (int i = 0; i<int(mfis_in_cv.size()); i++) {
        for (int j = 0; j<int(mfis.size()); j++) {
            if (mfis[j].getID() == mfis_in_cv[i].getID()) {
                tmpmfis.push_back(mfis[j]);
            }
        }
    }
    mfis = tmpmfis;

    //cout<<"Sorted mfis "<<endl;displayObjects(mfis);
    tmp.clear();
    vector<Object> result;
    for (int i = 0; i<int(mfis.size() - 2); i++) {
        double d12 = mfis[i].distP2ToP1(mfis[i + 1]);
        double d13 = mfis[i].distP2ToP1(mfis[i + 2]);
        if (d12 > d13) {
            if ((d13 / d12) < 0.5) {
                //cout<<i<<" "<<d13/d12<<endl;
                result.push_back(mfis[i]);
                tmp.push_back(mfis[i + 1]);
                i++;
                if (i == int(mfis.size() - 3)) {
                    result.push_back(mfis[i + 2]);
                }
            } else {
                result.push_back(mfis[i]);
                if (i == int(mfis.size() - 3)) {
                    result.push_back(mfis[i + 1]);
                    result.push_back(mfis[i + 2]);
                }
            }
        } else {
            result.push_back(mfis[i]);
            if (i == int(mfis.size() - 3)) {
                result.push_back(mfis[i + 1]);
                result.push_back(mfis[i + 2]);
            }
        }
    }

    /*for(int i=0;i<int(mfis.size());i++) {
            cout<<mfis[i].getOrt()<<endl;
    }*/
    tmp.push_back(rp);
    send.push_back(result);
    send.push_back(tmp);
    return send;
}

vector<Object> sort(vector<Object> sur, Object rp) {
    //vector<Object> result;
    for (int i = 0; i<int(sur.size()); i++) {
        //cout<<mfis[i].getID()<<" "<<rp.getAngleWithPointForASR(mfis[i].X2(),mfis[i].Y2())<<endl;
        sur[i].setOrt(rp.getAngleWithPoint(sur[i].X2(), sur[i].Y2()));
    }
    std::sort(sur.begin(), sur.end(), MyDataSortPredicate);

    return sur;
}

vector<Object> sortAccording2Length(vector<Object> Objects) {
    std::sort(Objects.begin(), Objects.end(), sortA2L);
    return Objects;
}

vector<vector<Object> > makeASR(vector<Object> mfis, Object crp) {
    vector<Object> result;

    Object s1;
    s1.set(mfis[0]);
    bool bline = false;

    for (int i = 0; i<int(mfis.size()); i++) {
        //if(mfis[i].X1() < s1.X1() && mfis[i].Y1() < s1.Y1() && abs(mfis[i].Y1()-mfis[i].Y2()) < 200) {
        if (mfis[i].Y1() < s1.Y1() && abs(mfis[i].Y1() - mfis[i].Y2()) < 200) {
            s1.set(mfis[i]);
            bline = true;
        }
    }
    if (bline == true) {
        //cout<<"Object S1 ";s1.display();
        result.push_back(s1); //bottom boundary line
    } else {
        Object rp(0, 0, 0, 300, 0);
        result.push_back(rp);
    }

    //s1.set(mfis[0]);
    for (int i = 0; i<int(mfis.size()); i++) {
        //if(mfis[i].X1() < s1.X1() && mfis[i].Y1() > s1.Y1() && abs(mfis[i].X1()-mfis[i].X2()) < 200) {
        if (mfis[i].X1() < s1.X1() && abs(mfis[i].X1() - mfis[i].X2()) < 200) {
            s1.set(mfis[i]);
        }
    }
    //cout<<"Object S2 ";s1.display();
    result.push_back(s1); //left boundary line

    //s1.set(mfis[0]);
    for (int i = 0; i<int(mfis.size()); i++) {
        //if(mfis[i].X1() > s1.X1() && mfis[i].Y1() > s1.Y1() && abs(mfis[i].Y1()-mfis[i].Y2()) < 200) {
        if (mfis[i].Y1() > s1.Y1() && abs(mfis[i].Y1() - mfis[i].Y2()) < 200) {
            s1.set(mfis[i]);
        }
    }
    //cout<<"Object S3 ";s1.display();
    result.push_back(s1); //top boundary line

    //s1.set(mfis[0]);
    for (int i = 0; i<int(mfis.size()); i++) {
        //if(mfis[i].X2() > s1.X2() && mfis[i].Y2() < s1.Y2() && abs(mfis[i].X1()-mfis[i].X2()) < 200) {
        if (mfis[i].X2() > s1.X2() && abs(mfis[i].X1() - mfis[i].X2()) < 200) {
            s1.set(mfis[i]);
        }
    }
    //cout<<"Object S4 ";s1.display();
    result.push_back(s1); //right boundary line

    vector<Object> hnvlines, lvlines, rvlines, bhlines, thlines;
    double l2bdist1, l2bdist2;

    //finding only vertical and horizontal lines
    for (int i = 0; i<int(mfis.size()); i++) {
        /*if(abs(mfis[i].X1()-mfis[i].X2()) < 100 || abs(mfis[i].Y1()-mfis[i].Y2()) < 100) {
                hnvlines.push_back(mfis[i]);
        }
         */
        if (abs(mfis[i].X1() - mfis[i].X2()) < 100) {//is vertical
            hnvlines.push_back(mfis[i]);
            l2bdist1 = result[1].perpendicularDistOfPoint(mfis[i].X2(), mfis[i].Y2()); //l 2 left b dist
            l2bdist2 = result[3].perpendicularDistOfPoint(mfis[i].X2(), mfis[i].Y2()); //l 2 right b dist
            if (l2bdist1 < l2bdist2) {//ture means vline close to left boundary
                lvlines.push_back(mfis[i]);
            } else {//means vline close to right boundary
                rvlines.push_back(mfis[i]);
            }
        } else if (abs(mfis[i].Y1() - mfis[i].Y2()) < 100) {//is horizontal
            hnvlines.push_back(mfis[i]);
            l2bdist1 = result[0].perpendicularDistOfPoint(mfis[i].X2(), mfis[i].Y2()); //l 2 bottom b dist
            l2bdist2 = result[2].perpendicularDistOfPoint(mfis[i].X2(), mfis[i].Y2()); //l 2 top b dist
            if (l2bdist1 < l2bdist2) {//ture means hline close to bottom boundary
                bhlines.push_back(mfis[i]);
            } else {//means hline close to top boundary
                thlines.push_back(mfis[i]);
            }

        }
    }

    //sorting horizontal and vertical lines
    bhlines = sort(bhlines, crp);
    thlines = sort(thlines, crp);
    lvlines = sort(lvlines, crp);
    rvlines = sort(rvlines, crp);
    //cout<<"bh l"<<endl;displayObjects(bhlines);
    //cout<<"th l"<<endl;displayObjects(thlines);
    //cout<<"lv l"<<endl;displayObjects(lvlines);
    //cout<<"rv l"<<endl;displayObjects(rvlines);

    /*	//perpendicular distance and point coordinate checking
            Object tmp;
            Point tp=mfis[6].ppCordOfPoint(mfis[3].X2(),mfis[3].Y2());
            tmp.set(mfis[3].X2(),mfis[3].Y2(),tp.X(),tp.Y(),1);
            hnvlines.push_back(tmp);

            cout<<"length "<<tmp.length()<<" "<<mfis[6].perpendicularDistOfPoint(mfis[3].X2(),mfis[3].Y2())<<endl;
     */

    vector<vector<Object> > finalresult;
    finalresult.push_back(result);
    //finalresult.push_back(hnvlines);
    finalresult.push_back(mfis);


    return finalresult;

}

vector<Object> makeCViewASR(vector<Object> mfis)
{
	vector<Object> result;

	Object s1;
	s1.set(mfis[0]);
	bool bline=false;

	for(int i=0;i<int(mfis.size());i++) {
		if(mfis[i].X1() < s1.X1() && mfis[i].Y1() < s1.Y1() ) {
			s1.set(mfis[i]);
			bline=true;
		}
	}
	if(bline==true) {
		//cout<<"Object S1 ";s1.display();
		result.push_back(s1);
	}
	else {
		Object rp(0,0,0,300,0);
		result.push_back(rp);
	}

	//s1.set(mfis[0]);
	for(int i=0;i<int(mfis.size());i++) {
		if(mfis[i].X1() < s1.X1() && mfis[i].Y1() > s1.Y1()) {// && abs(mfis[i].X1()-mfis[i].X2()) < 200) {
			s1.set(mfis[i]);
		}
	}
	//cout<<"Object S2 ";s1.display();
	result.push_back(s1);

	//s1.set(mfis[0]);
	for(int i=0;i<int(mfis.size());i++) {
		if(mfis[i].X1() > s1.X1() && mfis[i].Y1() > s1.Y1()) {//&& abs(mfis[i].Y1()-mfis[i].Y2()) < 200) {
			s1.set(mfis[i]);
		}
	}
	//cout<<"Object S3 ";s1.display();
	result.push_back(s1);

	//s1.set(mfis[0]);
	for(int i=0;i<int(mfis.size());i++) {
		if(mfis[i].X2() > s1.X2() && mfis[i].Y2() < s1.Y2() ) {//&& abs(mfis[i].X1()-mfis[i].X2()) < 200) {
			s1.set(mfis[i]);
		}
	}
	//cout<<"Object S4 ";s1.display();
	result.push_back(s1);



	return result;

}


//asr formation using yeap's theory(paper)
vector<vector<Object> > findProbableExits(vector<Object> cv)
{
	vector<vector<Object> > result;
	vector<Object> bObjects;//Objects on asr boundary
	double distp2top1;
	//double mindist=400;
	Object exit;
	int no_exit;
	vector<Object> exits;
	cout<<"******* Finding probable exits(yeap's theory) ******* "<<endl;
	exit.set(0,0,cv[0].X1(),cv[0].Y1(),1);//first side exit
	//exits.push_back(exit);
	int exits_counter=1;
	for(int i=0;i<int(cv.size());i++) {
		if(cv[i].getPEP2() == true) { //p2 is a probable exit end(p1)
			no_exit=0;
			for(int j=i+1;j<int(cv.size());j++) {
				if(cv[j].getPEP1() == true) { //p1 is a probable exit end(p2)
					if(no_exit == 0) {
						exit.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),j);
						no_exit++;
					}
					else {
						distp2top1=cv[i].distP2ToP1(cv[j]);
						if(exit.length() > distp2top1) { //condition to get shortest exit
							exit.set(cv[i].X2(),cv[i].Y2(),cv[j].X1(),cv[j].Y1(),j);
						}
					}
				}//if cv j
			}//for j
			bObjects.push_back(cv[i]);//boundary Object having exit vertex
			i=exit.getID()-1;//for triming
			//exit.setID(exits.back().getID()+1);
			exit.setID(exits_counter);
			exits.push_back(exit);
			exits_counter++;
			//exit.display();

			//i=j-1;
		}//if cv i
		else {
			//cout<<" i "<<i<<endl;
			bObjects.push_back(cv[i]);//boundary Objects btw exits
		}
	}// for i

	exit.set(cv[int(cv.size()-1)].X2(),cv[int(cv.size()-1)].Y2(),0,0,exits.back().getID()+1);//last side exit
	//exits.push_back(exit);
	//displayObjects(exits);
	//cout<<"boundary Objects "<<endl;displayObjects(bObjects);

	result.push_back(bObjects);
	result.push_back(exits);

	//cout<<" last Object of cv "<<endl;cv.back().display();

	return result;

}

//making asr using my theory
//1st arg- previous asr
//2nd arg- current asr
//3rd arg- current robot position
/*
vector<Object> getASRObjects(vector<Object> pmfis,vector<Object> cmfis, Object crp)
{

}*/


//finding boundary Objects using Yeap's theory(occluding edges)
vector<Object> getBoundaryObjects(vector<Object> cview)//, vector<Object> asr, Object crp)
{
	vector<GroupS> cv=makeGroup(cview);
	cout<<"getBoundary Objects: group"<<endl;
	//displayGroups(cv);

	vector<Object> tmpall,tmp;
	tmpall=cv[0].getObjects();
	for(int i=0;i<int(tmpall.size());i++) {
		tmpall[i].setGID(1);
	}

	for(int i=1;i<int(cv.size()-1);i++) {
		tmp=cv[i].getObjects();
		if(tmp[0].getP1OS()==0 || tmp[tmp.size()-1].getP2OS()==0) {
			for(int j=0;j<int(tmp.size());j++) {
				tmpall.push_back(tmp[j]);
			}
		}
	}
	tmp=cv[cv.size()-1].getObjects();
	for(int i=0;i<int(tmp.size());i++) {
		tmpall.push_back(tmp[i]);
	}

	return tmpall;

}

vector<Object> updateASR(vector<Object> asr, vector<Object> cvbsur, Object crp)
{
	bool isnewline;
	vector<Object> result=asr;
	/*
	can't replace first and last line of asr by cvbsur
	have to update/extend
	or use cvbsur from mfis
	*/

	for(int k=0;k<int(result.size());k++) {
		cout<<k<<" a w rp "<<crp.getAngleWithPoint(result[k].X1(),result[k].Y1())<<endl;
	}

	for(int i=0;i<int(cvbsur.size());i++) {
		isnewline=true;
		for(int j=0;j<int(asr.size());j++) {
			if(cvbsur[i].getID() == result[j].getID()) {
				result[j].set(cvbsur[i]);
				isnewline=false;
				break;
			}
		}
		if(isnewline == true) {
			double anglewithnl=crp.getAngleWithPoint(cvbsur[i].X1(),cvbsur[i].Y1());
			for(int k=0;k<int(result.size());k++) {
				if(anglewithnl > crp.getAngleWithPoint(result[k].X1(),result[k].Y1())) {
					result.insert(result.begin()+k,cvbsur[i]);
					break;
				}
			}
			//cout<<" angle with rp "<<anglewithnl<<endl;


			//result.push_back(cvbsur[i]);
		}
	}
	return result;
}

//split pasr using crp
//replace pasr by front casr
//add bottom part
vector<Object> mergeASR(vector<Object> pasr, vector<Object> casr, Object crp)
{

	cout<<"previous all asr lines"<<endl;displayObjects(pasr);

	//crp.set(0,2000,300,2000,0);
	Point cvrob;
	cvrob.set(crp.X1(),crp.Y1());
	double angle=crp.getAngleWithXaxis();

	//crp.set(0,0,0,300,0);
	//mfis.push_back(rp);

	if(angle > 180)
	angle=360-angle;
	cout<<"angle with xaxis "<<angle<<endl;
	angle = ((PI/180)*angle);//check it..................have problem

	vector<Object> pasr_in_cv=xformPVIntoCV(pasr,cvrob,angle);
	//vector<Object> pasr_ifor=getLinesIFoR(pasr_in_cv);

	cout<<"pasr into cv "<<endl;displayObjects(pasr_in_cv);

	vector<Object> forrs,rslines, result;
	cout<<"********** getting crossed lines *********"<<endl;
	bool abovex=false;
	for(int i=0;i<int(pasr_in_cv.size());i++) {
		if((pasr_in_cv[i].X1() < 0 && pasr_in_cv[i].Y1() < 0) || (pasr_in_cv[i].X2() < 0 && pasr_in_cv[i].Y2() < 0 )) {
			result.push_back(pasr[i]);
			if(i==0) cout<<" "<<i<<endl;
		}
		else {
			if(abovex == false && pasr_in_cv[i].X1() > 0) {
				for(int i=0;i<int(casr.size());i++) {
					result.push_back(casr[i]);
				}
				abovex=true;
			}

			//forrs.push_back(pasr_in_cv[i]);
			if( pasr_in_cv[i].Y1() < 0 || pasr_in_cv[i].Y2() < 0) {
				//rslines.push_back(pasr[i]);
				result.push_back(pasr[i]);

			}
		}
	}
	cout<<"lclines "<<endl;displayObjects(result);
			cout<<endl;
/*
	for(int i=0;i<int(forrs.size());i++) {
		if((pasr_in_cv[i].X2() > 0 || pasr_in_cv[i].X1() > 0 && pasr_in_cv[i].Y2() < 0) {
				rslines.push_back(pasr[i]);

		}

	}*/

	cout<<"rclines "<<endl;displayObjects(rslines);
			cout<<endl;

	//***** adding cv lines *************
	/*for(int i=0;i<int(casr.size());i++) {
		result.push_back(casr[i]);
	}*/

	//****** adding right side lines *******
	/*for(int i=0;i<int(rslines.size());i++) {
		result.push_back(rslines[i]);
	}*/

	cout<<"************ all asr lines *********"<<endl;
	displayObjects(result);

	//plotASR("MFIS/merged-asr",crp,result,result);
	//plotObjects("MFIS/merged-asr",crp,result);
	//displayObjects(mfis_in_cv);
	//rp.set(0,0,300,0,1);
	return result;
}

vector<Object> deleteInsideObjects(vector<Object> asr, vector<Object> cvbsur, Object crp)
{
	vector<Object> bsur;
	Object tmp;
	//plotObjects("MFIS/cvbObjects",crp,cvbsur);
	cout<<" cv boundary Objects: "<<endl;
	displayObjects(cvbsur);

	vector<Object> tmpls;
	for(int i=0;i<int(cvbsur.size());i++) {
		if(cvbsur[i].length() > 300) {
			cout<<" length: "<<cvbsur[i].length()<<" ";
			cvbsur[i].display();
			tmpls.push_back(cvbsur[i]);
		}
	}

	cvbsur=tmpls;
	displayObjects(cvbsur);
	for(int i=0;i<int(cvbsur.size());i++) {
		//if(cvbsur[i].length() > 300) {
			bool notboundary=false;
			for(int j=0;j<int(asr.size());j++) {

				if(cvbsur[i].getGID()==1) {
					Point mp=asr[j].midpoint();
					tmp.set(crp.X1(),crp.Y1(),mp.X(),mp.Y(),1);
					if(checkForIntersection(cvbsur[i],tmp) == true) {
						cout<<cvbsur[i].getID()<<" "<<asr[j].getID()<<endl;
						notboundary=true;
						break;
					}
					else if(cvbsur[i].isInsideObject(asr[j], crp)==true) {
						cout<<cvbsur[i].getID()<<" "<<asr[j].getID()<<endl;
						notboundary=true;
						break;
					}
				}
				else if(cvbsur[i].isInsideObject(asr[j], crp)==true) {
					cout<<cvbsur[i].getID()<<" "<<asr[j].getID()<<endl;
					notboundary=true;
					break;
				}
			}
			if(notboundary == false) {
				bsur.push_back(cvbsur[i]);
			}
		//}
	}

	//plotObjects("MFIS/cvbObjects-after-delete",crp,bsur);
	return bsur;
}

vector<Object> findLinesAroundFreeSpace(vector<Object> cview, vector<Exit> exits) {
    vector<Object> result;
    for(int i=0;i<int(exits.size()-1);i++)
    {
            for(int j=exits[i].getP2ID()-1;j<exits[i+1].getP1ID();j++) {
                    if(cview[j].X1() > 0 && cview[j].X2() >0)
                            cview[j].setPos(1);
                    else
                            cview[j].setPos(-1);

                    cview[j].setOoPV(true);
                    result.push_back(cview[j]);
            }
    }
    //plotObjects("MFIS/freeSpace.png",result,result);
    //cout<<"lines around free space"<<endl;
    //displayObjects(result);
    return result;
}

//tag each object: 1 for right -1 for left and true for current view status
vector<Object> tagObjectsAsSide(vector<Object> cview) {
			vector<Object> result;
			for(int i=0;i<int(cview.size());i++)
			{

					if(cview[i].X1() > 0 && cview[i].X2() >0)
						cview[i].setPos(1);
					else
						cview[i].setPos(-1);

					cview[i].setOoPV(true);
					result.push_back(cview[i]);

			}
			//plotObjects("MFIS/freeSpace.png",result,result);
			//cout<<"lines around free space"<<endl;
			//displayObjects(result);
			return result;
}

//tag each object: 1 for right -1 for left and true for current view status
//tag each object's view/step number

vector<Object> tagObjectsAsSideAndViewNumber(vector<Object> cview, int v) {
    vector<Object> result;
    for (int i = 0; i<int(cview.size()); i++) {

        if (cview[i].X1() > 0 && cview[i].X2() > 0)
            cview[i].setPos(1);
        else
            cview[i].setPos(-1);

        cview[i].setOoPV(true);
        cview[i].setVN(v);
        result.push_back(cview[i]);

    }
    //special case for viewno 39 of narrow corridor
    if(result[result.size()-1].length() > 1000)
        result[result.size()-1].setPos(1);
    //plotObjects("MFIS/freeSpace.png",result,result);
    //cout<<"lines around free space"<<endl;
    //displayObjects(result);
    return result;
}


//find objects in front of current robot position and then replace them by current view's objects
//N.B: inputs are ASRs (MFIS ASR, CV ASR) and x-axis at current robot positions in MFIS.
//		output is updated ASR as well as MFIS(if we consider)
//added by
//			hossain
//			26.05.2011
vector<Object> updateMFISandASR(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects) {
	cout<<"\033[1;32m-------------------Inside updateMFISandASR module---------------------\033[0m"<<endl;
	Point cvrob;
	cvrob.set(crp.X1(),crp.Y1());
	double angle=crp.getAngleWithXaxis();

	//if(angle > 180)
	//angle=360-angle;
	cout<<"angle with xaxis "<<angle<<endl;
	angle = ((angle/180)*PI);

	vector<Object> pasr_in_cv=xformPVIntoCV(pasr,cvrob,angle);
	//vector<Object> pasr_ifor=getLinesIFoR(pasr_in_cv);
	cout<<"casr, will be added"<<endl;
	displayObjects(casr);

	//find the boundary of the current free space
	double leftlimit = 0;
	double rightlimit = 0;
	double horizon = 0;
	for(int i=0;i<int(casr.size());i++){
		if(casr[i].X1() < leftlimit)
			leftlimit=casr[i].X1();
		if(casr[i].X2() < leftlimit)
			leftlimit=casr[i].X2();

		if(casr[i].X1() > rightlimit)
			rightlimit=casr[i].X1();
		if(casr[i].X2() > rightlimit)
			rightlimit=casr[i].X2();

		if(casr[i].Y1() > horizon)
			horizon=casr[i].Y1();
		if(casr[i].Y2() > horizon)
			horizon=casr[i].Y2();

	}

	vector<Object> result;
	cout<<"********** getting crossed lines *********"<<endl;
	Object tmp;
	vector<Object> tmps;
	int lastid=pasr[pasr.size()-1].getID();
	bool abovex=false;
	bool insertnow=false;
	vector<Object> firstnlast;
	for(int i=0;i<int(pasr_in_cv.size());i++) {
		if(pasr[i].getPos() == 1) {
			insertnow = true;
			cout<<"Will be replaed soon"<<endl;
		}vector<double> findBoundariesOfCV(vector<Object> casr);
		bool outofbox=false;//check whether this object is out of the current free space
							if(pasr_in_cv[i].Y1() > horizon+2000 && pasr_in_cv[i].Y2() > horizon+2000)
								outofbox=true;//result.push_back(pasr[i]);
							else if(pasr_in_cv[i].X1() < 0 && pasr_in_cv[i].X1() < leftlimit-300){
								outofbox=true;//result.push_back(pasr[i]);
							}
							else if(pasr_in_cv[i].X1() > 0 && pasr_in_cv[i].X1() > rightlimit+300){
								outofbox=true;//result.push_back(pasr[i]);
							}

							if(outofbox == true)
								cout<<"out of box is true"<<endl;

		if(((pasr_in_cv[i].Y1() < 0) || (pasr_in_cv[i].Y2() < 0 )) && outofbox == false) {
			cout<<" "<<pasr[i].getID();
			if(insertnow == true)
			cout<<" have to insert cv"<<endl;
				if((pasr_in_cv[i].Y1() < 0 && pasr_in_cv[i].Y2() > 0)) {
					vector<double> in_point=getIntersectionPoint(pasr[i],crp);
					pasr[i].setP2(in_point[0],in_point[1]);
					//result.push_back(pasr[i]);
				}
				else if(pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() < 0) {
					vector<double> in_point=getIntersectionPoint(pasr[i],crp);
					pasr[i].setP1(in_point[0],in_point[1]);
					//result.push_back(pasr[i]);
				}

				if(abovex == false && insertnow == true) {
					cout<<"replacing the current view"<<endl;
					for(int j=0;j<int(casr.size());j++) {
						tmp=remakeLineP2(refobjects[0],refobjects[1],casr[j],lastid+1,0,refobjects[0].getKP());
						if(casr[j].getPos() == 1)
							tmp.setPos(1);
						else
							tmp.setPos(0); //0 means unknown
						if(j == 0 or j == int(casr.size()-1) )
							firstnlast.push_back(tmp);
						result.push_back(tmp);
						lastid++;
					}
					abovex=true;
					//pasr[i].setPos(1);
					result.push_back(pasr[i]);
				}
				else {
					//if(abovex == true)
						//pasr[i].setPos(1);
					result.push_back(pasr[i]);
				}

		}
		else {


			if(outofbox == true){//means push the object into global map
				if(abovex == false && insertnow == true) {
					cout<<"replacing by cv"<<endl;
					for(int j=0;j<int(casr.size());j++) {
						tmp=remakeLineP2(refobjects[0],refobjects[1],casr[j],lastid+1,0,refobjects[0].getKP());
						if(casr[j].getPos() == 1)
							tmp.setPos(1);
						else
							tmp.setPos(0);
						if(j == 0 or j == int(casr.size()-1) )
							firstnlast.push_back(tmp);
						result.push_back(tmp);
						lastid++;
					}
					abovex=true;
				}
				//if(abovex == true && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0 && pasr_in_cv[i].X1() < 0 && pasr_in_cv[i].X2() < 0  )
					//cout<<"wired";
				//else if(abovex == false && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0 && pasr_in_cv[i].X1() > 0 && pasr_in_cv[i].X2() > 0  )
					//				cout<<"wired";
				//else
					result.push_back(pasr[i]);
			}
		}
	}

	result.back().setID(lastid+1);//to get the first id for next view
	cout<<endl<<"************ all asr lines *********"<<endl;
	displayObjects(result);

	cout<<"first and last objects of current view"<<endl;
	cout<<"firstnlast line vector size: "<<firstnlast.size()<<endl;
	displayObjects(firstnlast);

	//extension of first and last lines
	vector<Object> output;
	//double angle;
	for(int i=0;i<int(result.size());i++) {
		if(result[i].getID() == firstnlast[0].getID()) {//for first line
			angle=output.back().getAngleWithLine(firstnlast[0]);
			cout<<output.back().getID()<<" first linen angle "<<angle<<endl;
			if(abs(angle) < 5)
				output.back().setP2(result[i].X2(),result[i].Y2());//edit the previous line
			else													//means current line is being merged
				output.push_back(result[i]);//else keep the current line as new line
		}
		else if(result[i].getID() == firstnlast[1].getID()) {//for last line
			angle=result[i+1].getAngleWithLine(firstnlast[1]);
			cout<<result[i+1].getID()<<" last line angle "<<angle<<endl;
			if(abs(angle) < 5){
				firstnlast[1].setP2(result[i+1].X2(),result[i+1].Y2());
					output.push_back(firstnlast[1]);
					i++;//dont need to push the next line
			}			//has been merged with the current line
			else
				output.push_back(result[i]);//else keep the current line as new line

		}
		else
			output.push_back(result[i]);

	}
	//cout<<endl<<"last id of updated MFIS "<<lastid<<endl;
	cout<<endl<<"************ all asr lines *********"<<endl;
		displayObjects(output);

	//plotASR("MFIS/merged-asr",crp,result,result);
	//plotRobotView("MFIS/merged-asr",result,result);
	//displayObjects(mfis_in_cv);
	//rp.set(0,0,300,0,1);
	//return result;
	return output;
}

//for algorithm3
//find objects in front of current robot position and then replace them by current view's objects
//N.B: inputs are ASRs (MFIS ASR, CV ASR) and x-axis at current robot positions in MFIS.
//		output is updated MFIS/ASR(if we consider)
//added by
//			hossain
//			14.06.2011

Transporter computeMap(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects, int asrno) {
    cout << "\033[1;32m-------------------Inside computeMap module---------------------\033[0m" << endl;
    vector<Object> output;
    Point cvrob;
    cvrob.set(crp.X1(), crp.Y1()); //crp: x-axis at current robot position in MFIS
    double angle = crp.getAngleWithXaxis(); //current robot x-axis and Original x-axis



    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    //cout << "pasr" << endl;
    //displayObjects(pasr);
    /*cout<<"casr"<<endl;
    displayObjects(casr);*/
    //    cout << "ref objects being used" << endl;
    //    displayObjects(refobjects);

    vector<double> boundariesOfCV = findBoundariesOfCV(casr,500);
    vector<Object> pasr_in_cv = xformPVIntoCV(pasr, cvrob, angle);
    for (int i = 0; i<int(pasr.size()); i++) {
        pasr_in_cv[i].setVN(pasr[i].getVN());
    }
    vector<Object> boundarieObjects;
    Object oneBObject;
    oneBObject.set(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
    oneBObject.set(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
    oneBObject.set(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
    //    cout<<"pasr"<<endl;
    //    displayObjects(pasr);
    //    cout << "pasr in cv" << endl;
    //displayObjects(pasr_in_cv);
    //    if(asrno == 3)
   //     plotObjects("Maps/allOnCV.png",pasr_in_cv,boundarieObjects);
    //   waitHere();

    //bool insertion = false;
    //bool insert_now = false;
    int lastid = 0;
    if (pasr.size() != 0) {
        cout << pasr[pasr.size() - 1].getID() << endl;
        lastid = pasr[pasr.size() - 1].getID();
        cout << pasr[pasr.size() - 1].getID() << endl;
    }
    //int cv_no=casr[0].getVN();//for every step updating
    //crp.display();
    int cv_no = crp.getID() + 1; //for limiting updating

    vector<Object> targetObjectsInCV = findTargetObjects(casr);
    vector<Object> targetObjectsForNextStep;

    if (pasr.size() == 0) {//for those current asr which one has no objects yet
        Object tmp;
        vector<Object> result;

        for (unsigned int j = 0; j < casr.size(); j++) {
            tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
            if (casr[j].getPos() == 1)
                tmp.setPos(1);
            else
                tmp.setPos(0);
            tmp.setASRNo(asrno);
            tmp.setOoPV(true); //tag whether it's belong to previous view
            tmp.setVN(cv_no); //tag the view no
            result.push_back(tmp);

            for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                    targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                    targetObjectsForNextStep.back().setID(lastid + 1);
                    break;
                }
            }

            lastid++;
        }
        Transporter package;
        package.setView(result);
        package.setTargetObjects(targetObjectsForNextStep);
        return package;
    }

    cout << "cv no " << cv_no << endl;
    vector<int> insertion_start;
    vector<int> insertion_end;
    vector<int> exp_line_left;
    vector<int> exp_line_right;
    vector<Object> overlappedObjects;
    //bool first_delete=false;
    for (int i = 0; i<int(pasr_in_cv.size()); i++) {
        if (pasr_in_cv[i].Y1() < 0 && pasr_in_cv[i].Y2() > 0) {
            pasr[i].display();
            pasr_in_cv[i].display();
        }
        //to find starting and ending point of new views
        if (pasr_in_cv[i].X1() < 0 || pasr_in_cv[i].X2() < 0) {

            if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() < 0 && pasr_in_cv[i].Y2() > 0) {//to find expendable object
                //insertion_start.push_back(pasr[i].getID());
                cout << "expandable object on LEFT side" << endl;
                exp_line_left.push_back(pasr[i].getID());
                pasr[i].setPos(-1); //just to make sure
                //cout<<"\033[1;32m-------------------need to expand left object---------------------\033[0m"<<endl;
            }
        }
        //to find new lines insertion point


        if (pasr_in_cv[i].X1() > 0 || pasr_in_cv[i].X2() > 0) {

            if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() < 0) {//to find expendable object
                //insertion_start.push_back(pasr[i-1].getID());
                cout << "expandable object on RIGHT Side" << endl;
                exp_line_right.push_back(pasr[i].getID());
                pasr[i].setPos(1);
                //cout<<"\033[1;32m-------------------need to expand right object---------------------\033[0m"<<endl;
                //pasr[i].display();
            }
        }

        //finding overlapped objects with current view
        //if((cv_no-pasr[i].getVN()) < 4 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0) {
        //        if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0) {
        if (pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0 && pasr_in_cv[i].isThisInsideCV(boundariesOfCV) == true) {//&& pasr_in_cv[i].isThisOverlappingObject(casr) == true) {//pasr[i].getASRNo() == 2){//&&
            //need another condition if we want to merge cv 
            //             output.push_back(pasr[i]);//activate this if it's necessary to keep all the objects in the MFIS
            overlappedObjects.push_back(pasr[i]);
        } else {

            //pasr[i].setOoPV(false);
            output.push_back(pasr[i]);
        }


    }
    if (asrno == 3) {
        //    plotObjects("Maps/MFISrest.png",output,overlappedObjects);
        //    waitHere();
    }
    //    waitHere();
    cout << "Output after deleting objects from previous view" << endl;



    //displayObjects(output);
    if (output.size() == 0) {//no need to go below insert new lines and then send the package
        vector<Object> result;
        Object tmp;
        for (unsigned int j = 0; j < casr.size(); j++) {
            tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
            if (casr[j].getPos() == 1)
                tmp.setPos(1);
            else
                tmp.setPos(0);
            tmp.setASRNo(asrno);
            tmp.setOoPV(true); //tag whether it's belong to previous view
            tmp.setVN(cv_no); //tag the view no
            result.push_back(tmp);

            for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                    targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                    targetObjectsForNextStep.back().setID(lastid + 1);
                    break;
                }
            }

            lastid++;
        }
        result.back().setID(lastid + 1); //to get the first id for next view
        Transporter package;
        package.setView(result);
        package.setTargetObjects(targetObjectsForNextStep);
        return package;
    }

    if (insertion_start.size() < 1) {//find the insertion point of new lines
        for (int k = 0; k<int(output.size() - 1); k++) {
            //if(output[k].getOoPV() == true) {
            //	insertion_start.push_back(output[k].getID());
            if (output[k].getPos() == 1 && output[k + 1].getPos() == 1) {
                cout << "k " << k << endl;
                if (k == 0) //means have to insert at the beginning
                    insertion_start.push_back(0);
                else
                    insertion_start.push_back(output[k - 1].getID());
                break;
            }
        }
    }
    if (insertion_start.size() == 0) {//if no point found then insert at the end
        insertion_start.push_back(output.back().getID());
    }
    vector<Object> pobjects_cv = findTargetObjects(casr);
    //    cout<<"current view "<<endl;
    //    displayObjects(casr);
    //    cout<<"target objects "<<endl;
    //    displayObjects(pobjects_cv);
    cout << "Insertion point found: " << insertion_start.back() << endl;

    cout << "output size " << output.size() << endl;
    Object tmp;
    int j_start, j_end; //loop limit for insertion
    j_start = 0;
    j_end = int(casr.size());
    double shortestDBTO;

    //    int lastObjectID = output.back().getID();
    cout << endl << "Going to expand objects if necessary " << endl;
    cout << "Objects will be xpanded LEFT " << exp_line_left.size() << " RIGHT " << exp_line_right.size() << endl;

    //expend objects if necessay
    if (exp_line_left.size() > 0 || exp_line_right.size() > 0) {
        for (int i = 0; i<int(output.size()); i++) {
            if (exp_line_left.size() > 0) {//for first line
                if (exp_line_left.back() == output[i].getID()) {
                    tmp = remakeLineP2(refobjects[0], refobjects[1], casr[0], lastid + 1, 0, refobjects[0].getKP()); //checking first line
                    angle = output[i].getAngleWithLine(tmp);
                    shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                    cout << "angle for left(frist) " << angle << endl;
                    if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                        output[i].setOoPV(true); //tag whether it's belong to previous view
                        output[i].setVN(cv_no); //tag the view no
                        output[i].setP2(tmp.X2(), tmp.Y2()); //edit the line in MFIS
                        output[i].setASRNo(asrno);
                        output[i].display();
                        j_start = 1;
                    } else {
                        tmp = remakeLineP2(refobjects[0], refobjects[1], casr[1], lastid + 1, 0, refobjects[0].getKP()); //checking second line of cv
                        angle = output[i].getAngleWithLine(tmp);
                        shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                        cout << "angle for left(2nd) " << angle << " dist " << shortestDBTO << endl;
                        if ((abs(angle) < 6 && casr[1].Y1() < 300) && shortestDBTO < 500) {
                            output[i].setOoPV(true); //tag whether it's belong to previous view
                            output[i].setVN(cv_no); //tag the view no
                            output[i].setP2(tmp.X2(), tmp.Y2()); //edit the line in MFIS
                            output[i].setASRNo(asrno);

                            if (targetObjectsInCV[0].getID() == casr[1].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[0]);
                                targetObjectsForNextStep.back().setID(output[i].getID());
                            }

                            j_start = 2;
                        } else
                            j_start = 0;
                    }
                }
            } else
                j_start = 0;
            //cout<<"expan of right side "<<exp_line_right.size()<<endl;
            if (exp_line_right.size() > 0) {//for last object
                //                cout<<exp_line_right[0]<<endl;
                if (exp_line_right[0] == output[i].getID()) {
                    cout << "Right last object" << endl;
                    casr[j_end - 1].display();
                    refobjects[0].display();
                    cout << "KP " << refobjects[0].getKP() << endl;

                    tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j_end - 1], lastid + 1, 0, refobjects[0].getKP());
                    angle = output[i].getAngleWithLine(tmp);
                    shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                    cout << "angle for right(last) " << angle << endl;
                    if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                        output[i].setOoPV(true); //tag whether it's belong to previous view
                        output[i].setVN(cv_no); //tag the view no
                        output[i].setP1(tmp.X1(), tmp.Y1()); //edit the previous line
                        output[i].setASRNo(asrno);
                        cout << "right(last) expanded" << endl;
                        j_end = j_end - 1;
                    } else {
                        cout << "Current objects" << endl;
                        casr[j_end - 2].display();
                        tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j_end - 2], lastid + 1, 0, refobjects[0].getKP());
                        angle = output[i].getAngleWithLine(tmp);
                        shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                        cout << "angle for right(2nd last) " << angle << " dist " << shortestDBTO << endl;

                        if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                            output[i].setOoPV(true); //tag whether it's belong to previous view
                            output[i].setVN(cv_no); //tag the view no
                            output[i].setP1(tmp.X1(), tmp.Y1()); //edit the previous line
                            output[i].setASRNo(asrno);

                            if (targetObjectsInCV.back().getID() == casr[j_end - 2].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV.back());
                                targetObjectsForNextStep.back().setID(output[i].getID());
                            }
                            cout << "right(2nd last) expanded" << endl;

                            j_end = j_end - 2;
                        } else
                            j_end = j_end;
                    }
                }
            } else
                j_end = j_end;

        }
    } else {
        j_start = 0;
        j_end = j_end;
    }
    //    waitHere();
    cout << "Insertion begins" << endl;
    vector<Object> result;
    vector<Object> resetTargetObjects;
    cout << "Going to insert at " << insertion_start.back() << endl;
    //cout<<"j start and end "<<j_start<<" "<<j_end<<endl;
    //cout<<"output "<<output.size()<<endl;
    //displayObjects(output);
    for (int i = 0; i<int(output.size()); i++) { //new lines insertion
        if (output[i].getID() == insertion_start.back() or insertion_start.back() == 0) {

            output[i].setOoPV(false);
            if (exp_line_left.size() > 0)
                if (exp_line_left.back() == output[i].getID())
                    output[i].setOoPV(true); //special case when first line extended means it's still a last view object
            if (exp_line_right.size() > 0)
                if (exp_line_right[0] == output[i].getID())//special case when last object extended means it's still last view object
                    output[i].setOoPV(true);
            if (insertion_start.back() > 0)//otherwise have to insert this object after inserting new objects
                result.push_back(output[i]);
            refobjects[0].display();
            refobjects[1].display();
            cout << "KP " << refobjects[0].getKP() << endl;
            if (insertion_start.back() == 486) {
                vector<Object> tmptests;
                Object tmptest;
                tmptest.set(getObject(pasr, refobjects[0].getID()));
                tmptests.push_back(tmptest);
                //tmptest.display();
                tmptests.push_back(refobjects[1]);
                displayObjects(tmptests);
                plotObjects("MFIS/output.png", output, tmptests);
            }
            for (int j = j_start; j < j_end; j++) {
                tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
                if (casr[j].getPos() == 1)
                    tmp.setPos(1);
                else
                    tmp.setPos(0);
                tmp.setASRNo(asrno);
                tmp.setOoPV(true); //tag whether it's belong to previous view
                tmp.setVN(cv_no); //tag the view no
                result.push_back(tmp);

                for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                    if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                        targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                        targetObjectsForNextStep.back().setID(lastid + 1);
                        break;
                    }
                }

                lastid++;
            }
            if (insertion_start.back() == 0)
                result.push_back(output[i]);
        } else {
            output[i].setOoPV(false);
            if (exp_line_left.size() > 0)
                if (exp_line_left.back() == output[i].getID())
                    output[i].setOoPV(true); //special case when first line extended means it's still a last view object
            if (exp_line_right.size() > 0)
                if (exp_line_right[0] == output[i].getID())//special case when last object extended means it's still last view object
                    output[i].setOoPV(true);
            result.push_back(output[i]);
        }

    }
    result.back().setID(lastid + 1); //to get the first id for next view
    cout << "computed map" << endl;
    //    displayObjects(result);
    Transporter package;
    package.setView(result);
    package.setTargetObjects(targetObjectsForNextStep);
    return package;
}

//to split new asr after crossing the exit(findShortestExits module is being used to find exits)

double splitToFromNewASR(vector<Object> exitsinmfis, Object robotpath) {
    for (int i = 0; i<int(exitsinmfis.size()); i++) {
        if (checkForIntersection(exitsinmfis[i], robotpath) == 1 && exitsinmfis[i].length() < 1000) {
            cout<<"Exit Length "<<exitsinmfis[i].length()<<endl;
            return i;
        }

    }
    //exitsinmfis.push_back(robotpath);
    //plotASR("MFIS/NewASR.png",oldasr);
    return -1;
}

//when robot crosses a exit then it needs to split the current asr
vector<ASR> makeNewASR(std::vector<ASR> allASRs,vector<Object> xexits) {
    vector<Object> currentASR;
    currentASR=allASRs.back().getASRObjects();

    double angle = xexits.back().getAngleWithXaxis();
    angle = ((angle/180)*PI);
    Point mp;
    mp.set(xexits.back().mpX(),xexits.back().mpY());
    vector<Object> tmp=xformPVIntoCV(currentASR,mp,angle);
    vector<Object> oldasr,newasr;
    for(int j=0;j<int(tmp.size());j++) {
        if(tmp[j].Y1() < 0 or tmp[j].Y2() < 0)
            oldasr.push_back(currentASR[j]);
        else
            if(currentASR[j].getOoPV() == true)
                newasr.push_back(currentASR[j]);
    }
    allASRs.back().setASRObjects(oldasr);
    allASRs.back().setASRExit2(xexits.back());
    cout<<"last ASR ID: "<<allASRs.back().getASRID()<<endl;
    
    //ploting new completed ASR
    char asrname[100];
      sprintf(asrname, "%s%d%s", "MFIS/ASR-", allASRs.back().getASRID(), ".png");
//      plotSingleASR(asrname,allASRs.back());

    ASR lastASR;
    lastASR.setASRID(allASRs.back().getASRID()+1);
    lastASR.setASRObjects(newasr);
    lastASR.setASRExit1(xexits.back());
    lastASR.setASRExit2(xexits.back());//temporary for printing only
    allASRs.push_back(lastASR);

    return allASRs;
}

//when robot crosses a exit then it needs to split the current asr
//it gets the current ASR then splits to complete a ASR and to form new current ASR
//
ASRNetwork splitCurrentASR(ASRNetwork pm, vector<Object> xexits) {
    cout<<"Splitting Current ASR "<<endl;
   // cout<<"Current ASR id "<<pm.getCurrentASR().getASRID()<<endl;
    vector<Object> currentASR;
    currentASR=pm.getCurrentASR().getASRObjects(); //current ASR objects only
//     if(pm.getCurrentASR().getASRID() == 8)
//    plotRobotView("Maps/CurrentASR.png",xexits,currentASR);
    cout<<"Before spliting "<<endl;
    displayObjects(currentASR);
//    
//    if(pm.getCurrentASR().getASRID() == 8)
//        waitHere();
    
    int currentASRNumber = currentASR.back().getASRNo();

    double angle = xexits.back().getAngleWithXaxis();
    angle = ((angle/180)*PI);
    Point mp;
    mp.set(xexits.back().mpX(),xexits.back().mpY());
    vector<Object> tmp=xformPVIntoCV(currentASR,mp,angle);
    vector<Object> oldasr,newasr;
    
    if(pm.getCurrentASR().getASRID() == 88) { //special case for ASR8 of set19
        for(int i =0; i<currentASR.size(); i++) {

            if(i >6 && i < 16) 
                newasr.push_back(currentASR[i]);
            else 
                oldasr.push_back(currentASR[i]);
        }
    }
    else {
    for(int j=0;j<int(tmp.size());j++) {
        if(tmp[j].Y1() < 0 or tmp[j].Y2() < 0)
            oldasr.push_back(currentASR[j]);
        else {
//            if(currentASR[j].getOoPV() == true)
                newasr.push_back(currentASR[j]);
                newasr.back().setASRNo(currentASRNumber+1);
//            else 
//                oldasr.push_back(currentASR[j]); //some lines which r not from last updating view but above xaxis
        }        
    }
    }
   

    ASR lastASR;
    lastASR.setASRID(pm.getCurrentASR().getASRID());
    lastASR.setASRObjects(oldasr);
    lastASR.setASRExit1(pm.getCurrentASR().getASRExit1());
    lastASR.setASRExit2(xexits.back());
    lastASR.replaceTheWholeRoute(pm.getCurrentASR().getRoute());
    lastASR.setLimitingPoints(pm.getCurrentASR().getLimitingPoints());
    lastASR.setLineOfSitePoints(pm.getCurrentASR().getLineOfSitePoints());
    pm.addASR(lastASR);

    
    //ploting new completed ASR
    char asrname[100];
    sprintf(asrname, "%s%d%s", "Maps/ASR-", pm.getCurrentASR().getASRID(), ".png");
   // cout<<"asr name "<<asrname<<endl;
//    plotObjects(asrname,lastASR.getRoute(),oldasr);
    plotSingleASR(asrname,pm.getASRs().back());

    ASR newCurrentASR;
    newCurrentASR.setASRID(pm.getCurrentASR().getASRID()+1);
    newCurrentASR.setASRObjects(newasr);
    newCurrentASR.setASRExit1(xexits.back());
    newCurrentASR.setASRExit2(xexits.back());//temporary for printing only
    pm.setCurrentASR(newCurrentASR);
//plotObjects("MFIS/CurrentASR1.png",newasr,newasr);
 //   char wait[100];
  //  cin>>wait;
    return pm;
}

//it will be called after computing at least 2 ASRs
//then it will compare the distance between last complete ASR's exit and others previous ASR's exits
//if the below conditions are true then previous ASR will be replaced by the last computed ASR
//and the ASR followed by the previous ASR will be replaced by the growing ASR(incomplete/present ASR)
vector<ASR> closeTheLoopUsingExit(vector<ASR> allASRs) {
    ASR lastCompleteASR = allASRs[allASRs.size()-2];
    for(int i=0;i<int(allASRs.size()-2);i++) {
        Object oldASRExit=allASRs[i].getASRExit2();
        if(oldASRExit.distMPToPoint(lastCompleteASR.getASRExit2().mpX(),lastCompleteASR.getASRExit2().mpY()) < 1500) {
            oldASRExit=allASRs[i].getASRExit1();
            if(oldASRExit.distMPToPoint(lastCompleteASR.getASRExit1().mpX(),lastCompleteASR.getASRExit1().mpY()) < 5000)
            {
                allASRs[i].replaceASR(lastCompleteASR);
                //lastASR.replaceTheWholeRoute(allASRs[i+1].getRoute());
                allASRs[i+1].setASRObjects(allASRs.back().getASRObjects());
                
                return allASRs;
            }
            //return allASRs;
        }

    }
    return allASRs;
}

ASRNetwork closeTheLoopUsingBoundary(ASRNetwork perceptualMap, vector<Object> cv, vector<Exit> exits, Object crp) {
    vector<ASR> ASRs=perceptualMap.getASRs();
    vector<Object> cvInsideCurrentASR = findLinesAroundFreeSpace(cv, exits);
    double leftWall = 0, rightWall = 0, frontWall = 0;
    for (int i = 0; i<int(cvInsideCurrentASR.size() - 1); i++) {//-1 bcz first exit is pushed into currentASR vector  //which one is not necessary to check for boundary      
        if (cvInsideCurrentASR[i].X1() < leftWall)
            leftWall = cvInsideCurrentASR[i].X1();
        if (cvInsideCurrentASR[i].X2() < leftWall)
            leftWall = cvInsideCurrentASR[i].X2();

        if (cvInsideCurrentASR[i].X1() > rightWall)
            rightWall = cvInsideCurrentASR[i].X1();
        if (cvInsideCurrentASR[i].X2() > rightWall)
            rightWall = cvInsideCurrentASR[i].X2();

        if (cvInsideCurrentASR[i].Y1() > frontWall)
            frontWall = cvInsideCurrentASR[i].Y1();
        if (cvInsideCurrentASR[i].Y2() > frontWall)
            frontWall = cvInsideCurrentASR[i].Y2();
    }
    cout << "LeftWall " << leftWall << " RightWall " << rightWall << " FrontWall " << frontWall << endl;

    Point cvrob;
    cvrob.set(crp.X1(), crp.Y1());
    double angle = crp.getAngleWithXaxis();
    //cout<<"angle with xaxis "<<angle<<endl;
    angle = ((angle / 180) * PI);
    // vector<Object> transformedCurrentASR=xformPVIntoCV(currentASR,mp,angle);
    vector<Object> transformedPreviousASR;
    int insideObjectCounter=0;

    for (int i = 0; i<int(ASRs.size() - 1); i++) {
        transformedPreviousASR = xformPVIntoCV(ASRs[i].getASRObjects(), cvrob, angle);
        //plotObjects("MFIS/insideObjects.png",transformedPreviousASR,transformedPreviousASR);
        for (int j = 0; j<int(transformedPreviousASR.size()); j++) {
            if (transformedPreviousASR[j].X1() > leftWall && transformedPreviousASR[j].X1() < rightWall && transformedPreviousASR[j].Y1() < frontWall && transformedPreviousASR[j].Y1() > 0) {
                insideObjectCounter++;
            } else if (transformedPreviousASR[j].X2() > leftWall && transformedPreviousASR[j].X2() < rightWall && transformedPreviousASR[j].Y2() < frontWall && transformedPreviousASR[j].Y1() > 0) {
                insideObjectCounter++;
            }
        }

        if (insideObjectCounter > 9) {
            // ASRs[i].setASRObjects(ASRs.back().getASRObjects());
            cout<<"asr no"<<i+1<< "Size of ASR "<<ASRs.size()<<" inside objects "<<insideObjectCounter<<endl;
            cout << "replace the asr by current asr"<<endl;
            ASRs[i].setASRObjects(ASRs.back().getASRObjects());
            ASRs[i].setASRExit1(ASRs.back().getASRExit1());
            cout<<"need to replace"<<endl;
            //char a;
            //cin >> a;
            
            perceptualMap.setASRs(ASRs);
            perceptualMap.setWhereAmI(i+1);
            perceptualMap.setArrivedHome(true);
            perceptualMap.setLeftHome(false);
            perceptualMap.setLoopClosed(true);
            return perceptualMap;
        } else {
            insideObjectCounter = 0;
            
        }
    }
    //plotObjects("MFIS/convertedCurrentASR.png",transformedPreviousASR, cvInsideCurrentASR);
    //cout<<"Size of the ASRs "<<ASRs.size()<<endl;
    //cout<<"Size of the first ASR" <<ASRs[0].getASRObjects().size()<<endl;
    perceptualMap.setWhereAmI(ASRs.size());
    return perceptualMap;
}

ASRNetwork locateOldExitInCurrentASR(ASRNetwork pm,ASR oldASR, vector<Object> newExits) {
    vector<ASR> currentASRs=pm.getASRs();
    vector<Object> currentASRObjects = currentASRs.back().getASRObjects();
    vector<Object> oldASRObjects = oldASR.getASRObjects();
    Object oldExit=oldASR.getASRExit2();
   // cout<<"current ASR Objects"<<endl;
    //displayObjects(currentASRObjects);
    //cout<<"Old ASR Objects"<<endl;
    //displayObjects(oldASRObjects);
    vector<Object> tmp1,tmp2;
    //plotObjects("MFIS/old-current-objects.png",currentASRObjects,oldASRObjects);
    double angleBtwTargetObjects,firstTargetDist,secondTargetDist,angleBtwExits;
    int referencePoint = 0;
    for(int i=0;i<int(currentASRObjects.size());i++) {
        if(currentASRObjects[i].getP1OS() == 1 or currentASRObjects[i].getP2OS() == 1)
            for(int j=0;j<int(oldASRObjects.size());j++) {
                if(oldASRObjects[j].getP1OS() == 1 or oldASRObjects[j].getP2OS() == 1) {
                    angleBtwTargetObjects=abs(currentASRObjects[i].getAngleWithLine(oldASRObjects[j]));
                    if(angleBtwTargetObjects < 6 or angleBtwTargetObjects > 353) {
                        if(currentASRObjects[i].getP1OS() == 1 && oldASRObjects[j].getP1OS() == 1) {
                            firstTargetDist=currentASRObjects[i].distP1ToP1(oldASRObjects[j]);
                            referencePoint=1;
                        }
                        if(currentASRObjects[i].getP1OS() == 2 && oldASRObjects[j].getP1OS() == 2) {
                            firstTargetDist=currentASRObjects[i].distP2ToP2(oldASRObjects[j]);
                            referencePoint = 2;
                        }
                        
                        for(int k=0;k<int(newExits.size());k++) {
                            angleBtwExits=abs(oldExit.getAngleWithLine(newExits[k]));
                            if(angleBtwExits < 7 or angleBtwExits > 353) {
                                if(referencePoint == 1) {
                                    firstTargetDist = oldExit.distP1ToP1(newExits[k]);
                                    secondTargetDist = oldASRObjects[j].distP1ToP1(currentASRObjects[i]);
                                    if(abs(firstTargetDist-secondTargetDist) < 200) {
                                        cout<<endl<<"Got a probable reference Object "<<endl;
                                        Object newExit=remakeLineP2(currentASRObjects[i],oldASRObjects[j],oldExit,1,0, referencePoint);
                                        ASR cASR;
                                        cASR=pm.getASRs().back();
                                        cASR.setASRExit2(newExit);
                                        currentASRs.pop_back();
                                        currentASRs.push_back(cASR);
                                        pm.setASRs(currentASRs);
                                        pm.setOldExitRecognization(true);
                                        tmp1.push_back(currentASRObjects[i]);
                                        tmp2.push_back(oldASRObjects[j]);
                                        tmp2.push_back(oldExit);
                                        tmp1.push_back(newExits[k]);
                                        tmp1.push_back(newExit);
                                        plotObjects("MFIS/loopClosing.png",tmp1,tmp2);
                                        cout<<"distance btw two exits "<<oldExit.distP1ToP1(newExits[k])<<endl;
                                        //char aaa[100];
                                        //cin>>aaa;
                                        //plotPerceptualMapWithASRs("MFIS/cASRwithOldExit.png",pm.getASRs());
                                        //plotSingleASR("MFIS/cASRwithOldExit.png",cASR);
                                        return pm;
                                    }
                                }
                            }
                            
                        }
                        //go and find second target object
                        /*for(int k=i+1;k<int(currentASRObjects.size());k++) {
                            if(currentASRObjects[k].getP1OS() == 1 or currentASRObjects[k].getP2OS() == 1)
                                 for(int l=0;l<int(oldASRObjects.size());l++) {
                                     if((oldASRObjects[l].getP1OS() == 1 or oldASRObjects[l].getP2OS() == 1) && l != j) {
                                        angleBtwTargetObjects=abs(currentASRObjects[k].getAngleWithLine(oldASRObjects[l]));
                                        if(angleBtwTargetObjects < 6 or angleBtwTargetObjects > 353)
                                            if(currentASRObjects[k].getP1OS() == 1 && oldASRObjects[l].getP1OS() == 1)
                                                secondTargetDist=currentASRObjects[k].distP1ToP1(oldASRObjects[l]);
                                            if(currentASRObjects[k].getP1OS() == 2 && oldASRObjects[l].getP1OS() == 2)
                                                secondTargetDist=currentASRObjects[k].distP2ToP2(oldASRObjects[l]);
                                            if(abs(firstTargetDist-secondTargetDist) < 200) {
                                                Object newExit=remakeLineP2(currentASRObjects[i],oldASRObjects[j],oldExit,1,0, referencePoint);
                                                pm.getASRs().back().setASRExit2(newExit);
                                                cout<<endl<<"Got a probable reference Object "<<endl;
                                                vector<Object> tmpexits;
                                                tmpexits.push_back(newExit);
                                                tmp1.push_back(currentASRObjects[k]);
                                                tmp2.push_back(oldASRObjects[l]);
                                                tmp1.push_back(newExit);
                                                //plotObjects("MFIS/CurrentASR.png",pm.getASRs().back().getASRObjects(),tmpexits);
                                                plotObjects("MFIS/currentASR.png",tmp1,tmp2);
                                                return pm;
                                            }
                                     }
                                 }
                        }*///for k*/
                    }
                }
            }
    }
    return pm;
}

vector<ASR> splitASR(vector<Object> allobjects, int asrno, vector<Object> xexits) {//vector<Object> shortestexitsinmfis,vector<Object> refobjects,Object robotpath) {
    vector<ASR> result;
    ASR oneasr;
    vector<Object> objs;

    for (int i = 1; i <= asrno; i++) {
        for (int j = 0; j<int(allobjects.size()); j++) {
            if (allobjects[j].getASRNo() == i) {
                objs.push_back(allobjects[j]);
            }
        }
        oneasr.setASRObjects(objs);
        oneasr.setASRExit1(xexits[i - 1]);
        objs.clear();
        oneasr.setASRID(i);
        result.push_back(oneasr);
    }
    if (xexits.size() > 1) {
        for (int i = 1; i<int(xexits.size()); i++) {
            result[i - 1].setASRExit2(xexits[i]);
            double angle = xexits[i].getAngleWithXaxis();
            angle = ((angle / 180) * PI);
            Point mp;
            mp.set(xexits[i].mpX(), xexits[i].mpY());
            vector<Object> asrObjects = result[i - 1].getASRObjects();
            vector<Object> tmp = xformPVIntoCV(asrObjects, mp, angle);
            vector<Object> pushThis;
            for (int j = 0; j<int(tmp.size()); j++) {
                if (tmp[j].Y1() < 0 or tmp[j].Y2() < 0)
                    pushThis.push_back(asrObjects[j]);
            }
            result[i - 1].setASRObjects(pushThis);
        }
    }

    return result;
}

Transporter computeMapAsRobotView(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects, int asrno) {
    cout << "\033[1;32m-------------------Inside computeMap module---------------------\033[0m" << endl;
    vector<Object> output;
    Point cvrob;
    cvrob.set(crp.X1(), crp.Y1()); //crp: x-axis at current robot position in MFIS
    double angle = crp.getAngleWithXaxis(); //current robot x-axis and Original x-axis



    cout << "angle with xaxis " << angle << endl;
    angle = ((angle / 180) * PI);

    //cout << "pasr" << endl;
    //displayObjects(pasr);
    /*cout<<"casr"<<endl;
    displayObjects(casr);*/
    //    cout << "ref objects being used" << endl;
    //    displayObjects(refobjects);

    vector<double> boundariesOfCV = findBoundariesOfCV(casr,500);
    vector<Object> pasr_in_cv = xformPVIntoCV(pasr, cvrob, angle);
    for (int i = 0; i<int(pasr.size()); i++) {
        pasr_in_cv[i].setVN(pasr[i].getVN());
    }
    vector<Object> boundarieObjects;
    Object oneBObject;
    oneBObject.set(boundariesOfCV[0], 0, boundariesOfCV[0], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
    oneBObject.set(boundariesOfCV[0], boundariesOfCV[2], boundariesOfCV[1], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
    oneBObject.set(boundariesOfCV[1], 0, boundariesOfCV[1], boundariesOfCV[2], 1);
    boundarieObjects.push_back(oneBObject);
 
    int lastid = 0;
    if (pasr.size() != 0) {
        cout << pasr[pasr.size() - 1].getID() << endl;
        lastid = pasr[pasr.size() - 1].getID();
        cout << pasr[pasr.size() - 1].getID() << endl;
    }

    int cv_no = crp.getID() + 1; //for limiting updating

    vector<Object> targetObjectsInCV = findTargetObjects(casr);
    vector<Object> targetObjectsForNextStep;

    if (pasr.size() == 0) {//for those current asr which one has no objects yet
        Object tmp;
        vector<Object> result;

        for (unsigned int j = 0; j < casr.size(); j++) {
            tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
            if (casr[j].getPos() == 1)
                tmp.setPos(1);
            else
                tmp.setPos(0);
            tmp.setASRNo(asrno);
            tmp.setOoPV(true); //tag whether it's belong to previous view
            tmp.setVN(cv_no); //tag the view no
            result.push_back(tmp);

            for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                    targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                    targetObjectsForNextStep.back().setID(lastid + 1);
                    break;
                }
            }

            lastid++;
        }
        Transporter package;
        package.setView(result);
        package.setTargetObjects(targetObjectsForNextStep);
        return package;
    }

    cout << "cv no " << cv_no << endl;
    vector<int> insertion_start;
    vector<int> insertion_end;
    vector<int> exp_line_left;
    vector<int> exp_line_right;
    vector<Object> overlappedObjects;
    vector<Object> objectsOutOfCV;
    //bool first_delete=false;
    for (int i = 0; i<int(pasr_in_cv.size()); i++) {
        if (pasr_in_cv[i].Y1() < 0 && pasr_in_cv[i].Y2() > 0) {
            pasr[i].display();
            pasr_in_cv[i].display();
        }
        //to find starting and ending point of new views
        if (pasr_in_cv[i].X1() < 0 || pasr_in_cv[i].X2() < 0) {

            if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() < 0 && pasr_in_cv[i].Y2() > 0) {//to find expendable object
                //insertion_start.push_back(pasr[i].getID());
                cout << "expandable object on LEFT side" << endl;
                exp_line_left.push_back(pasr[i].getID());
                pasr[i].setPos(-1); //just to make sure
                //cout<<"\033[1;32m-------------------need to expand left object---------------------\033[0m"<<endl;
            }
        }
        //to find new lines insertion point


        if (pasr_in_cv[i].X1() > 0 || pasr_in_cv[i].X2() > 0) {

            if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() < 0) {//to find expendable object
                //insertion_start.push_back(pasr[i-1].getID());
                cout << "expandable object on RIGHT Side" << endl;
                exp_line_right.push_back(pasr[i].getID());
                pasr[i].setPos(1);
                //cout<<"\033[1;32m-------------------need to expand right object---------------------\033[0m"<<endl;
                //pasr[i].display();
            }
        }

        //finding overlapped objects with current view
        //if((cv_no-pasr[i].getVN()) < 4 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0) {
        //        if (pasr[i].getOoPV() == 1 && pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0) {
        if (pasr_in_cv[i].Y1() > 0 && pasr_in_cv[i].Y2() > 0) {// && pasr_in_cv[i].isThisInsideCV(boundariesOfCV) == true) {//&& pasr_in_cv[i].isThisOverlappingObject(casr) == true) {//pasr[i].getASRNo() == 2){//&&
            //need another condition if we want to merge cv 
            //             output.push_back(pasr[i]);//activate this if it's necessary to keep all the objects in the MFIS
            if(pasr_in_cv[i].isThisInsideCV(boundariesOfCV) == true)
                overlappedObjects.push_back(pasr[i]);
            else 
                objectsOutOfCV.push_back(pasr[i]);
        } else {

            //pasr[i].setOoPV(false);
            output.push_back(pasr[i]);
        }


    }
    cout<<"objects out of cv: "<<objectsOutOfCV.size()<<endl;
    vector<Object> objectsOutOfCVInCV;
    for(unsigned int i = 0; i<objectsOutOfCV.size();i++) {
        objectsOutOfCVInCV.push_back( remakeLineP2(refobjects[1], refobjects[0], objectsOutOfCV[i], casr.back().getID()+1, 0, refobjects[0].getKP()));
        //objectsOutOfCVInCV(casr.back().getID()+1);
        casr.push_back(objectsOutOfCVInCV.back());//add to current view
    }
    //plotObjects("Maps/outcvObjects.png",objectsOutOfCVInCV,casr);
    
    Object originToP1Object;
    Object robotdir;
    robotdir.set(0,0,0,500,1);
    
    //find orientation angle of all objects w r t x axis to arrange from Left to right
    double oriAngle;
    for(unsigned int i = 0;i<casr.size();i++) {
        originToP1Object.set(0,0,casr[i].X1(),casr[i].Y1(),1);
        oriAngle = robotdir.getAngleWithLine(originToP1Object);
        casr[i].setOrt(oriAngle);
    }
    cout<<endl<<endl<<"Current view - Number of objects: "<<casr.size()<<endl<<endl;
    displayObjects(casr);
    //sorting current view objects (including new objects which are out of cv from previous view) from left to right
    std::sort(casr.begin(),casr.end(),sortA2OrtAngleL2R);
    
    //setting object id according to their position from left to right
    for(unsigned int i = 0; i<casr.size(); i++) {
        casr[i].setID(i+1);
    }
    cout<<endl<<endl<<"Current view - Number of objects: "<<casr.size()<<endl<<endl;
    displayObjects(casr);
    //waitHere();
    
    
    
    if (asrno == 3) {
        //    plotObjects("Maps/MFISrest.png",output,overlappedObjects);
        //    waitHere();
    }
    //    waitHere();
    cout << "Output after deleting objects from previous view" << endl;



    //displayObjects(output);
    if (output.size() == 0) {//no need to go below insert new lines and then send the package
        vector<Object> result;
        Object tmp;
        for (unsigned int j = 0; j < casr.size(); j++) {
            tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
            if (casr[j].getPos() == 1)
                tmp.setPos(1);
            else
                tmp.setPos(0);
            tmp.setASRNo(asrno);
            tmp.setOoPV(true); //tag whether it's belong to previous view
            tmp.setVN(cv_no); //tag the view no
            result.push_back(tmp);

            for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                    targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                    targetObjectsForNextStep.back().setID(lastid + 1);
                    break;
                }
            }

            lastid++;
        }
        result.back().setID(lastid + 1); //to get the first id for next view
        Transporter package;
        package.setView(result);
        package.setTargetObjects(targetObjectsForNextStep);
        return package;
    }

    if (insertion_start.size() < 1) {//find the insertion point of new lines
        for (int k = 0; k<int(output.size() - 1); k++) {
            //if(output[k].getOoPV() == true) {
            //	insertion_start.push_back(output[k].getID());
            if (output[k].getPos() == 1 && output[k + 1].getPos() == 1) {
                cout << "k " << k << endl;
                if (k == 0) //means have to insert at the beginning
                    insertion_start.push_back(0);
                else
                    insertion_start.push_back(output[k - 1].getID());
                break;
            }
        }
    }
    if (insertion_start.size() == 0) {//if no point found then insert at the end
        insertion_start.push_back(output.back().getID());
    }
    vector<Object> pobjects_cv = findTargetObjects(casr);
    //    cout<<"current view "<<endl;
    //    displayObjects(casr);
    //    cout<<"target objects "<<endl;
    //    displayObjects(pobjects_cv);
    cout << "Insertion point found: " << insertion_start.back() << endl;

    cout << "output size " << output.size() << endl;
    Object tmp;
    int j_start, j_end; //loop limit for insertion
    j_start = 0;
    j_end = int(casr.size());
    double shortestDBTO;

    //    int lastObjectID = output.back().getID();
    cout << endl << "Going to expand objects if necessary " << endl;
    cout << "Objects will be xpanded LEFT " << exp_line_left.size() << " RIGHT " << exp_line_right.size() << endl;

    //expend objects if necessay
    if (exp_line_left.size() > 0 || exp_line_right.size() > 0) {
        for (int i = 0; i<int(output.size()); i++) {
            if (exp_line_left.size() > 0) {//for first line
                if (exp_line_left.back() == output[i].getID()) {
                    tmp = remakeLineP2(refobjects[0], refobjects[1], casr[0], lastid + 1, 0, refobjects[0].getKP()); //checking first line
                    angle = output[i].getAngleWithLine(tmp);
                    shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                    cout << "angle for left(frist) " << angle << endl;
                    if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                        output[i].setOoPV(true); //tag whether it's belong to previous view
                        output[i].setVN(cv_no); //tag the view no
                        output[i].setP2(tmp.X2(), tmp.Y2()); //edit the line in MFIS
                        output[i].setASRNo(asrno);
                        output[i].display();
                        j_start = 1;
                    } else {
                        tmp = remakeLineP2(refobjects[0], refobjects[1], casr[1], lastid + 1, 0, refobjects[0].getKP()); //checking second line of cv
                        angle = output[i].getAngleWithLine(tmp);
                        shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                        cout << "angle for left(2nd) " << angle << " dist " << shortestDBTO << endl;
                        if ((abs(angle) < 6 && casr[1].Y1() < 300) && shortestDBTO < 500) {
                            output[i].setOoPV(true); //tag whether it's belong to previous view
                            output[i].setVN(cv_no); //tag the view no
                            output[i].setP2(tmp.X2(), tmp.Y2()); //edit the line in MFIS
                            output[i].setASRNo(asrno);

                            if (targetObjectsInCV[0].getID() == casr[1].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV[0]);
                                targetObjectsForNextStep.back().setID(output[i].getID());
                            }

                            j_start = 2;
                        } else
                            j_start = 0;
                    }
                }
            } else
                j_start = 0;
            //cout<<"expan of right side "<<exp_line_right.size()<<endl;
            if (exp_line_right.size() > 0) {//for last object
                //                cout<<exp_line_right[0]<<endl;
                if (exp_line_right[0] == output[i].getID()) {
                    cout << "Right last object" << endl;
                    casr[j_end - 1].display();
                    refobjects[0].display();
                    cout << "KP " << refobjects[0].getKP() << endl;

                    tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j_end - 1], lastid + 1, 0, refobjects[0].getKP());
                    angle = output[i].getAngleWithLine(tmp);
                    shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                    cout << "angle for right(last) " << angle << endl;
                    if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                        output[i].setOoPV(true); //tag whether it's belong to previous view
                        output[i].setVN(cv_no); //tag the view no
                        output[i].setP1(tmp.X1(), tmp.Y1()); //edit the previous line
                        output[i].setASRNo(asrno);
                        cout << "right(last) expanded" << endl;
                        j_end = j_end - 1;
                    } else {
                        cout << "Current objects" << endl;
                        casr[j_end - 2].display();
                        tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j_end - 2], lastid + 1, 0, refobjects[0].getKP());
                        angle = output[i].getAngleWithLine(tmp);
                        shortestDBTO = shortestDistanceBtwTwoObjects(output[i], tmp);
                        cout << "angle for right(2nd last) " << angle << " dist " << shortestDBTO << endl;

                        if ((abs(angle) < 6 || abs(angle) > 354) && shortestDBTO < 500) {
                            output[i].setOoPV(true); //tag whether it's belong to previous view
                            output[i].setVN(cv_no); //tag the view no
                            output[i].setP1(tmp.X1(), tmp.Y1()); //edit the previous line
                            output[i].setASRNo(asrno);

                            if (targetObjectsInCV.back().getID() == casr[j_end - 2].getID()) {
                                targetObjectsForNextStep.push_back(targetObjectsInCV.back());
                                targetObjectsForNextStep.back().setID(output[i].getID());
                            }
                            cout << "right(2nd last) expanded" << endl;

                            j_end = j_end - 2;
                        } else
                            j_end = j_end;
                    }
                }
            } else
                j_end = j_end;

        }
    } else {
        j_start = 0;
        j_end = j_end;
    }
    //    waitHere();
    cout << "Insertion begins" << endl;
    vector<Object> result;
    vector<Object> resetTargetObjects;
    cout << "Going to insert at " << insertion_start.back() << endl;
    //cout<<"j start and end "<<j_start<<" "<<j_end<<endl;
    //cout<<"output "<<output.size()<<endl;
    //displayObjects(output);
    for (int i = 0; i<int(output.size()); i++) { //new lines insertion
        if (output[i].getID() == insertion_start.back() or insertion_start.back() == 0) {

            output[i].setOoPV(false);
            if (exp_line_left.size() > 0)
                if (exp_line_left.back() == output[i].getID())
                    output[i].setOoPV(true); //special case when first line extended means it's still a last view object
            if (exp_line_right.size() > 0)
                if (exp_line_right[0] == output[i].getID())//special case when last object extended means it's still last view object
                    output[i].setOoPV(true);
            if (insertion_start.back() > 0)//otherwise have to insert this object after inserting new objects
                result.push_back(output[i]);
            refobjects[0].display();
            refobjects[1].display();
            cout << "KP " << refobjects[0].getKP() << endl;
            if (insertion_start.back() == 486) {
                vector<Object> tmptests;
                Object tmptest;
                tmptest.set(getObject(pasr, refobjects[0].getID()));
                tmptests.push_back(tmptest);
                //tmptest.display();
                tmptests.push_back(refobjects[1]);
                displayObjects(tmptests);
                plotObjects("MFIS/output.png", output, tmptests);
            }
            for (int j = j_start; j < j_end; j++) {
                tmp = remakeLineP2(refobjects[0], refobjects[1], casr[j], lastid + 1, 0, refobjects[0].getKP());
                if (casr[j].getPos() == 1)
                    tmp.setPos(1);
                else
                    tmp.setPos(0);
                tmp.setASRNo(asrno);
                tmp.setOoPV(true); //tag whether it's belong to previous view
                tmp.setVN(cv_no); //tag the view no
                result.push_back(tmp);

                for (int k = 0; k<int(targetObjectsInCV.size()); k++) {
                    if (targetObjectsInCV[k].getID() == casr[j].getID()) {
                        targetObjectsForNextStep.push_back(targetObjectsInCV[k]);
                        targetObjectsForNextStep.back().setID(lastid + 1);
                        break;
                    }
                }

                lastid++;
            }
            if (insertion_start.back() == 0)
                result.push_back(output[i]);
        } else {
            output[i].setOoPV(false);
            if (exp_line_left.size() > 0)
                if (exp_line_left.back() == output[i].getID())
                    output[i].setOoPV(true); //special case when first line extended means it's still a last view object
            if (exp_line_right.size() > 0)
                if (exp_line_right[0] == output[i].getID())//special case when last object extended means it's still last view object
                    output[i].setOoPV(true);
            result.push_back(output[i]);
        }

    }
    result.back().setID(lastid + 1); //to get the first id for next view
    cout << "computed map" << endl;
    //    displayObjects(result);
    Transporter package;
    package.setView(result);
    package.setTargetObjects(targetObjectsForNextStep);
    return package;
}

void abstractASRs(vector<ASR> allASRs, vector<Object> mfis, vector<vector<Object> > wholeRouteMap, vector<Object> refObjects) {
    cout << "\033[1;32m-------------------Inside abstractASRs module---------------------\033[0m" << endl;
    vector<Object> asrObjects;
    vector<Object> firstObject;

    Object tempObj;
    vector<Object> abstractASR;
    vector<Object> exit;
    vector<Object> route;
    vector<Object> nodesOnPath;

    vector<Object> wholeRoute;

    vector<Object> dashedRoute;

    char asrFileName[100];

    MyRobot myrobot(0, 0);

    vector<Object> allASRObjects;

    cout<<endl<<endl<<"Place2"<<endl;
    displayObjects(allASRs[1].getASRObjects());
    
    for (unsigned int i = 0; i < 5; i++) {
        sprintf(asrFileName, "%s%d%s", "Maps/ASR-", i, ".png");
        plotObjectsOf3Kinds(asrFileName, allASRs[i].getASRObjects(), makeRectangle(allASRs[i].getASRExit1()), makeRectangle(allASRs[i].getASRExit2()));
        
        asrObjects = allASRs[i].getASRObjects();
        for (unsigned int k = 0; k < asrObjects.size(); k++) {
            //if (asrObjects[k].getPos() == 1) {
                tempObj.set(asrObjects[k].mpX(), asrObjects[k].mpY(), asrObjects[k + 1].mpX(), asrObjects[k + 1].mpY(), k + 1);
                abstractASR.push_back(tempObj);
                //abstractASR.push_back(asrObjects[k]);
            //}
        }
        sprintf(asrFileName, "%s%d%s", "Maps/placeBoundary-", i, ".png");
        plotObjects(asrFileName,abstractASR,abstractASR);
        abstractASR.clear();

    }
    
    
    waitHere();

//    for (unsigned int i = 2; i < 6; i++) {
//        // if (i < 6 or i > 7)
//        allASRObjects = addTwoVectorsOfObjects(allASRs[i].getASRObjects(), allASRObjects);
//    }
//    vector<Object> oldOnNew = projectingTheView(allASRs[1].getASRObjects(), refObjects[1], refObjects[0], 2);
//    //ector<Object> ASR1andASR2 = addTwoVectorsOfObjects(oldOnNew,);
//    exit = makeSquare(allASRs[1].getASRExit2());
//
//    //for exp2 
//    //exit = projectingTheView(exit,refObjects[0],refObjects[1],2);
//    //exit = addTwoVectorsOfObjects(exit,makeSquare(allASRs[1].getASRExit1()));
//    //plotObjectsOf4Kinds("Maps/ASR-22.png",allASRs[1].getASRObjects(),wholeRouteMap[1],exit,makeSquare(allASRs[1].getASRExit2()));
//
//
//    //exit = projectingTheView(exit,refObjects[1],refObjects[0],1);
//    //exit = addTwoVectorsOfObjects(exit,projectingTheView(makeSquare(allASRs[1].getASRExit1()),refObjects[1],refObjects[0],1));
//    exit = addTwoVectorsOfObjects(exit, makeSquare(allASRs[1].getASRExit1()));
//
//
//
//
//
//    ASR asr1, asr2;
//    asr1.setASRObjects(oldOnNew);
//    asr2.setASRObjects(projectingTheView(allASRs[1].getASRObjects(), refObjects[1], refObjects[0], 2));
//    ASR mergedASR = mergeOldandNewASRs(asr1, allASRs[9]);
//    //plotObjectsOf3Kinds("Maps/allASRs1.png", mergedASR.getASRObjects(), allASRObjects,allASRs[allASRs.size()-1].getASRObjects());
//    plotObjects("Maps/OldAndNewASRs.png", oldOnNew, allASRs[9].getASRObjects());
//    plotObjectsOf3Kinds("Maps/MergedASR2.png", mergedASR.getASRObjects(), exit, wholeRouteMap[10]);
//    exit.clear();
//    //waitHere();
//    //mergedASR = mergeOldandNewASRs(asr2,allASRs[7]);
//    //plotObjectsOf3Kinds("Maps/allASRs2.png", asr2.getASRObjects(), allASRObjects,mergedASR.getASRObjects());
//    waitHere();
//    
//    
//    
//    cout << "ref objects of loopclosing" << endl;
//    displayObjects(refObjects);
//
//    vector<Object> firstRobotPositionInNewASR;
//
//
//    vector<Object> oldOnCurrentASR;
    Object temp;
//
//    for (unsigned int i = 0; i < myrobot.getRobot().size(); i++) {
//        temp = remakeLineP2(refObjects[1], refObjects[0], myrobot.getRobot()[i], i + 1, 0, 1);
//        firstRobotPositionInNewASR.push_back(temp);
//    }
//
//    for (unsigned int i = 0; i < allASRs[0].getASRObjects().size(); i++) {
//        temp = remakeLineP2(refObjects[1], refObjects[0], allASRs[0].getASRObjects()[i], i + 1, 0, 1);
//        oldOnCurrentASR.push_back(temp);
//    }
//    vector<Object> mergedARS1 = mergeOldAndNewASR(oldOnCurrentASR, allASRs[allASRs.size() - 1].getASRObjects(), firstRobotPositionInNewASR,
//            allASRs[allASRs.size() - 1].getASRExit1(), allASRs[0].getASRExit2(), wholeRouteMap[0], wholeRouteMap.back(), refObjects);
//    //waitHere();    
//    plotObjectsOf3Kinds("Maps/currentAndOldASR.png", allASRs[allASRs.size() - 1].getASRObjects(), oldOnCurrentASR, firstRobotPositionInNewASR);
//
//
//
//    plotObjects("Maps/OldASR.png", allASRs[0].getASRObjects(), myrobot.getRobot());
//    plotObjects("Maps/currentASR.png", allASRs[allASRs.size() - 1].getASRObjects(), myrobot.getRobot());

    
    
    //going to process only complete ASRs(not current ASR)
    for (int i = 0; i < allASRs.size() - 1; i++) {
        for (int j = 0; j < allASRs[i].getASRObjects().size(); j++) {
            if (allASRs[i].getASRObjects()[j].length() > 500) {
                asrObjects.push_back(allASRs[i].getASRObjects()[j]);
            }
        }

        //for revisiting ASR2 through kitchen. Just adding new exit on old ASR
        if (allASRs.size() > 9 && i == 7) {
            //for exit 1
            exit.push_back(allASRs[5].getASRExit1());
            //exit.push_back(allASRs[allASRs.size() - 1].getASRExit1());
            tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[5].getASRExit1()); //left side line
            //tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[allASRs.size() - 1].getASRExit1()); //left side line
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
            exit.push_back(tempObj);
            tempObj.reverse();
            tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
            exit.push_back(tempObj);

            vector<Object> exitOnOldASR;
            for (unsigned int i = 0; i < exit.size(); i++) {
                temp = remakeLineP2(refObjects[0], refObjects[1], exit[i], i + 1, 0, 1);
                exitOnOldASR.push_back(temp);
            }
            exit = exitOnOldASR;
            //plotObjects("Maps/ModifiedASR2.png",allASRs[1].getASRObjects(),exitOnOldASR);
        }

        //for exit 1
        exit.push_back(allASRs[i].getASRExit1());
        tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[i].getASRExit1()); //left side line
        exit.push_back(tempObj);
        tempObj.reverse();
        tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
        exit.push_back(tempObj);
        tempObj.reverse();
        tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
        exit.push_back(tempObj);

        cout << endl << endl << "Objects of current ASR" << endl;
        //displayObjects(asrObjects);

        //exit1 to mpoint of first object
        abstractASR.push_back(Object(exit[1].mpX(), exit[1].mpY(), asrObjects[0].mpX(), asrObjects[0].mpY()));
        //new abstraction
        for (unsigned int k = 0; k < asrObjects.size() - 1; k++) {
            if (asrObjects[k + 1].getPos() == -1) {
                tempObj.set(asrObjects[k].mpX(), asrObjects[k].mpY(), asrObjects[k + 1].mpX(), asrObjects[k + 1].mpY(), k + 1);
                abstractASR.push_back(tempObj);
                //abstractASR.push_back(asrObjects[k]);
            }
            if (asrObjects[k + 1].getPos() == 1) {
                //for exit 2
                exit.push_back(allASRs[i].getASRExit2()); //4
                tempObj = makeLineAtPointWithObject(90, 0, 500, allASRs[i].getASRExit2()); //left side line
                exit.push_back(tempObj); //5
                tempObj.reverse();
                tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
                exit.push_back(tempObj); //6
                tempObj.reverse();
                tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
                exit.push_back(tempObj); //7

                //last leftside object to exit2
                tempObj.set(asrObjects[k].mpX(), asrObjects[k].mpY(), exit[5].mpX(), exit[5].mpY(), k + 1);
                abstractASR.push_back(tempObj);

                //exit2 to mpoint of first right side object
                tempObj.set(exit[7].mpX(), exit[7].mpY(), asrObjects[k + 1].mpX(), asrObjects[k + 1].mpY(), k + 1);
                abstractASR.push_back(tempObj);

                break;
            }
        }

        //right side objects
        for (unsigned int k = 0; k < asrObjects.size() - 1; k++) {
            if (asrObjects[k].getPos() == 1) {
                tempObj.set(asrObjects[k].mpX(), asrObjects[k].mpY(), asrObjects[k + 1].mpX(), asrObjects[k + 1].mpY(), k + 1);
                abstractASR.push_back(tempObj);
                //abstractASR.push_back(asrObjects[k]);
            }
        }
        //mpoint of last object to exit1
        abstractASR.push_back(Object(asrObjects[asrObjects.size() - 1].mpX(), asrObjects[asrObjects.size() - 1].mpY(), exit[3].mpX(), exit[3].mpY()));

        //finding node on path
        nodesOnPath.push_back(exit[2]);

        //finding route from exit1 to exit2
        for (unsigned int k = 0; k < allASRs[i].getRoute().size(); k++) {
            if (checkForIntersection(allASRs[i].getRoute()[k], exit[2]) == 0 && checkForIntersection(allASRs[i].getRoute()[k], exit[4]) == 0) {
                if (checkForIntersection(allASRs[i].getRoute()[k], exit[0]) == 0) {
                    route.push_back(allASRs[i].getRoute()[k]);

                    if (allASRs[i].getRoute()[k].distP1ToP1(nodesOnPath.back()) > 3000) {
                        nodesOnPath.push_back(allASRs[i].getRoute()[k]);
                    }
                }
            }
        }
        //route = allASRs[i].getRoute();        
        //exit1 to first routePoint
        route.push_back(Object(exit[2].mpX(), exit[2].mpY(), route[0].X1(), route[0].Y1()));
        route.push_back(Object(exit[4].mpX(), exit[4].mpY(), route[route.size() - 2].X2(), route[route.size() - 2].Y2()));

        //wholeRoute = addTwoVectorsOfObjects(wholeRoute,route);

        nodesOnPath.erase(nodesOnPath.begin());
        nodesOnPath = makeSquareAtLoSPoints(nodesOnPath);

        cout << "Printing ASR" << i + 1 << endl;
        firstObject.push_back(allASRs[i].getASRObjects()[0]);
        sprintf(asrFileName, "%s%d%s", "Maps/absASR-", i, ".png");
        //plotRobotView(asrFileName,firstObject,asrObjects);
        plotObjectsOf4Kinds(asrFileName, abstractASR, route, exit, nodesOnPath);
        sprintf(asrFileName, "%s%d%s", "Maps/ASR-", i, ".png");

        for (unsigned int k = 0; k < wholeRouteMap[i].size(); k++)
            dashedRoute = addTwoVectorsOfObjects(dashedRoute, breakTheLineInto(wholeRouteMap[i][k]));
        dashedRoute.clear();
        plotObjectsOf4Kinds(asrFileName, allASRs[i].getASRObjects(), dashedRoute, exit, mfis); //,nodesOnPath);


        dashedRoute.clear();
        firstObject.clear();
        asrObjects.clear();
        abstractASR.clear();
        exit.clear();
        route.clear();
        nodesOnPath.clear();
        // waitHere();
    }

    plotObjects("Maps/PM.png", wholeRoute, mfis);
}

vector<Object> mergeOldAndNewASR(vector<Object> oldASR, vector<Object> newASR, vector<Object> firstRPInNewASR,
        Object newASRExit, Object oldASRExit, vector<Object> oldRoute, vector<Object> newRoute, vector<Object> refObjects) {

    vector<Object> objectsBehindFirstRP;
    double y1, y2;
    for (unsigned int i = 0; i < newASR.size(); i++) {
        y1 = newASR[i].Y1() - firstRPInNewASR[6].Y1();
        y2 = newASR[i].Y2() - firstRPInNewASR[6].Y1();

        if (y1 < 0 || y2 < 0) {
            objectsBehindFirstRP.push_back(newASR[i]);
        }
    }

    //old route on newASR
    vector<Object> oldRouteOnNewASR;
    Object temp;
    for (unsigned int i = 0; i < oldRoute.size(); i++) {
        temp = remakeLineP2(refObjects[1], refObjects[0], oldRoute[i], i + 1, 0, 1);
        oldRouteOnNewASR.push_back(temp);
    }
    vector<Object> wholeNewASRRoute = addTwoVectorsOfObjects(oldRouteOnNewASR, newRoute);

    vector<Object> mergedASR = addTwoVectorsOfObjects(objectsBehindFirstRP, oldASR);

    vector<Object> exit;
    Object tempObj;
    //for exit 1
    exit.push_back(newASRExit);
    tempObj = makeLineAtPointWithObject(90, 0, 500, newASRExit); //left side line
    exit.push_back(tempObj);
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
    exit.push_back(tempObj);
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
    exit.push_back(tempObj);

    vector<Object> dashedNewRoute;
    for (unsigned int i = 0; i < newRoute.size(); i++) {
        dashedNewRoute = addTwoVectorsOfObjects(dashedNewRoute, breakTheLineInto(newRoute[i]));
    }
    plotObjectsOf3Kinds("Maps/NewASR.png", newASR, exit, dashedNewRoute);


    Object oldExitOnNewASR = remakeLineP2(refObjects[1], refObjects[0], oldASRExit, 2, 0, 1);
    //for exit 2
    exit.push_back(oldExitOnNewASR); //4
    tempObj = makeLineAtPointWithObject(90, 0, 500, oldExitOnNewASR); //left side line
    exit.push_back(tempObj); //5
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(90, 0, 1000, tempObj); //parallel line of original exit
    exit.push_back(tempObj); //6
    tempObj.reverse();
    tempObj = makeLineAtPointWithObject(90, 0, 500, tempObj); //right side line
    exit.push_back(tempObj); //7

    vector<Object> dashedRoute;
    for (unsigned int i = 0; i < wholeNewASRRoute.size(); i++) {
        dashedRoute = addTwoVectorsOfObjects(dashedRoute, breakTheLineInto(wholeNewASRRoute[i]));
    }

    plotObjectsOf3Kinds("Maps/MergedOldAndNewASR.png", mergedASR, exit, dashedRoute);

    return mergedASR;
}

void boundaryOfThePlace(vector<ASR> allASRs, vector<Object> mfis, vector<vector<Object> > wholeRouteMap, 
                                        vector<Object> refObjects, vector<Object> crossedExit) {
    cout << "\033[1;32m-------------------Inside abstractASRs module---------------------\033[0m" << endl;
    vector<Object> asrObjects;
    vector<Object> firstObject;

    Object tempObj;
    vector<Object> abstractASR;
    vector<Object> exit;
    vector<Object> route;
    vector<Object> nodesOnPath;

    vector<Object> wholeRoute;

    vector<Object> dashedRoute;

    char asrFileName[100];

    MyRobot myrobot(0, 0);

    vector<Object> allASRObjects;

//    cout<<endl<<endl<<"Place2"<<endl;
//    displayObjects(allASRs[1].getASRObjects());

    Object temp;

    vector<Object> allExits;
    vector<Object> originToPEP;
    
    
    //going to process only complete ASRs(not current ASR)
    for (int i = 0; i < allASRs.size(); i++) {
        for (int j = 0; j < allASRs[i].getASRObjects().size(); j++) {
            //if (allASRs[i].getASRObjects()[j].length() > 500) {
                asrObjects.push_back(allASRs[i].getASRObjects()[j]);
//                if(allASRs[i].getASRObjects()[j].getPEP2() == 1) {
//                    originToPEP.push_back(Object(0,0,allASRs[i].getASRObjects()[j].X2(),allASRs[i].getASRObjects()[j].Y2()));
//                    cout<<" "<<j<<endl;
//                }
            //}
        }
        cout<<"Places "<<i+1<<endl;
        //displayObjects(allASRs[i].getASRObjects());
        allExits = convertExitToObject(findShortestGap(allASRs[i].getASRObjects()));
        //waitHere();
                
        exit = makeRectangle(allASRs[i].getASRExit1());

        cout << endl << endl << "Objects of current ASR" << endl;
        //displayObjects(asrObjects);

        //exit1 to mpoint of first object
        abstractASR.push_back(Object(exit[1].mpX(), exit[1].mpY(), asrObjects[0].mpX(), asrObjects[0].mpY()));
        //new abstraction
        for (unsigned int k = 0; k < asrObjects.size() - 1; k++) {
           // if (asrObjects[k + 1].getPos() == -1) {
                tempObj.set(asrObjects[k].mpX(), asrObjects[k].mpY(), asrObjects[k + 1].mpX(), asrObjects[k + 1].mpY(), k + 1);
                abstractASR.push_back(tempObj);
                //abstractASR.push_back(asrObjects[k]);
            //}           
        }
        //mpoint of last object to exit1
        abstractASR.push_back(Object(asrObjects[asrObjects.size() - 1].mpX(), asrObjects[asrObjects.size() - 1].mpY(), exit[3].mpX(), exit[3].mpY()));
        
        vector<Object> boundary;
        vector<Object> gapOnBoundary;
        Object tempGap;
        //boundary
        //exit1 to mpoint of first object
        boundary.push_back(Object(exit[1].mpX(), exit[1].mpY(), asrObjects[0].X1(), asrObjects[0].Y1()));
        //for other surfaces
        for (unsigned int k = 0; k < asrObjects.size() - 1; k++) {
            boundary.push_back(asrObjects[k]);
            tempGap = Object(asrObjects[k].X2(), asrObjects[k].Y2(),asrObjects[k + 1].X1(), asrObjects[k + 1].Y1());
            if(tempGap.length() > 600) {
                
                gapOnBoundary.push_back(tempGap);
            }
            else
            boundary.push_back(tempGap);
        }
        boundary.push_back(asrObjects[asrObjects.size() - 1]);
        //mpoint of last object to exit1
        boundary.push_back(Object(asrObjects[asrObjects.size() - 1].X2(), asrObjects[asrObjects.size() - 1].Y2(), exit[3].mpX(), exit[3].mpY()));
        
               


        exit = addTwoVectorsOfObjects(exit,makeRectangle(allASRs[i].getASRExit2()));

        cout << "Printing ASR" << i + 1 << endl;
        firstObject.push_back(allASRs[i].getASRObjects()[0]);
        sprintf(asrFileName, "%s%d%s", "Maps/absASR-", i, ".png");
        //plotRobotView(asrFileName,firstObject,asrObjects);
        plotObjects(asrFileName, exit, abstractASR);
        sprintf(asrFileName, "%s%d%s", "Maps/ASR-", i, ".png");

//        for (unsigned int k = 0; k < wholeRouteMap[i].size(); k++)
//            dashedRoute = addTwoVectorsOfObjects(dashedRoute, breakTheLineInto(wholeRouteMap[i][k]));
//        dashedRoute.clear();
        plotObjects(asrFileName,allExits, allASRs[i].getASRObjects()); //,nodesOnPath);
        
        sprintf(asrFileName, "%s%d%s", "Maps/PlaceBoundary-", i, ".png");
        //plotRobotView(asrFileName,firstObject,asrObjects);
        plotObjectsOf3Kinds(asrFileName, boundary, exit,gapOnBoundary);


        dashedRoute.clear();
        firstObject.clear();
        asrObjects.clear();
        abstractASR.clear();
        exit.clear();
        route.clear();
        nodesOnPath.clear();
        // waitHere();
    }

    //plotObjects("Maps/PM.png", wholeRoute, mfis);
}