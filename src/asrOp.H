#ifndef _ASROP_H_
#define _ASROP_H_



#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"
#include "GroupS.H"
#include "Minfo.H"
#include "PathPlanning.H"
#include "Transporter.H"
#include "Map.H"
using namespace std;

#ifdef ASR_NAMESPACE
using namespace asr;
#endif


//for exits
vector<Object> getExitsInCV(vector<Object> nextpv);
vector<Object> deleteSameExits(vector<Object> pexits,vector<Object> tmpexits);

//for ASR
vector<vector<Object> > sortA2OA(vector<Object> mfis, Object rp);


vector<Object> sort(vector<Object> sur,Object rp);//sorting according to ort angle
vector<Object> sortAccording2Length(vector<Object> Objects);//sorting function according to length

vector<vector<Object> > makeASR(vector<Object> mfis, Object crp);
vector<Object> makeCViewASR(vector<Object> mfis);

vector<vector<Object> > findProbableExits(vector<Object> cv);

//find boundary lines using Yeap's theory(using occluding edges)
vector<Object> getBoundaryObjects(vector<Object> cview);//, vector<Object> asr, Object crp);

vector<Object> updateASR(vector<Object> asr, vector<Object> cvbsur, Object crp);

//merge two asr
vector<Object> mergeASR(vector<Object> pasr, vector<Object> casr, Object crp);

vector<Object> deleteInsideObjects(vector<Object> asr, vector<Object> cvbsur, Object crp);
vector<Object> findLinesAroundFreeSpace(vector<Object> cview, vector<Exit> exits);
vector<Object> tagObjectsAsSide(vector<Object> cview);//to make robotview using all objects from cv
vector<Object> tagObjectsAsSideAndViewNumber(vector<Object> cview, int v);
vector<Object> updateMFISandASR(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects);
Transporter computeMap(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects,int asrno);

Transporter computeMapAsRobotView(vector<Object> pasr, vector<Object> casr, Object crp, vector<Object> refobjects, int asrno);

double splitToFromNewASR(vector<Object> exitsinmfis,Object robotpath);
vector<ASR> makeNewASR(vector<ASR> allASRs,vector<Object> xexits);//splits the current asr
vector<ASR> closeTheLoopUsingExit(vector<ASR> allASRs);
ASRNetwork closeTheLoopUsingBoundary(ASRNetwork perceptualMap,vector<Object> cv, vector<Exit> exits, Object crp);
ASRNetwork locateOldExitInCurrentASR(ASRNetwork pm,ASR oldASR, vector<Object> newExits);
vector<ASR> splitASR(vector<Object> allobjects,int asrno, vector<Object> pasrs);
ASRNetwork splitCurrentASR(ASRNetwork pm, vector<Object> xexits);

void abstractASRs(vector<ASR> allASRs, vector<Object> mfis, vector<vector<Object> > wholeRouteMap, vector<Object> refObjects);
vector<Object> mergeOldAndNewASR(vector<Object> oldASR,vector<Object> newASR,vector<Object> firstRPInNewASR,
                          Object newASRExit, Object oldASRExit, vector<Object> oldRoute, vector<Object> newRoute, vector<Object> refObjects);


void boundaryOfThePlace(vector<ASR> allASRs, vector<Object> mfis, vector<vector<Object> > wholeRouteMap, 
                                        vector<Object> refObjects,vector<Object> crossedExit);

#endif
