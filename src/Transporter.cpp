#include <iostream>
#include <vector>

#include "Object.H"
#include "asr.H"
#include "Transporter.H"

using namespace std;

void Transporter::setViews(vector<vector<Object> > moreviews) { views = moreviews;}
vector<vector<Object> > Transporter::getViews() { return views; }

void Transporter::setView(vector<Object> cv) {	view=cv;}
vector<Object> Transporter::getView() {	return view;}
void Transporter::setReferenceObjects(vector<Object> refobs) {	refObjects=refobs;}
vector<Object> Transporter::getReferenceObjects(){ return refObjects;}
void Transporter::setTargetObjects(vector<Object> tobs) { tObjects=tobs;}
vector<Object> Transporter::getTargetObjects(){ return tObjects;}

void Transporter::setASRs(vector<ASR> asrs) {ASRs=asrs;}
vector<ASR> Transporter::getASRs() {return ASRs;}
void Transporter::setASR(ASR oneasr) {asr=oneasr;}
ASR Transporter::getASR() {return asr;}

void Transporter::setLostSituation(bool lost) {lostSituation=lost;}
bool Transporter::getLostSituation() { return lostSituation;}

void Transporter::setReferenceObjectsForLoopClosing(vector<Object> refobs) {
    refObjectsForLoopClosing = refobs;
}
    vector<Object> Transporter::getReferenceObjectsForLoopClosing() {
        return refObjectsForLoopClosing;
    }
void Transporter::setRobotPosition(vector<Object> rp) {
    robotPosition = rp;
}
    vector<Object> Transporter::getRobotPosition() {
        return robotPosition;
    }
    void Transporter::setAllRobotPositions(vector<Object> aRP) {
        allRobotPositions = aRP;
    }
    vector<Object> Transporter::getAllRobotPositions() {
        return allRobotPositions;
    }
    void Transporter::setMFIS(vector<Object> mfis){
        MFIS = mfis;
    }
    vector<Object> Transporter::getMFIS() {
        return MFIS;
    }
    
    void Transporter::setExits(vector<Exit> ex) {
        exits = ex;
    }
    vector<Exit> Transporter::getExits() {
        return exits;
    }