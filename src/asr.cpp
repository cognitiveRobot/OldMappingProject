#include <iostream>
#include <vector>
#include <algorithm>
#include "Object.H"

#include "asr.H"
//******************************** ASR Class ************************//

ASR::ASR(vector<Object> objs) {
	objects=objs;
}

int ASR::getASRID() {
	return id;
}

void ASR::setASRID(int a) {
	id = a;
}
vector<Object> ASR::getASRObjects() {
	return objects;
}
void ASR::setASRObjects(vector<Object> objs) {
	objects = objs;
}
Object ASR::getASRExit1() {
	return exits;
}
void ASR::setASRExit1(Object exs) {
	exits = exs;
}
Object ASR::getASRExit2() {
	return exit2;
}
void ASR::setASRExit2(Object exs) {
	exit2 = exs;
}

void ASR::replaceASR(ASR newasr) {
    objects=newasr.getASRObjects();
    exits=newasr.getASRExit1();
    exit2=newasr.getASRExit2();
    route = newasr.getRoute();
}

void ASR::updateRoute(Object robotpath) {
    route.push_back(robotpath);
}
vector<Object> ASR::getRoute() {
    return route;
}
void ASR::replaceTheWholeRoute(vector<Object> newRoute) {
    route=newRoute;
}
void ASR::clearRoute() {
    route.clear();
}

void ASR::addLimitingPoint(Object lpoint) {
    limitingPoints.push_back(lpoint);
}
void ASR::setLimitingPoints(vector<Object> lpoints) {
    limitingPoints = lpoints;
}
vector<Object> ASR::getLimitingPoints() {
    return limitingPoints;
}

void ASR::addLineOfSitePoints(Object lineOfSitePoint) {
    lineOfSitePoints.push_back(lineOfSitePoint);
}
void ASR::setLineOfSitePoints(vector<Object> lOfSitePoints){
    lineOfSitePoints = lOfSitePoints;
}
vector<Object> ASR::getLineOfSitePoints() {
    return lineOfSitePoints;
}
void ASR::setStartPoint(Object sPoint) {
    startPoint = sPoint;
}
Object ASR::getStartPoint() {
    return startPoint;
}
void ASR::setEndPoint(Object ePoint) {
    endPoint = endPoint;
}
Object ASR::getEndPoint() {
    return endPoint;
}