/*
 *  Plotting: contains helper functions for plotting Objects and/or points.
 *  Taken from the old gnuplot code.
 *
 *  Thomas
 */
#ifndef _THESIS_H_
#define _THESIS_H_
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>
#include <cmath>

#include "Object.H"
#include "PathPlanning.H"
#include "asrOp.H"
#include "PointAndSurface.H"

//#include "PointAndObject.H"

using namespace std;

//it reads a laser file and print in .png showing laser points, rays, and robot position
void printASingleScan();

//it computes a global map using only odometric information
void odometricErrorMap(vector<Object> & errorMap, vector<Object> currentView,
        double traveledDistance, double robotFacing);
#endif