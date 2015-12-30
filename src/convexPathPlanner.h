/* 
 * File:   convexPathPlanner.h
 * Author: Jean-Nicolas
 *
 * Created on 7 juin 2011, 14:47
 */

#ifndef CONVEXPATHPLANNER_H
#define	CONVEXPATHPLANNER_H

#define PLOT_PATH_COMPUTATION true

#include <vector>
#include "PointAndSurface.H"
#include "GeometryFuncs.H"
#include "polygons.h"
#include "space.h"

using namespace std;

struct PointAndAngle {
    PointXY point;
    double angle;
};


// This is the class you need to use to make the robot move
class ConvexRobotPathPlanner {
    unsigned steps_number;
    double surroundingDistance;

    vector<pair<double, double> > savedPath;
    unsigned currentIndex;

    PathComputationModule* myCurrentView;

    pair<double, double> getNextDestinationWithoutVerification(vector<Surface> view, PointXY goal = PointXY::INVALID);
    
    PointAndAngle convertMoveIntoPos(PointAndAngle current, pair<double, double> moveCoordinates);
    vector<PointXY> convertDestinationsVectToPointsVect(const vector<pair<double, double> > & v, const PointAndAngle & origin);

public:

    // Constructor of the "ConvexRobotPathPlanner"  : 
    // surrounding distance is the amount we increase the width of the obstacles (must be at least the robot radius)
    // steps_numbers is the maximum number of polygons you can cross to reach the goal (should be beetween 5 and 10).
    // If the steps_number is too big, due to the imprecisions of the move, the robot won't be able to execute precisely the move, and it might get lost
    ConvexRobotPathPlanner(unsigned steps_number, double surroundingDistance = 400) : steps_number(steps_number), surroundingDistance(surroundingDistance) {
        myCurrentView = NULL;
    }

    pair<double, double> getNextDestination(vector<Surface> view, PointXY goal = PointXY::INVALID); // returns a pair (angle, movedistance) to execute for the next move

    void plotSpaceAndPath(const char* filename);

    void clear();

    vector<pair<double, double> > getSavedPath() const {
        return savedPath;
    }

};



/*
 * Example of use of the class : 
 * ConvexRobotPathPlanner myRobot(7, 400);
 * while (true) {
 *      nextMove = myRobot.getNextDestination(view); // compute the next move
 *      // Make the robot execute the move
 *      // Get a new view
 *      // compute MFIS or sthg about the view (optionnal)
 * }
 */
        


#endif	/* CONVEXPATHPLANNER_H */

