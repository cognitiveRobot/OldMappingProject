/* 
 * File:   space.h
 * Author: Jean-Nicolas
 *
 * Created on 27 mai 2011, 13:19
 */

#ifndef SPACE_H
#define	SPACE_H


#include <vector>
#include "PointAndSurface.H"
#include "GeometryFuncs.H"
#include "polygons.h"

using namespace std;



// This class represents the space
class Space {
protected:
    vector<MPolygon> polygons;
    vector<Surface> obstacles;

    bool isInSurroundingObstacle(PointXY p);


private:
    bool surroundingMode;
    double surroundingDistance;
    vector<Surface> surroundingObstacles;


    vector<MPolygon> simplifySpace();
    vector<MPolygon> computeTriangulation();
    vector<Surface> computeNonIntersectingObstacles();
    vector<Surface> computeSurroundingObstacles();

    vector<Surface> getSurfacesNotInsideSurroundingObstacles(const vector<Surface> & s);
    vector<Surface> addEnvironnementObstacle();
    vector<Surface> addBehindObstacle();
    void computePolygons();


public:

    // Constructor of the "space" representation : 
    // surrounding mode is whether we use surrounding obstacles instead of obstacles (to prevent the robot from hitting something)
    // surrounding distance is the amount we increase the width of the obstacles (must be at least the robot radius)
    Space(vector<Surface>obstacles, bool surroundingMode = false, double surroundingDistance = 400) :
    obstacles(obstacles), surroundingMode(surroundingMode), surroundingDistance(surroundingDistance) {
        computePolygons();
    }

    vector<MPolygon> getPolygons();
    vector<Surface> getObstacles();
    void printSpace();

    void plotSpace(char * filename);

};



// This class is the path computation module. (but it is not the PATH PLANNER). It just computes a path in a given environnement, for the robot
class PathComputationModule : public Space {
    PointXY savedGoal;
    
    vector<MPolygon> computePathToBestPolygon(PointXY currentPos, unsigned steps_number);
    vector<MPolygon> computePathToGoal(PointXY currentPos, PointXY goal, unsigned steps_number);
    vector<MPolygon> computePathToPolygon(const MPolygon & current_polygon, const MPolygon & objective, unsigned steps_number);
    vector<PointXY> checkPointsOptimisation(vector<PointXY> cp);
    

public:

    // Constructor of the "PathComputationModule"  : 
    // surrounding distance is the amount we increase the width of the obstacles (must be at least the robot radius)
    PathComputationModule(vector<Surface>obstacles, double surroundingDistance = 400) : Space(obstacles, true, surroundingDistance) {
    }

    vector<PointXY> computeCheckPointsToBestPolygon(PointXY currentPos, unsigned steps_number);
    vector<PointXY> computeCheckPointsToGoal(PointXY currentPos, PointXY goal, unsigned steps_number);
    vector<pair<double, double> > computeDestinationsToBestPolygons(PointXY currentPos, unsigned steps_number);
    vector<pair<double, double> > computeDestinationsToGoal(PointXY currentPos, PointXY goal, unsigned steps_number);


    void plotSpaceAndPath(const char * filename, PointXY currentPos, unsigned steps_number);

    PointXY getGoal() const {
        return savedGoal;
    }

};


#endif	/* SPACE_H */

