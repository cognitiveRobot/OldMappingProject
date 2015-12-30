#include <cstdlib>
#include <iostream>
#include <sstream>
#include <cmath>
#include "space.h"
#include "convexPathPlanner.h"
#include "generalFunctions.h"

using namespace std;



int viewNo = 0;

// Returns the next (angle,move) you have to do WITHOUT checking if it is possible (new obstacle for instance)
pair<double, double> ConvexRobotPathPlanner::getNextDestinationWithoutVerification(vector<Surface> view, PointXY goal) {
    // We compute the next destination, without checking if it is reachable (new obstacle, imprecision ...)
    if (savedPath.size() == 0 || currentIndex + 1 >= savedPath.size()) {
        if (myCurrentView != NULL) {
            delete myCurrentView;
        }

        if (currentIndex + 1 >= savedPath.size() && savedPath.size() != 0) {
            clear(); // We erase everything saved (including the previously saved goal which should have been reached now)
        }

        //cout << "-- Recomputation of the new surrounding space" << endl;


        //savedPath = myCurrentView->computeDestinationsToGoal(PointXY(0, 0), myGoal, steps_number);

        if (goal == PointXY::INVALID) {
            myCurrentView = new PathComputationModule(view, surroundingDistance);
            cout << "--- I need a new goal" << endl;
            savedPath = myCurrentView->computeDestinationsToBestPolygons(PointXY(0, 0), steps_number);
        } else {
            cout << "--- Going to the given goal" << endl;
            // We add an obstacle such as the goal will be in an obstacle
            //view.push_back(Surface(PointXY(goal.getX() - 1, goal.getX() + 1), PointXY(goal.getX() + 1, goal.getY() + 1)));
            myCurrentView = new PathComputationModule(view, surroundingDistance);
            savedPath = myCurrentView->computeDestinationsToGoal(PointXY(0, 0), goal, steps_number);
        }

        cout << "-- Computation of the new path to follow : " << savedPath.size() << " steps" << endl;
        currentIndex = 0;


        // Plotting and saving the view
        if (PLOT_PATH_COMPUTATION) {
            viewNo++;
            ostringstream os;
            os << "Out/Quantum-" << viewNo << ".png";
            plotSpaceAndPath(os.str().c_str());
        }
        // End of plotting


    } else {
        currentIndex++;
    }

    if (currentIndex < savedPath.size()) {
        cout << "- Next move : dist : " << savedPath[currentIndex].second << "; angle : " << savedPath[currentIndex].first << endl;
        return savedPath[currentIndex];
    } else {
        cout << " - I'm in a surrounding Polygon ! OR I can't reach my goal" << endl;
        cout << currentIndex << " - " << savedPath.size() << endl;
        return pair<double, double>(rand() % 360 - 180, 0);
    }
}


// Returns the next (angle,move) the robot has to do after checking if this move is possible (otherwise, it computes a new path)
pair<double, double> ConvexRobotPathPlanner::getNextDestination(vector<Surface> view, PointXY goal) {


    // Computation of the next move
    pair<double, double> nextMove = getNextDestinationWithoutVerification(view, goal);

    double angle = nextMove.first;
    double dist = nextMove.second;


    // Collision avoidance (we check whether something could hit us if we do what we have decided to do)
    if (isAround(dist, 0)) {
        // No collision risk since we turn on ourselves
    } else {

        double x = -dist * sin(deg2rad(angle));
        double y = dist * cos(deg2rad(angle));

        Surface ray(PointXY(0, 0), PointXY(x, y));


        vector<Surface> viewSurroundingObstacles;
        vector<Surface> stmp;

        for (unsigned i = 0; i < view.size(); i++) {
            stmp = view[i].getSurroundingObstacles(surroundingDistance);
            for (unsigned j = 0; j < stmp.size(); j++) {
                viewSurroundingObstacles.push_back(stmp[j]);
            }
        }

        bool intersectionFound = false;
        for (unsigned i = 0; i < viewSurroundingObstacles.size(); i++) {
            if (ray.intersects(viewSurroundingObstacles[i])) {
                intersectionFound = true;
                break;
            }
        }

        if (intersectionFound) {
            cout << "- Saved Path can't be followed !" << endl;
            // We reinitialize the savedPath and compute a new one
            clear();
            nextMove = getNextDestinationWithoutVerification(view); // This one doesn't need collision verification since it has just been computed
        }
    }

    return nextMove;

}

// Given a move, this function gives you the coordinates of the point you'll be after that move
PointAndAngle ConvexRobotPathPlanner::convertMoveIntoPos(PointAndAngle current, pair<double, double> moveCoordinates) {
    current.angle = current.angle + moveCoordinates.first;
    current.point = PointXY(-moveCoordinates.second * sin(deg2rad(current.angle)), moveCoordinates.second * cos(deg2rad(current.angle))) + current.point;
    return current;
}

// Smae function, but for a vector of moves
vector<PointXY> ConvexRobotPathPlanner::convertDestinationsVectToPointsVect(const vector<pair<double, double> > & v, const PointAndAngle & origin) {
    PointAndAngle currentPositionning = origin;


    vector<PointXY> myPath(1, origin.point);

    for (unsigned i = 0; i < v.size(); i++) {
        currentPositionning = convertMoveIntoPos(currentPositionning, v[i]);
        myPath.push_back(currentPositionning.point);
    }

    myPath = cloneSuppression(myPath);

    return myPath;
}

// Clear the computed path
void ConvexRobotPathPlanner::clear() {
    savedPath.clear();
    currentIndex = 0;
}
