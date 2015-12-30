#include "space.h"
#include "generalFunctions.h"

using namespace std;

double vectorsAngleRad(const PointXY & v1, const PointXY & v2) {
    double l1 = v1.distFrom(PointXY(0, 0));
    double l2 = v2.distFrom(PointXY(0, 0));

    double cos_angle = (v1.getX() * v2.getX() + v1.getY() * v2.getY()) / (l1 * l2);
    double sin_angle = (v1.getX() * v2.getY() - v1.getY() * v2.getX()) / (l1 * l2);

    double alpha = acos(cos_angle);
    if (sin_angle < 0) {
        alpha *= -1.0;
    }
    return alpha;
}

double vectorsAngleDeg(const PointXY & v1, const PointXY & v2) {
    return vectorsAngleRad(v1, v2) * 57.2957795;
}

// Computation of the path to reach the goal
// steps_number is the maximum number of polygons you can cross to reach that goal
vector<pair<double, double> > PathComputationModule::computeDestinationsToGoal(PointXY currentPos, PointXY goal, unsigned steps_number) {
    vector<pair<double, double> > result(0);

    vector<PointXY> points;
    cout<<"Inside computeDestinationToGoal"<<endl;
    savedGoal = goal;
    points = computeCheckPointsToGoal(currentPos, goal, steps_number);
    
    
    if (points.size() > 0) {
        // We suppose that the robot is oriented parallel to the y axis
        points.insert(points.begin(), PointXY(currentPos.getX(), currentPos.getY() - 10. * EPSILON)); // Avoid to make a different case for initial Point

        PointXY v1, v2;
        double dist, angle;
        const double stepLength = 1000;
        for (unsigned i = 1; i < points.size() - 1; i++) {
            v1 = points[i] - points[i - 1];
            v2 = points[i + 1] - points[i];

            dist = v2.distFrom(PointXY(0, 0));
            angle = vectorsAngleDeg(v1, v2);

            if (fabs(angle) >= 0.1) {
                // In that case, we can't see our futur goal, so we proceed in two times to go there, to be able to check we can really go there between
                result.push_back(pair<double, double>(angle, 0));
            }

            while (dist > 0) {
                result.push_back(pair<double, double>(0, min(dist, stepLength)));
                dist -= stepLength;
            }
        }
    } else {
        // The goal cannot be reached
        cout << "Can not reach the first goal ... No path to go there ..." << endl;
//        PointXY secondGoal(goal.getX()/2,goal.getY()/2); //hossain
//        computeDestinationsToGoal(currentPos,secondGoal,steps_number);//hossain
        result.push_back(pair<double, double>(0, 0));
    }
    
    return result;
}

// Computation of the path to reach the best polygon
// steps_number is the maximum number of polygons you can cross to reach that maximum surface polygon : thus
// the function return a path to the biggest polygon which distance is inferior or equal to steps_number
vector<pair<double, double> > PathComputationModule::computeDestinationsToBestPolygons(PointXY currentPos, unsigned steps_number) {
    vector<pair<double, double> > result;

    vector<PointXY> points;
    points = computeCheckPointsToBestPolygon(currentPos, steps_number);

    // We suppose that the robot is oriented parallel to the y axis
    points.insert(points.begin(), PointXY(currentPos.getX(), currentPos.getY() - 10. * EPSILON)); // Avoid to make a different case for initial Point

    PointXY v1, v2;
    double dist, angle;
    const double stepLength = 1000;
    for (unsigned i = 1; i < points.size() - 1; i++) {
        v1 = points[i] - points[i - 1];
        v2 = points[i + 1] - points[i];

        dist = v2.distFrom(PointXY(0, 0));
        angle = vectorsAngleDeg(v1, v2);

        if (fabs(angle) >= 0.1) {
            // In that case, we can't see our futur goal, so we proceed in two times to go there, to be able to check we can really go there between
            result.push_back(pair<double, double>(angle, 0));
        }

        while (dist > 0) {
            result.push_back(pair<double, double>(0, min(dist, stepLength)));
            dist -= stepLength;
        }


    }

    return result;
}

// Function which optimize the computed path
vector<PointXY> PathComputationModule::checkPointsOptimisation(vector<PointXY> cp) {
    const double dist_max = 1001;
    PointXY p;



    for (unsigned i = 0; i < cp.size() - 1; i++) {
        if (cp[i].distFrom(cp[i + 1]) > dist_max) {
            p = cp[i]+(cp[i + 1] - cp[i]) / cp[i].distFrom(cp[i + 1]) * (dist_max - 1);
            cp.insert(cp.begin() + i + 1, p);
        }
    }


    bool points_ok[cp.size()];
    for (unsigned j = 0; j < cp.size(); j++) {
        points_ok[j] = true;
    }
    bool intersectionFound;
    Surface stmp;


    for (int i = cp.size() - 1; i >= 0; i--) {
        for (unsigned j = 0; j < i; j++) {

            intersectionFound = false;
            if (points_ok[i] && points_ok[j]) {
                stmp = Surface(cp[i], cp[j]);

                for (unsigned k = 0; k < obstacles.size(); k++) {
                    if (stmp.intersectsStrictly(obstacles[k])) {
                        intersectionFound = true;
                        break;
                    }
                }

                if (!intersectionFound) {
                    for (unsigned k = j + 1; k <= i - 1; k++) {
                        points_ok[k] = false;
                    }
                }
            }
        }
    }

    vector<PointXY> result;
    for (unsigned i = 0; i < cp.size(); i++) {
        if (points_ok[i]) {
            result.push_back(cp[i]);
        }
    }

    return result;
}

// Computation of the check points to go to the best polygon
vector<PointXY> PathComputationModule::computeCheckPointsToBestPolygon(PointXY currentPos, unsigned steps_number) {

    vector<PointXY> result;

    if (isInSurroundingObstacle(currentPos)) {
        result.push_back(currentPos);
        return result;
    }

    vector<MPolygon> path;
    path = computePathToBestPolygon(currentPos, steps_number);

    vector<PointXY> ReversedPoints;

    ReversedPoints.push_back(path[0].getMiddle());
    for (unsigned i = 0; i < path.size() - 1; i++) {
        ReversedPoints.push_back(path[i].getCommonSurface(path[i + 1]).getMiddle());
    }
    ReversedPoints.push_back(currentPos);


    for (unsigned i = 0; i < ReversedPoints.size(); i++) {
        result.push_back(ReversedPoints[ReversedPoints.size() - 1 - i]);
    }

    result = checkPointsOptimisation(result);

    return result;
}

// Computation of the check points to go to the goal
vector<PointXY> PathComputationModule::computeCheckPointsToGoal(PointXY currentPos, PointXY goal, unsigned steps_number) {

    vector<PointXY> result;

    if (isInSurroundingObstacle(currentPos)) {
        result.push_back(currentPos);
        cout<<"output: "<<result.size()<<endl;
        return result;
    }

    
    vector<MPolygon> path;
    path = computePathToGoal(currentPos, goal, steps_number);

    if (path.size() > 0) {
        vector<PointXY> ReversedPoints;
        ReversedPoints.push_back(goal);
        if (path[0].getMiddle() != goal) {
            ReversedPoints.push_back(path[0].getMiddle());
        }
        for (unsigned i = 0; i < path.size() - 1; i++) {
            ReversedPoints.push_back(path[i].getCommonSurface(path[i + 1]).getMiddle());
        }
        ReversedPoints.push_back(currentPos);

        for (unsigned i = 0; i < ReversedPoints.size(); i++) {
            result.push_back(ReversedPoints[ReversedPoints.size() - 1 - i]);
        }

        result = checkPointsOptimisation(result);
    }
    return result;
}



// Function which compute the succession of polygon you have to go to to reach the goal
// WARNING : the path is reversed
vector<MPolygon> PathComputationModule::computePathToGoal(PointXY currentPos, PointXY goal, unsigned steps_number) {
    vector<MPolygon> path;
    MPolygon current_polygon;
    MPolygon goal_polygon;

    // Computation of current Polygon
    for (unsigned i = 0; i < polygons.size(); i++) {
        if (pointInPolygon(currentPos, polygons[i].getSurfaces(), false)) {
            current_polygon = polygons[i];
            break;
        }
    }
    for (unsigned i = 0; i < polygons.size(); i++) {
        if (pointInPolygon(goal, polygons[i].getSurfaces(), false)) {
            goal_polygon = polygons[i];
            break;
        }
    }

    path = computePathToPolygon(current_polygon, goal_polygon, steps_number);

    return path;

}


// Function which compute the succession of polygon you have to go to to reach the best polygon
// WARNING : the path is reversed
vector<MPolygon> PathComputationModule::computePathToBestPolygon(PointXY currentPos, unsigned steps_number) {
    vector<MPolygon> path;
    MPolygon current_polygon;

    // Computation of current Polygon
    for (unsigned i = 0; i < polygons.size(); i++) {
        if (pointInPolygon(currentPos, polygons[i].getSurfaces(), false)) {
            current_polygon = polygons[i];
            break;
        }
    }

    // Computation of each polygon surface
    double polygons_surfaces[polygons.size()];
    for (unsigned i = 0; i < polygons.size(); i++) {
        polygons_surfaces[i] = polygons[i].getSurface();
    }

    // Choice of the best polygon to go to
    while (true) {
        int max_index = 0;
        double max_value = polygons_surfaces[0];

        for (unsigned i = 0; i < polygons.size(); i++) {
            if (polygons_surfaces[i] > max_value && polygons[i] != current_polygon) {
                max_index = i;
                max_value = polygons_surfaces[i];
            }
        }

        if (isAround(max_value, 0)) {
            cerr << "-- I can't find any polygon to go to, going to the center of mine !!!" << endl;
            path.clear();
            path.push_back(current_polygon);
            savedGoal = current_polygon.getMiddle();
            return path;
        }

        path.clear();
        path = computePathToPolygon(current_polygon, polygons[max_index], steps_number);

        if (path.size() > 0) {
            savedGoal = polygons[max_index].getMiddle();
            return path;
        } else {
            // Impossible to go to that polygon
            polygons_surfaces[max_index] = 0;
        }
    }



}

// Recursive function to compute the pah to go to another polygon
vector<MPolygon> PathComputationModule::computePathToPolygon(const MPolygon & current_polygon, const MPolygon & objective, unsigned steps_number) {

    vector<MPolygon> result(0);
    vector<MPolygon> pathtmp;
    unsigned best_size = steps_number + 1;

    if (current_polygon == objective) {
        result.push_back(objective);
        return result;
    }


    if (steps_number == 0) {
        return result;
    }


    for (unsigned i = 0; i < polygons.size(); i++) {
        if (current_polygon.hasCommonSurface(polygons[i])) {
            pathtmp = computePathToPolygon(polygons[i], objective, steps_number - 1);
            pathtmp.push_back(current_polygon);

            if (pathtmp[0] == objective) {
                if (pathtmp.size() < best_size) {
                    result = pathtmp;
                    best_size = pathtmp.size();
                }
            }
        }
    }

    return result;
}
