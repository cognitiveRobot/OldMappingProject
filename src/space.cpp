#include "space.h"
#include "PointAndSurface.H"
#include "polygons.h"

using namespace std;

// Function which will try to merge polygons together (the new polygon has to be convex) so as
// you have less polygons
vector<MPolygon> Space::simplifySpace() {
    MPolygon ptmp;

    vector<MPolygon> vp = polygons;

    bool changed;

    do {
        changed = false;

        for (unsigned i = 0; i < vp.size(); i++) {
            for (unsigned j = i + 1; j < vp.size(); j++) {
                ptmp = vp[i].MergePolygons(vp[j]);

                if (!ptmp.isEmpty() && ptmp.isConvex()) {
                    changed = true;
                    vp[i] = ptmp;
                    vp[j] = vp[vp.size() - 1];
                    vp.pop_back();
                    break;
                    break;
                }
            }
        }
    } while (changed);

    return vp;
}

// Compute a triangulation of the environnement :
// For that, it draws lines beetween the vertices of each obstacle, and then it links them to create triangles
// This function could be improved using Constrained Delaunay Triangulation
vector<MPolygon> Space::computeTriangulation() {
    vector<Surface> vertices;
    vector<Surface> vtmp(3);
    vector<MPolygon> result;
    vector<Surface> stmp;
    Surface s;
    MPolygon ptmp;


    vertices = obstacles;
    bool intersectionFound;

    for (unsigned i = 0; i < obstacles.size(); i++) {
        for (unsigned j = i + 1; j < obstacles.size(); j++) {
            if (i != j) {
                stmp.clear();

                s = Surface(obstacles[i].getP1(), obstacles[j].getP1());
                if (obstacles[i].getP1() != obstacles[j].getP1() && s != obstacles[i] && s != obstacles[j] && !s.isInside(obstacles[i], true) && !s.isInside(obstacles[j], true)) {
                    stmp.push_back(s);
                }

                s = Surface(obstacles[i].getP1(), obstacles[j].getP2());
                if (obstacles[i].getP1() != obstacles[j].getP2() && s != obstacles[i] && s != obstacles[j] && !s.isInside(obstacles[i], true) && !s.isInside(obstacles[j], true)) {
                    stmp.push_back(s);
                }

                s = Surface(obstacles[i].getP2(), obstacles[j].getP1());
                if (obstacles[i].getP2() != obstacles[j].getP1() && s != obstacles[i] && s != obstacles[j] && !s.isInside(obstacles[i], true) && !s.isInside(obstacles[j], true)) {
                    stmp.push_back(s);
                }

                s = Surface(obstacles[i].getP2(), obstacles[j].getP2());
                if (obstacles[i].getP2() != obstacles[j].getP2() && s != obstacles[i] && s != obstacles[j] && !s.isInside(obstacles[i], true) && !s.isInside(obstacles[j], true)) {
                    stmp.push_back(s);
                }

                for (unsigned k = 0; k < stmp.size(); k++) {
                    intersectionFound = false;
                    for (unsigned p = 0; p < vertices.size(); p++) {
                        if (stmp[k].intersectsStrictly(vertices[p])) {
                            intersectionFound = true;
                            break;
                        }
                    }
                    if (!intersectionFound) {
                        vertices.push_back(stmp[k]);
                    }
                }

            }
        }
    }

    // We delete the double
    for (unsigned i = 0; i < vertices.size(); i++) {
        for (unsigned j = i + 1; j < vertices.size(); j++) {
            if (vertices[i] == vertices[j]) {
                vertices.erase(vertices.begin() + j);
                j--;
            }
        }
    }

    // Deletion of obstacles within surrounding obstacles
    vertices = getSurfacesNotInsideSurroundingObstacles(vertices);


    
    vector<unsigned> links[vertices.size()];
    unsigned **isLinked;

    isLinked = new unsigned*[vertices.size()];

    for (unsigned i = 0; i < vertices.size(); i++) {
        isLinked[i] = new unsigned[vertices.size()];
        for (unsigned j = 0; j < vertices.size(); j++) {
            isLinked[i][j] = 0;
        }
    }

    for (unsigned i = 0; i < vertices.size(); i++) {
        for (unsigned j = 0; j < vertices.size(); j++) {
            if (i != j) {
                if (vertices[i].getP1() == vertices[j].getP1()) {
                    links[i].push_back(j);
                    isLinked[i][j] = 11;
                    isLinked[j][i] = 11;

                } else if (vertices[i].getP1() == vertices[j].getP2()) {
                    links[i].push_back(j);
                    isLinked[i][j] = 12;
                    isLinked[j][i] = 21;

                } else if (vertices[i].getP2() == vertices[j].getP1()) {
                    links[i].push_back(j);
                    isLinked[i][j] = 21;
                    isLinked[j][i] = 12;

                } else if (vertices[i].getP2() == vertices[j].getP2()) {
                    links[i].push_back(j);
                    isLinked[i][j] = 22;
                    isLinked[j][i] = 22;
                }
            }

        }
    }

   //End of vertices linking


    //Starting triangulation



    bool polygon_found;
    bool is_surrounding_obstacle;
    unsigned j, k;
    // We have the vertices of our triangulation, we must create triangles
    for (unsigned i = 0; i < vertices.size(); i++) {
        for (unsigned j2 = 0; j2 < links[i].size(); j2++) {
            j = links[i][j2];

            for (unsigned k2 = 0; k2 < links[j].size(); k2++) {
                k = links[j][k2];


                if (i != k && j != k && i != j) {
                    if (isLinked[i][k] > 0 && ((isLinked[j][i] + isLinked[j][k]) / 10) % 2 == 1) {
                        vtmp[0] = vertices[i];
                        vtmp[1] = vertices[j];
                        vtmp[2] = vertices[k];
                        ptmp = MPolygon(vtmp);

                        if (!ptmp.isFlat()) {
                            polygon_found = false;

                            // Non-already in the vector verification
                            for (unsigned p = 0; p < result.size(); p++) {
                                if (result[p] == ptmp) {
                                    polygon_found = true;
                                    break;
                                }
                            }

                            if (!polygon_found) {
                                is_surrounding_obstacle = false;

                                for (unsigned p = 0; p < vertices.size(); p++) {
                                    if (pointInPolygon((vertices[p].getP1() + vertices[p].getP2()) / 2.0, vtmp, true)) {
                                        is_surrounding_obstacle = true;
                                        break;
                                    }
                                }
                                if (!is_surrounding_obstacle) {
                                    result.push_back(ptmp);
                                }
                            }
                        }
                    }
                }

            }
        }
    }
    delete isLinked;


    return result;
}

// Add a  big obstacle arount the space, at a distance of at least 1000 of each obstacle, so as 
// every point is inside a polygon
vector<Surface> Space::addEnvironnementObstacle() {
    double xmin = 0, ymin = -1;
    double xmax = 0, ymax = 1;
    const double addDistance = 1000;

    for (unsigned i = 0; i < obstacles.size(); i++) {
        xmin = min(xmin, min(obstacles[i].getX1(), obstacles[i].getX2()));
        ymin = min(ymin, min(obstacles[i].getY1(), obstacles[i].getY2()));
        xmax = max(xmax, max(obstacles[i].getX1(), obstacles[i].getX2()));
        ymax = max(ymax, max(obstacles[i].getY1(), obstacles[i].getY2()));
    }

    xmin -= addDistance;
    xmax += addDistance;
    ymax += addDistance;

    vector<Surface> result = obstacles;
    result.push_back(Surface(PointXY(xmin, ymin), PointXY(xmin, ymax), true));
    result.push_back(Surface(PointXY(xmin, ymax), PointXY(xmax, ymax), true));
    result.push_back(Surface(PointXY(xmax, ymax), PointXY(xmax, ymin), true));
    result.push_back(Surface(PointXY(xmax, ymin), PointXY(xmin, ymin), true));

    return result;

}



// Creates a virtual obstacle behind the robot to prevent him from having a goal behind him (it is possible due to surrounding polygons), and to turn for more than 90°
vector<Surface> Space::addBehindObstacle() {
    double distance_behind = -10;
    double length = 10000;

    vector<Surface> result = obstacles;
    result.push_back(Surface(PointXY(-length / 2., distance_behind), PointXY(length / 2., distance_behind), true));
    return result;

}


// Computation of the triangulation, and then of the optimisation of the environnement
void Space::computePolygons() {
    if (surroundingMode) {
        surroundingObstacles = computeSurroundingObstacles();
        obstacles = surroundingObstacles;
        
        // Add global environnement polygon
        //obstacles = addEnvironnementObstacle();
        obstacles = addBehindObstacle();
        
        
        vector<Surface> obstacles2 = computeNonIntersectingObstacles();
        obstacles = getSurfacesNotInsideSurroundingObstacles(obstacles2);
        


        //cout << obstacles.size() << " obstacle after intersections" << endl;
        polygons = computeTriangulation();
        //cout << polygons.size() << " polygons before simplification" << endl;
        polygons = simplifySpace();
        //cout << polygons.size() << " polygons after simplification" << endl;
    } else {
        obstacles = computeNonIntersectingObstacles();
        obstacles = addEnvironnementObstacle();
        //cout << obstacles.size() << " obstacle after intersections" << endl;
        polygons = computeTriangulation();
        //cout << polygons.size() << " polygons before simplification" << endl;
        polygons = simplifySpace();
        //cout << polygons.size() << " polygons after simplification" << endl;

    }
}


// Create new smaller obstacles, such as any obstacle cross an other one 
// In fact, if we have two obstacles intersecting, we divide each obstacle into two smaller obstacles
// Then, every "small obstacle" won't stricly intersect any of the other obstacle
vector<Surface> Space::computeNonIntersectingObstacles() {
    vector<Surface> result = obstacles;
    vector<Surface> toAddI, toAddJ;
    PointXY ptmp;

    for (unsigned i = 0; i < result.size(); i++) {
        for (unsigned j = i + 1; j < result.size(); j++) {
            ptmp = intersectLines(result[i].getP1(), result[i].getP2(), result[j].getP1(), result[j].getP2());

            if (result[i].intersects(result[j])) {
                toAddI.clear();
                toAddJ.clear();

                if (ptmp != result[i].getP1()) {
                    toAddI.push_back(Surface(result[i].getP1(), ptmp, true));
                }
                if (ptmp != result[i].getP2()) {
                    toAddI.push_back(Surface(result[i].getP2(), ptmp, true));
                }
                if (ptmp != result[j].getP1()) {
                    toAddJ.push_back(Surface(result[j].getP1(), ptmp, true));
                }
                if (ptmp != result[j].getP2()) {
                    toAddJ.push_back(Surface(result[j].getP2(), ptmp, true));
                }

                result[i] = toAddI[0];
                result[j] = toAddJ[0];

                if (toAddI.size() > 1) {
                    result.push_back(toAddI[1]);
                }
                if (toAddJ.size() > 1) {
                    result.push_back(toAddJ[1]);
                }
            }
        }
    }
    return result;
}

vector<MPolygon> Space::getPolygons() {
    return polygons;
}

vector<Surface> Space::getObstacles() {
    return obstacles;
}

void Space::printSpace() {
    for (unsigned i = 0; i < polygons.size(); i++) {
        cout << "----------------------" << endl;
        cout << "Polygon " << i << endl;
        cout << "----------------------" << endl;
        polygons[i].printPolygon();
        cout << endl << endl;

    }
}


// Compute the surrounding space of each obstacle
// In fact, to prevent the robot from hitting the walls, we create bigger obstacles (the amount 
// we increase the width of the obstacle is "surrounding distance" (which is in fact at least the
// robot radius).
// Thus, the robot, for all our computations, will be considered as a point, and won't be able to hit anything
vector<Surface> Space::computeSurroundingObstacles() {
    vector<Surface> result;
    vector<Surface> stmp;

    for (unsigned i = 0; i < obstacles.size(); i++) {
        stmp = obstacles[i].getSurroundingObstacles(surroundingDistance);

        for (unsigned j = 0; j < stmp.size(); j++) {
            result.push_back(stmp[j]);
        }
    }

    return result;
}

vector<Surface> Space::getSurfacesNotInsideSurroundingObstacles(const vector<Surface> & s) {
    vector<Surface> result;
    PointXY p;


    if (surroundingObstacles.size() % 4 != 0) {
        cerr << "Incorrect Surrounding Obstacles Size !!!" << endl;
    }

    for (unsigned i = 0; i < s.size(); i++) {
        p = s[i].getMiddle();

        if (!isInSurroundingObstacle(p)) {
            result.push_back(s[i]);
        }
    }
    return result;
}

// Check whether the point is inside a surrounding obtacle
bool Space::isInSurroundingObstacle(PointXY p) {
    vector<Surface> vtmp(4);

    for (unsigned j = 0; j < surroundingObstacles.size() / 4; j++) {
        for (unsigned k = 0; k < 4; k++) {
            vtmp[k] = surroundingObstacles[4 * j + k];
        }

        if (pointInPolygon(p, vtmp, true)) {
            return true;
            break;
        }
    }

    return false;
}
