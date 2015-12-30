/* 
 * File:   DivideIntoPolygons.cpp
 * Author: Jean-Nicolas
 *
 * Created on 27 mai 2011, 10:14
 * The goal is to divide our space using convex polygons
 */
#include "polygons.h"


// This function sort the surfaces of the polygon, so that they are in trigonometric order and S[i].p2=s[i+1].p1
void MPolygon::SortSurfaces() {
    Surface stmp;

    for (unsigned i = 0; i < surfaces.size(); i++) {
        for (unsigned j = i + 1; j < surfaces.size(); j++) {

            if (surfaces[i].getP2() == surfaces[j].getP1()) {
                stmp = surfaces[i + 1];
                surfaces[i + 1] = surfaces[j];
                surfaces[j] = stmp;
                break;
            } else if (surfaces[i].getP2() == surfaces[j].getP2()) {
                if (i + 1 != j) {
                    stmp = surfaces[i + 1];
                    surfaces[i + 1] = surfaces[j].switchPoints();
                    surfaces[j] = stmp;
                } else {
                    surfaces[i + 1] = surfaces[i + 1].switchPoints();
                }
                break;
            }
        }
    }
}

// This function reverse the order of the surfaces of the polygon
void MPolygon::ReversePolygon() {
    vector<Surface> stmp(surfaces.size());

    for (unsigned i = 0; i < surfaces.size(); i++) {
        stmp[surfaces.size() - 1 - i] = surfaces[i].switchPoints();
    }
    surfaces = stmp;
}

// This function returns the middle of the polygon
PointXY MPolygon::getMiddle() {
    PointXY p(0, 0);

    for (unsigned i = 0; i < surfaces.size(); i++) {
        p = p + surfaces[i].getP1();
    }

    p = p / (double) (surfaces.size());

    return p;
}


// This function sort the polygon
void MPolygon::SortPolygon() {
    SortSurfaces();

    // Check if the points of the polygons are given in the trigonometric order
    // it eases a lot computations

    // We get the middle point p of the polygon
    PointXY p = getMiddle();

    PointXY v1 = surfaces[0].getP1() - p;
    PointXY v2 = surfaces[0].getP2() - p;

    if (v1.getX() * v2.getY() - v2.getX() * v1.getY() < 0) {
        // The polygon is not in trigonometrical order
        ReversePolygon();
    }
}


// This function checks whether the numbers of surfaces of the polygon is 0
bool MPolygon::isEmpty() {
    return (surfaces.size() == 0);
}


// This function merges two polygons which have at least one common surface. Each polygon has to be sorted
// in trigonometric order
MPolygon MPolygon::MergePolygons(const MPolygon & other) {
    // Polygons have to be separated (but not strictly) and to be convex 
    // Since the polygons are convex, they have 0 or 1 common surface, no more
    // Points of Polygons have to be in trigonometrical order (done by the constructor)

    vector<Surface> sresult;

    bool found = false;
    unsigned i_pos, j_pos;

    // We find the common surface to make the link computation easy
    for (unsigned i = 0; i < surfaces.size(); i++) {
        for (unsigned j = 0; j < other.surfaces.size(); j++) {
            if (surfaces[i] == other.surfaces[j] && !surfaces[i].IsAnObstacle() && !other.surfaces[j].IsAnObstacle()) {
                found = true;
                i_pos = i;
                j_pos = j;
                break;
                break;
            }
        }
    }



    if (!found) {
        // If there are no common surface, then we return an empty polygon
        return MPolygon(sresult);
    }



    for (int i = 0; i <= (int) i_pos - 1; i++) {
        sresult.push_back(surfaces[i]);
    }
    for (int i = j_pos + 1; i <= (int) j_pos - 1 + (int) other.surfaces.size(); i++) {
        sresult.push_back(other.surfaces[i % other.surfaces.size()]);
    }
    for (unsigned i = i_pos + 1; i < surfaces.size(); i++) {
        sresult.push_back(surfaces[i]);
    }


    return MPolygon(sresult);
}

// Check whether the polygon is convex
bool MPolygon::isConvex() {

    for (unsigned i = 0; i < surfaces.size(); i++) {
        for (unsigned j = 0; j < surfaces.size(); j++) {
            if (i != j) {
                Surface s(surfaces[i].getP1(), surfaces[j].getP1());


                if (s != surfaces[i] && s != surfaces[j]) {
                    if (!pointInPolygon((s.getP1() + s.getP2()) / 2, surfaces)) {
                        return false;
                    }
                }
            }
        }
    }

    return true;
}


// Check whether two polygons have common suface
bool MPolygon::hasCommonSurface(const MPolygon & other) const {
    for (unsigned i = 0; i < surfaces.size(); i++) {
        for (unsigned j = 0; j < other.surfaces.size(); j++) {
            if (surfaces[i] == other.surfaces[j] && !surfaces[i].IsAnObstacle() && !other.surfaces[j].IsAnObstacle()) {
                return true;
            }
        }
    }
    
    return false;
}

// Give the common surface of two polygons is it exists
Surface MPolygon::getCommonSurface(const MPolygon & other) {
    for (unsigned i = 0; i < surfaces.size(); i++) {
        for (unsigned j = 0; j < other.surfaces.size(); j++) {
            if (surfaces[i] == other.surfaces[j] && !surfaces[i].IsAnObstacle() && !other.surfaces[j].IsAnObstacle()) {
                return surfaces[i];
            }
        }
    }
    cerr<<"ERROR : No Common Surface found !!!"<<endl;
    return Surface();
}

bool MPolygon::isFlat() {
    // Only works for triangles
    if (surfaces.size() != 3) {
        return false;
    } else {
        PointXY v1 = surfaces[1].getP1() - surfaces[0].getP1();
        PointXY v2 = surfaces[2].getP1() - surfaces[0].getP1();

        return isAround(v1.getX() * v2.getY() - v2.getX() * v1.getY(), 0);
    }

}

void MPolygon::printPolygon() {
    for (unsigned i = 0; i < surfaces.size(); i++) {
        surfaces[i].printSurface();
    }
}

 // Compute the surface of a convex Polygon
double MPolygon::getSurface() {
    vector<Surface> s = surfaces;
    PointXY AB, AC, BC;
    double a, b, c, p;
    double result = 0;

    unsigned n = s.size();

    while (n >= 3) {
        AC = s[n - 1].getP1() - s[n - 3].getP1();
        AB = s[n - 1].getP1() - s[n - 2].getP1();
        BC = s[n - 2].getP1() - s[n - 3].getP1();

        a = sqrt(BC.getX() * BC.getX() + BC.getY() * BC.getY());
        b = sqrt(AC.getX() * AC.getX() + AC.getY() * AC.getY());
        c = sqrt(AB.getX() * AB.getX() + AB.getY() * AB.getY());

        p = a + b + c;
        result += sqrt(p * (p - 2. * a)*(p - 2. * b)*(p - 2. * c) / 16.); // http://www-clips.imag.fr/geta/User/christophe.chenon/triangle.html

        s[n - 2] = s[n - 1];
        s.pop_back();
        n--;
    }

    return result;
}