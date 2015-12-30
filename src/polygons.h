/* 
 * File:   polygons.h
 * Author: Jean-Nicolas
 *
 * Created on 27 mai 2011, 10:24
 */

#ifndef POLYGONS_H
#define	POLYGONS_H

#include <vector>
#include "PointAndSurface.H"
#include "GeometryFuncs.H"


using namespace std;

class MPolygon {
    vector<Surface> surfaces;

    void SortSurfaces();
    void SortPolygon();
    void ReversePolygon();


public:
    MPolygon MergePolygons(const MPolygon & other);
    bool isEmpty();
    bool isConvex();
    bool isFlat();

    bool hasCommonSurface(const MPolygon & other) const;

    Surface getCommonSurface(const MPolygon & other);

    MPolygon() {
    }

    MPolygon(vector<Surface> surfaces_) {
        surfaces = surfaces_;

        if (!isEmpty()) {
            SortPolygon();
        }
    }

    vector<Surface> getSurfaces() {
        return surfaces;
    }

    PointXY getMiddle();

    bool operator==(const MPolygon & other) const {
        if (surfaces.size() != other.surfaces.size()) {
            return false;
        }

        bool start_found = false;
        unsigned i_start;

        for (unsigned i = 0; i < other.surfaces.size(); i++) {
            if (surfaces[0] == other.surfaces[i]) {
                start_found = true;
                i_start = i;
                break;
            }
        }

        for (unsigned i = 0; i < surfaces.size(); i++) {
            if (surfaces[i] != other.surfaces[(i + i_start) % surfaces.size()]) {
                return false;
            }
        }

        return true;

    }

    bool operator!=(const MPolygon & other) const {
        return !((*this) == other);
    }

    void printPolygon();

    double getSurface();
};


#endif	/* POLYGONS_H */

