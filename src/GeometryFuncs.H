/*
 *  Helper functions for geometric calculations.
 *
 *  Thomas
 */

#ifndef _GEOMETRYFUNCS_H_
#define _GEOMETRYFUNCS_H_

#include <cmath>
#include <iostream>
#include <vector>
#include "PointAndSurface.H"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Intersects the (infinite) lines g and h, described by the points G1, G2 and H1, H2.
 * If the lines are parallel, then the result will be (NaN,NaN), which is equal to PointXY::INVALID.
 */
PointXY intersectLines(const PointXY & g1, const PointXY & g2, const PointXY & h1, const PointXY & h2);


/* Gets the angle and distance between newSurf and referenceNew, and applies them to referenceOld.
 * The result is a surface in the old coordinate space.
 * Used for updating the MFIS with new surfaces.
 */
Surface distanceAngleTransform(const Surface & newSurf,
        const Surface & referenceNew,
        const Surface & referenceOld,
        SurfaceMatch::MatchingSurfaceEndpoints referencePoints);


/* Extends the surface (tobeextended) to have the length of (extensionReference).
 * Keeps the ID of tobeextended, but copies the occlusion from extensionReference.
 * If extendFromPoint1 is true, then P1 will be fixed and P2 be changed.
 */
const Surface extendSurface(const Surface & toBeExtended,
        const Surface & extensionReference,
        bool extendFromPoint1);

// Gets a point on this surface that is the nearest to the point specified.
PointXY nearestPointOnSurf(const Surface & surf, const PointXY & other, bool limitToEndpoints);

// Returns the smallest distance between two surfaces (limited to their endpoints).
double distBetweenSurfs(const Surface & s1, const Surface & s2);

// Intersects a line with a circle. Returns a vector of points, containing all intersections (0, 1 or 2 points).
std::vector<PointXY> intersectLineWithCircle(const PointXY & p1,
        const PointXY & p2,
        const PointXY & cCenter,
        double r);

// Returns true if a point is "behind" a line. Behind is the side of the line which (movementDir) faces.
bool isBehindLine(const PointXY & pointToCheck, const Surface & line, const PointXY & movementDir);

/* Determines if a point is inside a polygon. Uses ray casting algorithm.
 * NOTE: if the point is EXACTLY ON a vertex or edge, then result is true (inside).
 */
bool pointInPolygon(const PointXY & point, const std::vector<Surface> & polygon, bool strictly = false);

double deg2rad(double degAngle);

double rad2deg(double radAngle);

// Reduces an angle to the smallest difference bewtween two intersecting lines (always <=90 deg)
double normAngleDiff(double degAngle);

#endif
