/* 
 * File:   PolygonChoice.h
 * Author: Jean-Nicolas
 *
 * Created on 2 juin 2011, 13:31
 */

#include <vector>
#include "PathPlanning.H"
#include "space.h"
#include "PointAndSurface.H"
#include "convexPathPlanner.h"
#include "Object.H"

#ifndef POLYGONCHOICE_H
#define	POLYGONCHOICE_H



Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Surface> s, int viewno);
Destination DestinationToGo(ConvexRobotPathPlanner *myPathPlanner, vector<Object> s, int viewno);

#endif	/* POLYGONCHOICE_H */

