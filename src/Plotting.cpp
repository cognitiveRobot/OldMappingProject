/*
 *  Plotting: contains helper functions for plotting Objects and/or points.
 *  Taken from the old gnuplot code.
 *
 *  Thomas
 *	modified by mhossain
 *      1 RED
 *      2 GREEN
 *      3 BLUE
 */
#include <vector>

#include <iostream>
#include <cmath>
#include <deque>
#include "readAndwriteASCII.H"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/foreach.hpp>
#include <fstream>
#include "PointAndSurface.H"
#include "Point.H"

#include "Plotting.H"
#include "Object.H"
#include "PathPlanning.H"
#include "asrOp.H"
#include "GeometricOp.H"


const char * GNUPLOT_PATH = "/usr/local/bin/gnuplot";
const unsigned int PLOT_RESOLUTION_X = 2400;
const unsigned int PLOT_RESOLUTION_Y = 2400;
const double PLOT_BORDER_FACTOR = 0.05; // Times the original with/height of the image

namespace bg = boost::geometry;

// Plot a combination of points and Objects

void plotObjects(const char * filename, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;

    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Objects\" with linespoints 1 19, \n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        //if(Objects[i].getASRNo() == 1) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
        //}
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}


// Plot a combination of points and Objects

void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth %d\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y, 5);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);
    //    fprintf(fgnup, "set yrange[%d:%d]\n", -7000, 27000);
    //    fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 11000);
    
     //to print id
//      for (int i = 0; i<int(Objects.size()); i++) {
//    fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects[i].getID(),Objects[i].mpX(),Objects[i].mpY()+200);
//      }

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines %d, \\\n", 1);
    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint %d 19\n", 1);

    // Plot pose
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        //if(Objects[i].getASRNo() == 1) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
        //}
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}


// Plot a combination of points and Objects //int argument is pass to highlight a surface

void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects, int refID) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);
    //    fprintf(fgnup, "set yrange[%d:%d]\n", -7000, 27000);
    //    fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 11000);
    
     //to print id
      for (int i = 0; i<int(Objects.size()); i++) {
          if(Objects[i].getID() == refID)
              fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects[i].getID(),Objects[i].mpX()+500,Objects[i].mpY()+500);
          else
              fprintf(fgnup, "set label \"%d\" at %g,%g\n", Objects[i].getID(),Objects[i].mpX()+500,Objects[i].mpY()+500);
      }

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines 3, \\\n");
    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint 1 19\n");

    // Plot pose
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        //if(Objects[i].getASRNo() == 1) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
        //}
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

void plotObjectsAndPoints(const char * filename, vector<Object> pose, vector<Point> points, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    for (int i = 0; i<int(points.size()); i++) {
        minX = min(minX, points[i].X());
        maxX = max(maxX, points[i].X());
        minY = min(minY, points[i].Y());
        maxY = max(maxY, points[i].Y());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 4.0;
        maxX += diff / 4.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 2\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
//    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
//    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
        fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);
        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);

    //1 = red, 2 = green, 3 = blue
    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Points\" with points 1 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines 1\n");

    // Plot Robot Positions
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    //plot points
    for (int i = 0; i<int(points.size()); i++) {
        fprintf(fgnup, "%g ", points[i].X());
        fprintf(fgnup, "%g\n", points[i].Y());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

// Plot a combination of points and Objects

void plotObjectsAndPExits(const char * filename, vector<Object> pose, vector<Exit> Objects2, vector<Object> Objects1) 
{
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) 
    {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 5\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //fprintf(fgnup, "set yrange[%d:%d]\n", -500, 22000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 5000);

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Probable Exits\" with lines 3, \\\n");
    fprintf(fgnup, "\"-\" ti \"Objects\" with linesspoint 1 19\n");

    // Plot lines for robot
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Exits
    for (int i = 0; i<int(Objects2.size()); i++) 
    {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

// Plot a combination of points and Objects

void plotAll(const char * filename, vector<Object> pose, vector<Object> tobjects, vector<Exit> Objects2, vector<Object> Objects1) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 5\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with lines 3, \\\n");
    fprintf(fgnup, "\"-\" ti \"Probable Exits\" with lines 4, \\\n");
    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint 1 19\n");

    // Plot lines for robot
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Target Objects
    for (int i = 0; i<int(tobjects.size()); i++) {
        fprintf(fgnup, "%g ", tobjects[i].X1());
        fprintf(fgnup, "%g\n", tobjects[i].Y1());
        fprintf(fgnup, "%g ", tobjects[i].X2());
        fprintf(fgnup, "%g\n\n", tobjects[i].Y2());
    }
    fprintf(fgnup, "e\n");
    
    // Plot Exits
    for (int i = 0; i<int(Objects2.size()); i++) {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");    

    fflush(fgnup);
    fclose(fgnup);
}

//parameters: filename, robot pose, some other objects, all view objects

void plotObjectsOf3Kinds(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Object> Objects2) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth %d\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y,5);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);

    //to print id
//    for (int i = 0; i<int(Objects2.size()); i++) {
//        fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects2[i].getID(), Objects2[i].mpX(), Objects2[i].mpY() + 400);
//    }

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with linespoint 1, \\\n"); //for first argument
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with linespoint 2, \\\n"); //for second argument
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines 3\n"); //for third argument

   
    // Plot lines for robot - first argument
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot target Objects - 2nd argument
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects - 3rd argument
    for (int i = 0; i<int(Objects2.size()); i++) {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");
    
   


    fflush(fgnup);
    fclose(fgnup);
}

void plotObjectsOf3Kinds(const char * filename, vector<Object> pose, vector<Object> Objects1, 
        vector<Object> Objects2, vector<int> limitingPoints) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);

    //to print id
//    for (int i = 0; i<int(Objects2.size()); i++) {
//        fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects2[i].getID(), Objects2[i].mpX(), Objects2[i].mpY() + 400);
//    }

    fprintf(fgnup, "plot ");
    for (unsigned int i = 0; i < limitingPoints.size(); i++) {
        fprintf(fgnup, "\"-\" notitle \"Pose\" with linespoints %d 19", i + 1); //for first argument
        fprintf(fgnup, ", \\\n");
    }
    //fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with linespoint 1, \\\n"); //for first argument
    fprintf(fgnup, "\"-\" ti \"Objects1\" with linespoint 2, \\\n"); //for second argument
    fprintf(fgnup, "\"-\" ti \"Objects2\" with lines 3\n"); //for third argument

    // Plot Objects
    for (unsigned int lp = 0; lp < limitingPoints.size(); lp++) {
        for (int i = 0; i<int(pose.size()); i++) {
            if (pose[i].getLimitingPoint() == limitingPoints[lp]) {
                fprintf(fgnup, "%g ", pose[i].X1());
                fprintf(fgnup, "%g\n", pose[i].Y1());
                fprintf(fgnup, "%g ", pose[i].X2());
                fprintf(fgnup, "%g\n\n", pose[i].Y2());
            }
        }
        fprintf(fgnup, "e\n");
    }


    // Plot target Objects - 2nd argument
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects - 3rd argument
    for (int i = 0; i<int(Objects2.size()); i++) {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");
    
   


    fflush(fgnup);
    fclose(fgnup);
}

void plotObjectsOf4Kinds(const char * filename, vector<Object> pose,
        vector<Object> nodes,
        vector<Object> Objects1,
        vector<Object> Objects2) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);

    fprintf(fgnup, "plot \"-\" ti \"Objects\" with lines 3, \\\n"); //for first argument
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with lines 1, \\\n"); //for second argument
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with linespoint 4 19, \\\n"); //for third argument
    fprintf(fgnup, "\"-\" ti \"Robot Positions\" with linespoint 2 19\n"); //for forth argument


    // Plot pose
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot nodes
    for (int i = 0; i<int(nodes.size()); i++) {
        fprintf(fgnup, "%g ", nodes[i].X1());
        fprintf(fgnup, "%g\n", nodes[i].Y1());
        fprintf(fgnup, "%g ", nodes[i].X2());
        fprintf(fgnup, "%g\n\n", nodes[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects1
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");


    // Plot objects2
    for (int i = 0; i<int(Objects2.size()); i++) {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

// Plot a combination of points and Objects

void plotRobotView(const char * filename, vector<Object> pose, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 8\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //fprintf(fgnup, "set yrange[%d:%d]\n", -500, 22000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 5000);


    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Gaps\" with lines 3, \\\n");
    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint 1 19\n");

    // Plot points
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    Object oneline;
    vector<Object> Obj;
    for (int i = 0; i<int(Objects.size() - 2); i++) {
        Object oneline(Objects[i].X2(), Objects[i].Y2(), Objects[i + 1].X1(), Objects[i + 1].Y1(), 1);
        Obj = breakTheLineInto(oneline);
        for (int j = 0; j<int(Obj.size()); j++) {
            fprintf(fgnup, "%g ", Obj[j].X1());
            fprintf(fgnup, "%g\n", Obj[j].Y1());
            fprintf(fgnup, "%g ", Obj[j].X2());
            fprintf(fgnup, "%g\n\n", Obj[j].Y2());
        }
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}



// Plot a combination of points and Objects

void plotASR(const char * filename, vector<Object> Objects, int totalasr) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }


    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 5\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    fprintf(fgnup, "plot ");

    // int i =1;
    for (int i = 1; i <= totalasr; i++) {

        fprintf(fgnup, "\"-\" ti \"ASR\" with linespoint %d 19", i);
        if (i == totalasr)
            fprintf(fgnup, "\n");
        else
            fprintf(fgnup, ", \\\n");
    }

    for (int j = 1; j <= totalasr; j++) {
        for (int i = 0; i < int(Objects.size()); i++)//int(Objects.size());i++)
        {
            if (Objects[i].getASRNo() == j) {
                fprintf(fgnup, "%g ", Objects[i].X1());
                fprintf(fgnup, "%g\n", Objects[i].Y1());
                fprintf(fgnup, "%g ", Objects[i].X2());
                fprintf(fgnup, "%g\n\n", Objects[i].Y2());
            }
            //fprintf(fgnup,"e\n");
        }
        fprintf(fgnup, "e\n");
    }

    fflush(fgnup);
    fclose(fgnup);
}

// Plot a combination of points and Objects

void plotSingleASR(const char * filename, ASR singleASR) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    //extract objects from ASR to print
    vector<Object> Objects, pose, limitingPoints;
    Objects = singleASR.getASRObjects();
    Objects.push_back(singleASR.getASRExit1());
    Objects.push_back(singleASR.getASRExit2());

    //    limitingPoints = singleASR.getLimitingPoints();
    limitingPoints = makeSquareAtLoSPoints(singleASR.getLineOfSitePoints());

    pose = singleASR.getRoute(); //robot path for this ASR

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 5\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //fprintf(fgnup, "set yrange[%d:%d]\n", -500, 22000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 5000);


    fprintf(fgnup, "plot \"-\" ti \"Objects\" with linespoint 1 19, \\\n");
    fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Limiting Points\" with lines 1, \\\n");
    fprintf(fgnup, "\"-\" ti \"Gaps\" with linespoint 2 19\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot points
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Limiting points
    for (int i = 0; i<int(limitingPoints.size()); i++) {
        fprintf(fgnup, "%g ", limitingPoints[i].X1());
        fprintf(fgnup, "%g\n", limitingPoints[i].Y1());
        fprintf(fgnup, "%g ", limitingPoints[i].X2());
        fprintf(fgnup, "%g\n\n", limitingPoints[i].Y2());
    }
    fprintf(fgnup, "e\n");

    Object oneline;
    //    vector<Object> Obj;
    //    //gaps from p1 of exit1 to p1 of first objects of this asr
    //    oneline.set(Objects[Objects.size() - 2].X1(), Objects[Objects.size() - 2].Y1(), Objects[0].X1(), Objects[0].Y1(), 1);
    //    Obj = breakTheLineInto(oneline);
    //    for (int j = 0; j<int(Obj.size()); j++) {
    //        fprintf(fgnup, "%g ", Obj[j].X1());
    //        fprintf(fgnup, "%g\n", Obj[j].Y1());
    //        fprintf(fgnup, "%g ", Obj[j].X2());
    //        fprintf(fgnup, "%g\n\n", Obj[j].Y2());
    //    }
    //    //gaps
    //    for (int i = 0; i<int(Objects.size() - 3); i++) {
    //        oneline.set(Objects[i].X2(), Objects[i].Y2(), Objects[i + 1].X1(), Objects[i + 1].Y1(), 1);
    //        Obj = breakTheLineInto(oneline);
    //        for (int j = 0; j<int(Obj.size()); j++) {
    //            fprintf(fgnup, "%g ", Obj[j].X1());
    //            fprintf(fgnup, "%g\n", Obj[j].Y1());
    //            fprintf(fgnup, "%g ", Obj[j].X2());
    //            fprintf(fgnup, "%g\n\n", Obj[j].Y2());
    //        }
    //    }
    //    //gaps from p2 of exit1 to p1 of last objects of this asr
    //    oneline.set(Objects[Objects.size() - 3].X2(), Objects[Objects.size() - 3].Y2(), Objects[Objects.size() - 2].X2(), Objects[Objects.size() - 2].Y2(), 1);
    //    Obj = breakTheLineInto(oneline);
    //    for (int j = 0; j<int(Obj.size()); j++) {
    //        fprintf(fgnup, "%g ", Obj[j].X1());
    //        fprintf(fgnup, "%g\n", Obj[j].Y1());
    //        fprintf(fgnup, "%g ", Obj[j].X2());
    //        fprintf(fgnup, "%g\n\n", Obj[j].Y2());
    //    }

    //cross line through exits
    Point mp;
    for (int j = 0; j < 2; j++) {//for two exits
        mp.set(Objects[Objects.size() - 2 + j].mpX(), Objects[Objects.size() - 2 + j].mpY());
        Objects[Objects.size() - 2 + j].setP1(mp.X(), mp.Y());
        for (int i = 1; i < 8; i++) {//for 7 lines
            oneline = makeLineAtPointWithObject(45 * i, 0, Objects[Objects.size() - 2 + j]);
            fprintf(fgnup, "%g ", oneline.X1());
            fprintf(fgnup, "%g\n", oneline.Y1());
            fprintf(fgnup, "%g ", oneline.X2());
            fprintf(fgnup, "%g\n\n", oneline.Y2());
        }
    }


    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

void plotPerceptualMapWithASRs(const char * filename, vector<ASR> ASRs) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;

    //first ASR
    vector<Object> Objects, pose;
    for (int j = 0; j<int(ASRs.size()); j++) {
        Objects = ASRs[j].getASRObjects();
        for (int i = 0; i<int(Objects.size()); i++) {
            minX = min(minX, Objects[i].X1());
            minX = min(minX, Objects[i].X2());
            maxX = max(maxX, Objects[i].X1());
            maxX = max(maxX, Objects[i].X2());
            minY = min(minY, Objects[i].Y1());
            minY = min(minY, Objects[i].Y2());
            maxY = max(maxY, Objects[i].Y1());
            maxY = max(maxY, Objects[i].Y2());
        }
    }


    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 5\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //fprintf(fgnup, "set yrange[%d:%d]\n", -500, 22000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 5000);
    fprintf(fgnup, "plot ");
    for (int k = 0; k<int(ASRs.size()); k++) {
        fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint %d 19", k + 1);
        fprintf(fgnup, ", \\\n");
        fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines 12, \\\n");
        if (k == int(ASRs.size() - 1))
            fprintf(fgnup, "\"-\" ti \"Gaps\" with linespoint 13 19\n");
        else
            fprintf(fgnup, "\"-\" ti \"Gaps\" with linespoint 13 19, \\\n");
    }

    for (int k = 0; k<int(ASRs.size()); k++) {

        Objects = ASRs[k].getASRObjects();
        Objects.push_back(ASRs[k].getASRExit1());
        Objects.push_back(ASRs[k].getASRExit2());
        pose = ASRs[k].getRoute();

        // Plot Objects
        for (int i = 0; i<int(Objects.size()); i++) {
            fprintf(fgnup, "%g ", Objects[i].X1());
            fprintf(fgnup, "%g\n", Objects[i].Y1());
            fprintf(fgnup, "%g ", Objects[i].X2());
            fprintf(fgnup, "%g\n\n", Objects[i].Y2());
        }
        fprintf(fgnup, "e\n");
        fprintf(fgnup, "\n");

        // Plot Robot positions/route
        for (int i = 0; i<int(pose.size()); i++) {
            fprintf(fgnup, "%g ", pose[i].X1());
            fprintf(fgnup, "%g\n", pose[i].Y1());
            fprintf(fgnup, "%g ", pose[i].X2());
            fprintf(fgnup, "%g\n\n", pose[i].Y2());
        }
        fprintf(fgnup, "e\n");
        fprintf(fgnup, "\n");

        //gaps between two objects
        Object oneline;
        vector<Object> Obj;
        //gaps from p1 of exit1 to p1 of first objects of this asr
        oneline.set(Objects[Objects.size() - 2].X1(), Objects[Objects.size() - 2].Y1(), Objects[0].X1(), Objects[0].Y1(), 1);
        Obj = breakTheLineInto(oneline);
        for (int j = 0; j<int(Obj.size()); j++) {
            fprintf(fgnup, "%g ", Obj[j].X1());
            fprintf(fgnup, "%g\n", Obj[j].Y1());
            fprintf(fgnup, "%g ", Obj[j].X2());
            fprintf(fgnup, "%g\n\n", Obj[j].Y2());
        }
        //gaps
        for (int i = 0; i<int(Objects.size() - 3); i++) {
            oneline.set(Objects[i].X2(), Objects[i].Y2(), Objects[i + 1].X1(), Objects[i + 1].Y1(), 1);
            Obj = breakTheLineInto(oneline);
            for (int j = 0; j<int(Obj.size()); j++) {
                fprintf(fgnup, "%g ", Obj[j].X1());
                fprintf(fgnup, "%g\n", Obj[j].Y1());
                fprintf(fgnup, "%g ", Obj[j].X2());
                fprintf(fgnup, "%g\n\n", Obj[j].Y2());
            }
        }
        //gaps from p2 of exit1 to p1 of last objects of this asr
        oneline.set(Objects[Objects.size() - 3].X2(), Objects[Objects.size() - 3].Y2(), Objects[Objects.size() - 2].X2(), Objects[Objects.size() - 2].Y2(), 1);
        Obj = breakTheLineInto(oneline);
        for (int j = 0; j<int(Obj.size()); j++) {
            fprintf(fgnup, "%g ", Obj[j].X1());
            fprintf(fgnup, "%g\n", Obj[j].Y1());
            fprintf(fgnup, "%g ", Obj[j].X2());
            fprintf(fgnup, "%g\n\n", Obj[j].Y2());
        }

        //cross line through exits
        Point mp;
        for (int j = 0; j < 2; j++) {//for two exits
            mp.set(Objects[Objects.size() - 2 + j].mpX(), Objects[Objects.size() - 2 + j].mpY());
            Objects[Objects.size() - 2 + j].setP1(mp.X(), mp.Y());
            for (int i = 1; i < 8; i++) {//for 7 lines
                oneline = makeLineAtPointWithObject(45 * i, 0, Objects[Objects.size() - 2 + j]);
                fprintf(fgnup, "%g ", oneline.X1());
                fprintf(fgnup, "%g\n", oneline.Y1());
                fprintf(fgnup, "%g ", oneline.X2());
                fprintf(fgnup, "%g\n\n", oneline.Y2());
            }
        }


        fprintf(fgnup, "e\n");
    }


    fflush(fgnup);
    fclose(fgnup);
}

void plotAllASR(vector<ASR> allasr, vector<Object> currentRobotPosition) {
    vector<Object> objs;

    char asrname[80];
    MyRobot myrobot(0, 0);
    vector<Object> startingRP = myrobot.getRobot();
    //cout<<"asr exits "<<allasr[allasr.size()-1].getASRExits().size()<<endl;
    //    for (int i = 0; i<int(allasr.size()); i++) {
    //
    //        // objs = allasr[i].getASRObjects();
    //        //objs.push_back(allasr[i].getASRExit1());
    //        //objs.push_back(allasr[i].getASRExit2());
    //        /*if(i == int(allasr.size()-1))//true means last asr that is still growing
    //                objs.push_back(allasr[i].getASRExit1());
    //        else
    //                objs.push_back(allasr[i].getASRExit2());*/
    //        //dummy = allasr[i].getRoute();
    //
    //        sprintf(asrname, "%s%d%s", "MFIS/asr-", 1 + i, ".png");
    //        //cout<<"asr objects "<<objs.size()<<endl;
    //        //dummy.push_back(objs[0]);
    //        //displayObjects(objs);
    //        cout << "ASR " << allasr[i].getASRID() << ", total objects: " << objs.size() << endl;
    //        plotSingleASR(asrname, allasr[i]);
    //    }

    //all ASR objects except current one
    vector<Object> allObjectsInMFIS;
    for (int j = 0; j<int(allasr.size() - 1); j++) {
        for (int k = 0; k<int(allasr[j].getASRObjects().size()); k++)
            allObjectsInMFIS.push_back(allasr[j].getASRObjects()[k]);
    }

    //objects of current ASR
    vector<Object> objectsFromCurrentASR = allasr.back().getASRObjects();

    //first and last Robot positions
    for (int i = 0; i < currentRobotPosition.size(); i++) {
        startingRP.push_back(currentRobotPosition[i]);
    }

    plotObjectsOf3Kinds("Maps/ASRNetwork.png", allObjectsInMFIS, objectsFromCurrentASR, startingRP);
}

// Plot a combination of points and surfaces

void plotPointsAndSurfaces(const char * filename, const vector<PointXY> & points, const vector<Surface> & surfaces) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (vector<PointXY>::const_iterator it = points.begin(); it != points.end(); ++it) {
        minX = min(minX, it->getX());
        maxX = max(maxX, it->getX());
        minY = min(minY, it->getY());
        maxY = max(maxY, it->getY());
    }


    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 20\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Surfaces\" with linespoint 3 19, \\\n");
    fprintf(fgnup, "\"-\" ti \"Points\" with points 1 2, \\\n");
    fprintf(fgnup, "\"-\" ti \"Occluding edges\" with points 2 4\n");

    // Plot surfaces
    for (vector<Surface>::const_iterator surfIt = surfaces.begin(); surfIt != surfaces.end(); ++surfIt) {
        fprintf(fgnup, "%g ", surfIt->getX1());
        fprintf(fgnup, "%g\n", surfIt->getY1());
        fprintf(fgnup, "%g ", surfIt->getX2());
        fprintf(fgnup, "%g\n\n", surfIt->getY2());
    }
    fprintf(fgnup, "e\n");

    // Plot points
    for (vector<PointXY>::const_iterator it = points.begin(); it != points.end(); ++it) {
        fprintf(fgnup, "%g ", it->getX());
        fprintf(fgnup, "%g\n", it->getY());
    }
    fprintf(fgnup, "e\n");

    // Plot occluding edges
    bool noOccluding = true;
    for (vector<Surface>::const_iterator surfIt = surfaces.begin(); surfIt != surfaces.end(); ++surfIt) {
        if (surfIt->isP1Occluding()) {
            noOccluding = false;
            fprintf(fgnup, "%g ", surfIt->getX1());
            fprintf(fgnup, "%g\n", surfIt->getY1());
        }
        if (surfIt->isP2Occluding()) {
            noOccluding = false;
            fprintf(fgnup, "%g ", surfIt->getX2());
            fprintf(fgnup, "%g\n", surfIt->getY2());
        }
    }
    if (noOccluding) // For the special case that there's no occluding edge at all
        fprintf(fgnup, "%g %g\n", 0.0, 0.0);

    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}


//

void plotPerceptualMap(const char * filename, vector<Object> pose, vector<Object> Objects, int ASRNumber) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //    fprintf(fgnup, "set yrange[%d:%d]\n", -7000, 27000);
    //    fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 11000);
    fprintf(fgnup, "plot ");

    for (unsigned int i = 0; i < ASRNumber; i++) {
        fprintf(fgnup, "\"-\" ti \"Objects\" with linespoints %d 19", i + 1);
        fprintf(fgnup, ", \\\n");
    }
    // fprintf(fgnup,"\"-\" ti \"small Objects\" with linespoint 2 2\\\n");
    fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines 10\n");


    // Plot Objects
    for (unsigned int asr = 0; asr < ASRNumber; asr++) {
        for (int i = 0; i<int(Objects.size()); i++) {
            if (Objects[i].getASRNo() == asr + 1) {
                fprintf(fgnup, "%g ", Objects[i].X1());
                fprintf(fgnup, "%g\n", Objects[i].Y1());
                fprintf(fgnup, "%g ", Objects[i].X2());
                fprintf(fgnup, "%g\n\n", Objects[i].Y2());
            }
        }
        fprintf(fgnup, "e\n");
    }

    // Plot Robot Positions
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");


    fflush(fgnup);
    fclose(fgnup);
}

void plotObjects(const char * filename, vector<Object> pose, vector<Object> Objects, vector<int> limitingPoints) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    //    fprintf(fgnup, "set yrange[%d:%d]\n", -7000, 27000);
    //    fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 11000);
    fprintf(fgnup, "plot ");

    int lastI;
    for (unsigned int i = 0; i < limitingPoints.size(); i++) {
        fprintf(fgnup, "\"-\" notitle \"Objects\" with linespoints %d 19", i + 1);
       // fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d", i+1);
        lastI = i + 1;
        fprintf(fgnup, ", \\\n");
        
       
    
    }
    // fprintf(fgnup,"\"-\" ti \"small Objects\" with linespoint 2 2\\\n");
    fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d\n", lastI);
//    for (unsigned int i = 0; i < limitingPoints.size(); i++) {
//    fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d", i);
//    fprintf(fgnup, ", \n");
//    }


    // Plot Objects
    for (unsigned int lp = 0; lp < limitingPoints.size(); lp++) {
        for (int i = 0; i<int(Objects.size()); i++) {
            if (Objects[i].getLimitingPoint() == limitingPoints[lp]) {
                //if (Objects[i].getID() == limitingPoints[lp]) {
                //cout<<limitingPoints[lp]<<endl;
                fprintf(fgnup, "%g ", Objects[i].X1());
                fprintf(fgnup, "%g\n", Objects[i].Y1());
                fprintf(fgnup, "%g ", Objects[i].X2());
                fprintf(fgnup, "%g\n\n", Objects[i].Y2());
            }
        }
//        int st= lp * 9;
//       // fprintf(fgnup, "e\n");
//        for (int j = 0; j<9; j++) {
//        fprintf(fgnup, "%g ", pose[st+j].X1());
//        fprintf(fgnup, "%g\n", pose[st+j].Y1());
//        fprintf(fgnup, "%g ", pose[st+j].X2());
//        fprintf(fgnup, "%g\n\n", pose[st+j].Y2());
//        }
        fprintf(fgnup, "e\n");
    }

//      for (unsigned int lp = 0; lp < limitingPoints.size(); lp++) {
//            int st= lp * 9;
//       // fprintf(fgnup, "e\n");
//        for (int j = 0; j<9; j++) {
//        fprintf(fgnup, "%g ", pose[st+j].X1());
//        fprintf(fgnup, "%g\n", pose[st+j].Y1());
//        fprintf(fgnup, "%g ", pose[st+j].X2());
//        fprintf(fgnup, "%g\n\n", pose[st+j].Y2());
//        }
//            fprintf(fgnup, "e\n");
//      }
        
    // Plot Robot Positions
    for (int i = 0; i<int(pose.size()); i++) {
        cout << i << endl;
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");


    fflush(fgnup);
    fclose(fgnup);
}

void plotObjects(const char * filename, vector<vector<Object> > pose) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < pose.size(); i++) {
        for (unsigned int j = 0; j < pose[i].size(); j++) {
        minX = min(minX, pose[i][j].X1());
        minX = min(minX, pose[i][j].X2());
        maxX = max(maxX, pose[i][j].X1());
        maxX = max(maxX, pose[i][j].X2());
        minY = min(minY, pose[i][j].Y1());
        minY = min(minY, pose[i][j].Y2());
        maxY = max(maxY, pose[i][j].Y1());
        maxY = max(maxY, pose[i][j].Y2());
    }
    }
    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    // fprintf(fgnup, "set yrange[%d:%d]\n", 3000, 6000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, -2000);
    fprintf(fgnup, "plot ");

     
    for (unsigned int i = 0; i < pose.size(); i++) {
        fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d", i + 2); //for second argument
        fprintf(fgnup, ", \\\n");
    }
    
    // Plot Robot Positions second argument
    for (unsigned int i = 0; i < pose.size(); i++) {
        for (unsigned int j = 0; j < pose[i].size(); j++) {
            fprintf(fgnup, "%g ", pose[i][j].X1());
            fprintf(fgnup, "%g\n", pose[i][j].Y1());
            fprintf(fgnup, "%g ", pose[i][j].X2());
            fprintf(fgnup, "%g\n\n", pose[i][j].Y2());
        }
        fprintf(fgnup, "e\n");
    }


    fflush(fgnup);
    fclose(fgnup);
}


void plotObjects(const char * filename, vector<Object> singlePose, vector<vector<Object> > pose, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }
    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    // fprintf(fgnup, "set yrange[%d:%d]\n", 3000, 6000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, -2000);
    fprintf(fgnup, "plot ");

    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoints %d 19, \\\n", 1); //for objects/3rd argument  
    for (unsigned int i = 0; i < pose.size(); i++) {
        fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d", i + 2); //for second argument
        fprintf(fgnup, ", \\\n");
    }
    fprintf(fgnup, "\"-\" ti \"Odo\" with lines %d\n", 1); //for singlePose/1st argument

    // Plot Objects/3rd argument
    for (int i = 0; i<int(Objects.size()); i++) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());

    }
    fprintf(fgnup, "e\n");

    // Plot Robot Positions second argument
    for (unsigned int i = 0; i < pose.size(); i++) {
        for (unsigned int j = 0; j < pose[i].size(); j++) {
            fprintf(fgnup, "%g ", pose[i][j].X1());
            fprintf(fgnup, "%g\n", pose[i][j].Y1());
            fprintf(fgnup, "%g ", pose[i][j].X2());
            fprintf(fgnup, "%g\n\n", pose[i][j].Y2());
        }
        fprintf(fgnup, "e\n");
    }

    // Plot singlePose first argument
    for (int i = 0; i<int(singlePose.size()); i++) {
        fprintf(fgnup, "%g ", singlePose[i].X1());
        fprintf(fgnup, "%g\n", singlePose[i].Y1());
        fprintf(fgnup, "%g ", singlePose[i].X2());
        fprintf(fgnup, "%g\n\n", singlePose[i].Y2());

    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

void plotObjects(const char * filename, vector<Object> allPreviousRPosition, vector<Object> singlePose, 
        vector<vector<Object> > pose, vector<Object> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }
    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
    // fprintf(fgnup, "set yrange[%d:%d]\n", 3000, 6000);
    //fprintf(fgnup, "set xrange[%d:%d]\n", -5000, -2000);
    fprintf(fgnup, "plot ");

    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoints %d 19, \\\n", 1); //for objects/3rd argument  
    for (unsigned int i = 0; i < pose.size(); i++) {
        fprintf(fgnup, "\"-\" ti \"Robot Positions\" with lines %d", i + 2); //for second argument
        fprintf(fgnup, ", \\\n");
    }
    fprintf(fgnup, "\"-\" ti \"Odo\" with lines %d\n", 1); //for singlePose/2nd argument

    // Plot Objects/3rd argument
    for (int i = 0; i<int(Objects.size()); i++) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());

    }
    fprintf(fgnup, "e\n");

    // Plot Robot Positions second argument
    for (unsigned int i = 0; i < pose.size(); i++) {
        for (unsigned int j = 0; j < pose[i].size(); j++) {
            fprintf(fgnup, "%g ", pose[i][j].X1());
            fprintf(fgnup, "%g\n", pose[i][j].Y1());
            fprintf(fgnup, "%g ", pose[i][j].X2());
            fprintf(fgnup, "%g\n\n", pose[i][j].Y2());
        }
        fprintf(fgnup, "e\n");
    }

    // Plot singlePose first argument
    for (int i = 0; i<int(singlePose.size()); i++) {
        fprintf(fgnup, "%g ", singlePose[i].X1());
        fprintf(fgnup, "%g\n", singlePose[i].Y1());
        fprintf(fgnup, "%g ", singlePose[i].X2());
        fprintf(fgnup, "%g\n\n", singlePose[i].Y2());

    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}


// Plot a combination of points and surfaces

void plotPoints(const char * filename, vector<Point> points) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned int i = 0; i < points.size(); i++) {
        minX = min(minX, points[i].X());
        maxX = max(maxX, points[i].X());
        minY = min(minY, points[i].Y());
        maxY = max(maxY, points[i].Y());
    }


    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 20\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Surfaces\" with points 3 19\n");
    //fprintf(fgnup, "\"-\" ti \"Points\" with points 1 2\n");
    //fprintf(fgnup, "\"-\" ti \"Occluding edges\" with points 2 4\n");

    //    // Plot surfaces
    //    for (vector<Surface>::const_iterator surfIt = surfaces.begin(); surfIt != surfaces.end(); ++surfIt) {
    //        fprintf(fgnup, "%g ", surfIt->getX1());
    //        fprintf(fgnup, "%g\n", surfIt->getY1());
    //        fprintf(fgnup, "%g ", surfIt->getX2());
    //        fprintf(fgnup, "%g\n\n", surfIt->getY2());
    //    }
    //    fprintf(fgnup, "e\n");

    // Plot points
    for (unsigned int i = 0; i < points.size(); i++) {
        fprintf(fgnup, "%g ", points[i].X());
        fprintf(fgnup, "%g\n", points[i].Y());
    }
    fprintf(fgnup, "e\n");

    //    // Plot occluding edges
    //    bool noOccluding = true;
    //    for (vector<Surface>::const_iterator surfIt = surfaces.begin(); surfIt != surfaces.end(); ++surfIt) {
    //        if (surfIt->isP1Occluding()) {
    //            noOccluding = false;
    //            fprintf(fgnup, "%g ", surfIt->getX1());
    //            fprintf(fgnup, "%g\n", surfIt->getY1());
    //        }
    //        if (surfIt->isP2Occluding()) {
    //            noOccluding = false;
    //            fprintf(fgnup, "%g ", surfIt->getX2());
    //            fprintf(fgnup, "%g\n", surfIt->getY2());
    //        }
    //    }
    //    if (noOccluding) // For the special case that there's no occluding edge at all
    //        fprintf(fgnup, "%g %g\n", 0.0, 0.0);
    //
    //    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

void plotPoints(const char * filename, vector<PointXY> points) {
    vector<Point> newPoints;
    for (unsigned int i = 0; i < points.size(); i++) {
        newPoints.push_back(Point(points[i].getX(), points[i].getY()));
    }
    plotPoints(filename, newPoints);
}

void plotpathandobject(const char * filename, vector<Object> pose, vector<Object> Objects1, 
                                                                                     vector<Object> Objects2) 
{
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup)
    {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth %d\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y,5);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);


    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with linespoint 1, \\\n"); //for first argument
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with linespoint 2, \\\n"); //for second argument
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines 1, \\\n"); //for third argument
    //fprintf(fgnup, "\"-\" ti \"Paths\" with lines 5\n"); 
   
    // Plot lines for robot - first argument
    for (int i = 0; i<int(pose.size()); i++) 
    {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot target Objects - 2nd argument
    for (int i = 0; i<int(Objects1.size()); i++) 
    {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects - 3rd argument
   for (int i = 0; i<int(Objects2.size()); i++) 
    {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");
   
    /*
    for(int i = 0; i < allpositions.size(); i++)
    {
        fprintf(fgnup, "%g ", allpositions[i].X());
        fprintf(fgnup, "%g\n", allpositions[i].Y());
        fprintf(fgnup, "%g ", allpositions[i+1].X());
        fprintf(fgnup, "%g\n\n", allpositions[i+1].Y());
    }
    fprintf(fgnup, "e\n");
*/

    fflush(fgnup);
    fclose(fgnup);

}

void plotExitsandBoundary(const char * filename, vector<Exit> exits) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(exits.size()); i++) 
    {
        minX = min(minX, exits[i].X1());
        minX = min(minX, exits[i].X2());
        maxX = max(maxX, exits[i].X1());
        maxX = max(maxX, exits[i].X2());
        minY = min(minY, exits[i].Y1());
        minY = min(minY, exits[i].Y2());
        maxY = max(maxY, exits[i].Y1());
        maxY = max(maxY, exits[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;

    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth 10\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);

    fprintf(fgnup, "plot \"-\" ti \"Objects\" with linespoints 1 19, \n");

    // Plot Objects
    for (int i = 1; i <= int(exits.size()); i++) 
    {
            //if(Objects[i].getASRNo() == 1) {
            fprintf(fgnup, "%g ", exits[i-1].X1());
            fprintf(fgnup, "%g\n", exits[i-1].Y1());
            fprintf(fgnup, "%g ", exits[i-1].X1());
            fprintf(fgnup, "%g\n\n", exits[i-1].Y1());
            //}
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}


void plotObjectsofexits(const char * filename, vector<Object> pose, vector<Exit> Objects) {
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects.size()); i++) {
        minX = min(minX, Objects[i].X1());
        minX = min(minX, Objects[i].X2());
        maxX = max(maxX, Objects[i].X1());
        maxX = max(maxX, Objects[i].X2());
        minY = min(minY, Objects[i].Y1());
        minY = min(minY, Objects[i].Y2());
        maxY = max(maxY, Objects[i].Y1());
        maxY = max(maxY, Objects[i].Y2());
    }

    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth %d\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y, 5);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);
    //    fprintf(fgnup, "set yrange[%d:%d]\n", -7000, 27000);
    //    fprintf(fgnup, "set xrange[%d:%d]\n", -5000, 11000);
    
     //to print id
//      for (int i = 0; i<int(Objects.size()); i++) {
//    fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects[i].getID(),Objects[i].mpX(),Objects[i].mpY()+200);
//      }

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with lines %d, \\\n", 1);
    fprintf(fgnup, "\"-\" ti \"Objects\" with linespoint %d 19\n", 3);

    // Plot pose
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects
    for (int i = 0; i<int(Objects.size()); i++) {
        //if(Objects[i].getASRNo() == 1) {
        fprintf(fgnup, "%g ", Objects[i].X1());
        fprintf(fgnup, "%g\n", Objects[i].Y1());
        fprintf(fgnup, "%g ", Objects[i].X2());
        fprintf(fgnup, "%g\n\n", Objects[i].Y2());
        //}
    }
    fprintf(fgnup, "e\n");

    fflush(fgnup);
    fclose(fgnup);
}

void plotObjectsOf3KindswithExits(const char * filename, vector<Object> pose, vector<Object> Objects1, vector<Exit> Objects2) 
{
    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (int i = 0; i<int(Objects1.size()); i++) {
        minX = min(minX, Objects1[i].X1());
        minX = min(minX, Objects1[i].X2());
        maxX = max(maxX, Objects1[i].X1());
        maxX = max(maxX, Objects1[i].X2());
        minY = min(minY, Objects1[i].Y1());
        minY = min(minY, Objects1[i].Y2());
        maxY = max(maxY, Objects1[i].Y1());
        maxY = max(maxY, Objects1[i].Y2());
    }

    for (int i = 0; i<int(Objects2.size()); i++) {
        minX = min(minX, Objects2[i].X1());
        minX = min(minX, Objects2[i].X2());
        maxX = max(maxX, Objects2[i].X1());
        maxX = max(maxX, Objects2[i].X2());
        minY = min(minY, Objects2[i].Y1());
        minY = min(minY, Objects2[i].Y2());
        maxY = max(maxY, Objects2[i].Y1());
        maxY = max(maxY, Objects2[i].Y2());
    }


    for (int i = 0; i<int(pose.size()); i++) {
        minX = min(minX, pose[i].X1());
        minX = min(minX, pose[i].X2());
        maxX = max(maxX, pose[i].X1());
        maxX = max(maxX, pose[i].X2());
        minY = min(minY, pose[i].Y1());
        minY = min(minY, pose[i].Y2());
        maxY = max(maxY, pose[i].Y1());
        maxY = max(maxY, pose[i].Y2());
    }

    // Make sure x and y have the same range so the image isn't skewed
    double xRange = maxX - minX;
    double yRange = maxY - minY;
    double diff = yRange - xRange;
    if (diff > 0) {
        minX -= diff / 2.0;
        maxX += diff / 2.0;
    } else {
        minY -= -diff / 2.0;
        maxY += -diff / 2.0;
    }

    // Add a border to the image
    double border = (maxX - minX) * PLOT_BORDER_FACTOR; // x and y now have the same range
    minX -= border;
    maxX += border;
    minY -= border;
    maxY += border;


    fprintf(fgnup, "set terminal png size %d,%d nocrop linewidth %d\n", PLOT_RESOLUTION_X, PLOT_RESOLUTION_Y,5);
    fprintf(fgnup, "set output \"%s\"\n", filename);
    fprintf(fgnup, "set yrange[%g:%g]\n", minY, maxY);
    fprintf(fgnup, "set xrange[%g:%g]\n", minX, maxX);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1500, 7000);//for thesis first view
//        fprintf(fgnup, "set xrange[%d:%d]\n", -3000, 6000);
//    fprintf(fgnup, "set yrange[%d:%d]\n", -1000, 16000);
//        fprintf(fgnup, "set xrange[%d:%d]\n", -4000, 8000);

    //to print id
//    for (int i = 0; i<int(Objects2.size()); i++) {
//        fprintf(fgnup, "set label \"(%d)\" at %g,%g\n", Objects2[i].getID(), Objects2[i].mpX(), Objects2[i].mpY() + 400);
//    }

    fprintf(fgnup, "plot \"-\" ti \"Robot Positions\" with linespoint 1, \\\n"); //for first argument
    fprintf(fgnup, "\"-\" ti \"Target Objects\" with linespoint 2, \\\n"); //for second argument
    fprintf(fgnup, "\"-\" ti \"Objects\" with lines 3\n"); //for third argument

   
    // Plot lines for robot - first argument
    for (int i = 0; i<int(pose.size()); i++) {
        fprintf(fgnup, "%g ", pose[i].X1());
        fprintf(fgnup, "%g\n", pose[i].Y1());
        fprintf(fgnup, "%g ", pose[i].X2());
        fprintf(fgnup, "%g\n\n", pose[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot target Objects - 2nd argument
    for (int i = 0; i<int(Objects1.size()); i++) {
        fprintf(fgnup, "%g ", Objects1[i].X1());
        fprintf(fgnup, "%g\n", Objects1[i].Y1());
        fprintf(fgnup, "%g ", Objects1[i].X2());
        fprintf(fgnup, "%g\n\n", Objects1[i].Y2());
    }
    fprintf(fgnup, "e\n");

    // Plot Objects - 3rd argument
    for (int i = 0; i<int(Objects2.size()); i++) {
        fprintf(fgnup, "%g ", Objects2[i].X1());
        fprintf(fgnup, "%g\n", Objects2[i].Y1());
        fprintf(fgnup, "%g ", Objects2[i].X2());
        fprintf(fgnup, "%g\n\n", Objects2[i].Y2());
    }
    fprintf(fgnup, "e\n");
    
   


    fflush(fgnup);
    fclose(fgnup);
}





/*
//by arthur
void drawTwoOverlappingPics(vector<Object> ViewA, Point robotPositionA, vector<Object> ViewB, Point robotPositionB) {
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double> > polygon;
    typedef boost::geometry::model::d2::point_xy<double> point_type;
    char coordinates_1[10000] = {};
    char coordinates_2[10000] = {};
    
    char ImgFileName1[80];
    char ImgFileName2[80];
    char ImgFileName3[80];

    char para_1[10000];
    char para_2[10000];

    polygon poly1, poly2, p;
    std::deque<polygon> output;

    sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, robotPositionA.X(), " ", robotPositionA.Y(), ",");

    //read the coordinates of each points, 
    for (int i = 0; i < ViewA.size(); i++) {
        sprintf(coordinates_1, "%s%f%s%f%s", coordinates_1, ViewA[i].X1(), " ", ViewA[i].Y1(), ",");
        sprintf(coordinates_1, "%s%f%s%f", coordinates_1, ViewA[i].X2(), " ", ViewA[i].Y2());
        if (i != ViewA.size() - 1) {
            sprintf(coordinates_1, "%s%s", coordinates_1, ",");
        } else {
            //sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", ViewA[0].X1(), " ", ViewA[0].Y1());
            sprintf(coordinates_1, "%s%s%f%s%f", coordinates_1, ",", robotPositionA.X(), " ", robotPositionA.Y());
        }

    }
    cout << "coordinates_1:" << coordinates_1 << endl;

    sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, robotPositionB.X(), " ", robotPositionB.Y(), ",");
    for (int i = 0; i < ViewB.size(); i++) {
        sprintf(coordinates_2, "%s%f%s%f%s", coordinates_2, ViewB[i].X1(), " ", ViewB[i].Y1(), ",");
        sprintf(coordinates_2, "%s%f%s%f", coordinates_2, ViewB[i].X2(), " ", ViewB[i].Y2());

        if (i != ViewB.size() - 1) {
            sprintf(coordinates_2, "%s%s", coordinates_2, ",");
        } else {
            //sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", ViewB[0].X1(), " ", ViewB[0].Y1());
            sprintf(coordinates_2, "%s%s%f%s%f", coordinates_2, ",", robotPositionB.X(), " ", robotPositionB.Y());
        }

    }

    // std::cout << "coordinates" << coordinates << std::endl;
    sprintf(para_1, "%s%s%s", "POLYGON((", coordinates_1, "))");
    sprintf(para_2, "%s%s%s", "POLYGON((", coordinates_2, "))");

    // Calculate the area of a cartesian polygon

    bg::read_wkt(para_1, poly1);
    bg::read_wkt(para_2, poly2);
    sprintf(ImgFileName1, "%s", "intersectionmap1.svg");
    sprintf(ImgFileName2, "%s", "intersectionmap2.svg");
    sprintf(ImgFileName3, "%s", "intersectionmap3.svg");
    std::ofstream svg1(ImgFileName1);
    std::ofstream svg2(ImgFileName2);
    std::ofstream svg3(ImgFileName3);

    boost::geometry::svg_mapper<point_type> mapper1(svg1, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper2(svg2, 2000, 2000);
    boost::geometry::svg_mapper<point_type> mapper3(svg3, 2000, 2000);
    // Add geometries such that all these geometries fit on the map



    bg::intersection(poly1, poly2, output);

    BOOST_FOREACH(p, output) {

    }

    mapper1.add(poly1);

    mapper2.add(poly1);
    mapper2.add(poly2);

    mapper3.add(poly1);
    mapper3.add(poly2);
    mapper3.add(p);


    mapper1.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper2.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly1, "fill-opacity:0.8;fill:rgb(255,0,0);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(poly2, "fill-opacity:0.8;fill:rgb(0,0,255);stroke:rgb(0,0,153);stroke-width:2", 3);
    mapper3.map(p, "fill-opacity:0.8;fill:rgb(0,255,0);stroke:rgb(0,0,153);stroke-width:2", 3);

}
*/