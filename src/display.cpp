#include <iostream>
#include <cstdlib>
#include <fstream>
#include <cmath>
#include <vector>
#include "space.h"
#include "polygons.h"
#include "convexPathPlanner.h"
#define IMG_SIZE 1000.

using namespace std;






#if defined(WIN32) || defined(WIN64)
#include <opencv/cv.h>    
#include <opencv/cvaux.h>
#include <opencv/highgui.h>


void Space::plotSpace(char * filename) {
    double scaleCoef;
    CvScalar polygonsColor = cvScalar(0, 255, 0);
    CvScalar obstaclesColor = cvScalar(255, 0, 0);

    vector<Surface> polygonsSurfaces;

    for (unsigned i = 0; i < polygons.size(); i++) {
        vector<Surface> stmp = polygons[i].getSurfaces();

        for (unsigned j = 0; j < stmp.size(); j++) {
            polygonsSurfaces.push_back(stmp[j]);
        }
    }

    // Finding xmin, xmax, ymin, ymax
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        xmin = min(xmin, polygonsSurfaces[i].getX1());
        xmin = min(xmin, polygonsSurfaces[i].getX2());
        xmax = max(xmax, polygonsSurfaces[i].getX1());
        xmax = max(xmax, polygonsSurfaces[i].getX2());
        ymin = min(ymin, -polygonsSurfaces[i].getY1());
        ymin = min(ymin, -polygonsSurfaces[i].getY2());
        ymax = max(ymax, -polygonsSurfaces[i].getY1());
        ymax = max(ymax, -polygonsSurfaces[i].getY2());
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        xmin = min(xmin, obstacles[i].getX1());
        xmin = min(xmin, obstacles[i].getX2());
        xmax = max(xmax, obstacles[i].getX1());
        xmax = max(xmax, obstacles[i].getX2());
        ymin = min(ymin, -obstacles[i].getY1());
        ymin = min(ymin, -obstacles[i].getY2());
        ymax = max(ymax, -obstacles[i].getY1());
        ymax = max(ymax, -obstacles[i].getY2());
    }

    // We want a symetry on xaxis
    //xmin = min(xmin, -xmax);
    //xmax = max(xmax, -xmin);

    xmin -= (xmax - xmin)*0.05;
    ymin -= (ymax - ymin)*0.05;
    xmax += (xmax - xmin)*0.05;
    ymax += (ymax - ymin)*0.05;

    scaleCoef = IMG_SIZE / max(xmax - xmin, ymax - ymin);

    // Image creation
    IplImage* img = cvCreateImage(cvSize(IMG_SIZE, IMG_SIZE), IPL_DEPTH_32F, 3);

    // Fill with white
    cvRectangle(img, cvPoint(0, 0), cvPoint(IMG_SIZE, IMG_SIZE), cvScalar(255, 255, 255), -1);

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        cvLine(img, cvPoint((polygonsSurfaces[i].getX1() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY1() - ymin) * scaleCoef), cvPoint((polygonsSurfaces[i].getX2() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY2() - ymin) * scaleCoef), polygonsColor, 1);
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        cvLine(img, cvPoint((obstacles[i].getX1() - xmin) * scaleCoef, (-obstacles[i].getY1() - ymin) * scaleCoef), cvPoint((obstacles[i].getX2() - xmin) * scaleCoef, (-obstacles[i].getY2() - ymin) * scaleCoef), obstaclesColor, 1);
    }


    // Window creation
    cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("mainWin", 300, 100);
    cvShowImage("mainWin", img);
    cvSaveImage(filename, img);

    // wait for a key
    cvWaitKey(0);

    cvReleaseImage(&img);

}

void PathComputationModule::plotSpaceAndPath(const char* filename, PointXY currentPos, unsigned steps_number) {
    double scaleCoef;
    CvScalar polygonsColor = cvScalar(0, 255, 0);
    CvScalar obstaclesColor = cvScalar(255, 0, 0);
    CvScalar pathColor = cvScalar(0, 0, 255);

    vector<Surface> polygonsSurfaces;

    for (unsigned i = 0; i < polygons.size(); i++) {
        vector<Surface> stmp = polygons[i].getSurfaces();

        for (unsigned j = 0; j < stmp.size(); j++) {
            polygonsSurfaces.push_back(stmp[j]);
        }
    }

    vector<PointXY> pathPoints = computeCheckPointsToBestPolygon(currentPos, steps_number);


    // Finding xmin, xmax, ymin, ymax
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        xmin = min(xmin, polygonsSurfaces[i].getX1());
        xmin = min(xmin, polygonsSurfaces[i].getX2());
        xmax = max(xmax, polygonsSurfaces[i].getX1());
        xmax = max(xmax, polygonsSurfaces[i].getX2());
        ymin = min(ymin, -polygonsSurfaces[i].getY1());
        ymin = min(ymin, -polygonsSurfaces[i].getY2());
        ymax = max(ymax, -polygonsSurfaces[i].getY1());
        ymax = max(ymax, -polygonsSurfaces[i].getY2());
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        xmin = min(xmin, obstacles[i].getX1());
        xmin = min(xmin, obstacles[i].getX2());
        xmax = max(xmax, obstacles[i].getX1());
        xmax = max(xmax, obstacles[i].getX2());
        ymin = min(ymin, -obstacles[i].getY1());
        ymin = min(ymin, -obstacles[i].getY2());
        ymax = max(ymax, -obstacles[i].getY1());
        ymax = max(ymax, -obstacles[i].getY2());
    }

    // We want a symetry on xaxis
    //xmin = min(xmin, -xmax);
    //xmax = max(xmax, -xmin);

    xmin -= (xmax - xmin)*0.05;
    ymin -= (ymax - ymin)*0.05;
    xmax += (xmax - xmin)*0.05;
    ymax += (ymax - ymin)*0.05;

    scaleCoef = IMG_SIZE / max(xmax - xmin, ymax - ymin);

    // Image creation
    IplImage* img = cvCreateImage(cvSize(IMG_SIZE, IMG_SIZE), IPL_DEPTH_32F, 3);

    // Fill with white
    cvRectangle(img, cvPoint(0, 0), cvPoint(IMG_SIZE, IMG_SIZE), cvScalar(255, 255, 255), -1);

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        cvLine(img, cvPoint((polygonsSurfaces[i].getX1() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY1() - ymin) * scaleCoef), cvPoint((polygonsSurfaces[i].getX2() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY2() - ymin) * scaleCoef), polygonsColor, 1);
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        cvLine(img, cvPoint((obstacles[i].getX1() - xmin) * scaleCoef, (-obstacles[i].getY1() - ymin) * scaleCoef), cvPoint((obstacles[i].getX2() - xmin) * scaleCoef, (-obstacles[i].getY2() - ymin) * scaleCoef), obstaclesColor, 1);
    }

    for (unsigned i = 0; i < pathPoints.size() - 1; i++) {
        cvLine(img, cvPoint((pathPoints[i].getX() - xmin) * scaleCoef, (-pathPoints[i].getY() - ymin) * scaleCoef), cvPoint((pathPoints[i + 1].getX() - xmin) * scaleCoef, (-pathPoints[i + 1].getY() - ymin) * scaleCoef), pathColor, 1);
    }
    cvCircle(img, cvPoint((currentPos.getX() - xmin) * scaleCoef, (-currentPos.getY() - ymin) * scaleCoef), 0.5 * scaleCoef, pathColor, -1);


    // Window creation
    cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("mainWin", 300, 100);
    cvShowImage("mainWin", img);
    cvSaveImage(filename, img);

    // wait for a key
    cvWaitKey(0);

    cvReleaseImage(&img);

}

void ConvexRobotPathPlanner::plotSpaceAndPath(const char* filename) {
    double scaleCoef;
    CvScalar polygonsColor = cvScalar(0, 255, 0);
    CvScalar obstaclesColor = cvScalar(255, 0, 0);
    CvScalar pathColor = cvScalar(0, 0, 255);

    vector<Surface> polygonsSurfaces;
    vector<MPolygon> polygons = myCurrentView->getPolygons();
    vector<Surface> obstacles = myCurrentView->getObstacles();

    for (unsigned i = 0; i < polygons.size(); i++) {
        vector<Surface> stmp = polygons[i].getSurfaces();

        for (unsigned j = 0; j < stmp.size(); j++) {
            polygonsSurfaces.push_back(stmp[j]);
        }
    }


    PointAndAngle currentPositionning;
    currentPositionning.point = PointXY(0, 0);
    currentPositionning.angle = 0;

    vector<PointXY> pathPoints = convertDestinationsVectToPointsVect(savedPath, currentPositionning);

   

    // Finding xmin, xmax, ymin, ymax
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        xmin = min(xmin, polygonsSurfaces[i].getX1());
        xmin = min(xmin, polygonsSurfaces[i].getX2());
        xmax = max(xmax, polygonsSurfaces[i].getX1());
        xmax = max(xmax, polygonsSurfaces[i].getX2());
        ymin = min(ymin, -polygonsSurfaces[i].getY1());
        ymin = min(ymin, -polygonsSurfaces[i].getY2());
        ymax = max(ymax, -polygonsSurfaces[i].getY1());
        ymax = max(ymax, -polygonsSurfaces[i].getY2());
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        xmin = min(xmin, obstacles[i].getX1());
        xmin = min(xmin, obstacles[i].getX2());
        xmax = max(xmax, obstacles[i].getX1());
        xmax = max(xmax, obstacles[i].getX2());
        ymin = min(ymin, -obstacles[i].getY1());
        ymin = min(ymin, -obstacles[i].getY2());
        ymax = max(ymax, -obstacles[i].getY1());
        ymax = max(ymax, -obstacles[i].getY2());
    }

    // We want a symetry on xaxis
    //xmin = min(xmin, -xmax);
    //xmax = max(xmax, -xmin);

    xmin -= (xmax - xmin)*0.05;
    ymin -= (ymax - ymin)*0.05;
    xmax += (xmax - xmin)*0.05;
    ymax += (ymax - ymin)*0.05;

    scaleCoef = IMG_SIZE / max(xmax - xmin, ymax - ymin);
    scaleCoef = 0.02;

    // Image creation
    IplImage* img = cvCreateImage(cvSize(IMG_SIZE, IMG_SIZE), IPL_DEPTH_32F, 3);

    // Fill with white
    cvRectangle(img, cvPoint(0, 0), cvPoint(IMG_SIZE, IMG_SIZE), cvScalar(255, 255, 255), -1);

    for (unsigned i = 0; i < polygonsSurfaces.size(); i++) {
        cvLine(img, cvPoint((polygonsSurfaces[i].getX1() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY1() - ymin) * scaleCoef), cvPoint((polygonsSurfaces[i].getX2() - xmin) * scaleCoef, (-polygonsSurfaces[i].getY2() - ymin) * scaleCoef), polygonsColor, 1);
    }
    for (unsigned i = 0; i < obstacles.size(); i++) {
        cvLine(img, cvPoint((obstacles[i].getX1() - xmin) * scaleCoef, (-obstacles[i].getY1() - ymin) * scaleCoef), cvPoint((obstacles[i].getX2() - xmin) * scaleCoef, (-obstacles[i].getY2() - ymin) * scaleCoef), obstaclesColor, 1);
    }

    if (savedPath.size() > 0) {
        for (unsigned i = 0; i < pathPoints.size() - 1; i++) {
            cvLine(img, cvPoint((pathPoints[i].getX() - xmin) * scaleCoef, (-pathPoints[i].getY() - ymin) * scaleCoef), cvPoint((pathPoints[i + 1].getX() - xmin) * scaleCoef, (-pathPoints[i + 1].getY() - ymin) * scaleCoef), pathColor, 1);
        }

        for (unsigned i = 1; i < pathPoints.size() - 1; i++) {
            cvCircle(img, cvPoint((pathPoints[i].getX() - xmin) * scaleCoef, (-pathPoints[i].getY() - ymin) * scaleCoef), 100 * scaleCoef, pathColor, -1);
        }
    } else {
        cerr << "Impossible to plot savedPath, not computed yet ..." << endl;
    }

    cvCircle(img, cvPoint((currentPositionning.point.getX() - xmin) * scaleCoef, (-currentPositionning.point.getY() - ymin) * scaleCoef), 200 * scaleCoef, pathColor, -1);



    // Window creation
    cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("mainWin", 600, 100);
    cvShowImage("mainWin", img);
    cvSaveImage(filename, img);

    // wait for a key
    cvWaitKey(0);

    cvReleaseImage(&img);

}





#else

void ConvexRobotPathPlanner::plotSpaceAndPath(const char* filename) {
    const char * GNUPLOT_PATH = "/usr/bin/gnuplot";
    const unsigned int PLOT_RESOLUTION_X = 2400;
    const unsigned int PLOT_RESOLUTION_Y = 2400;
    const double PLOT_BORDER_FACTOR = 0.05; // Times the original with/height of the image

    FILE * fgnup = popen(GNUPLOT_PATH, "w");
    if (!fgnup) {
        cerr << "ERROR: " << GNUPLOT_PATH << " not found" << endl;
        return;
    }


    vector<Surface> polygonSurfaces;
    vector<MPolygon> polygons = myCurrentView->getPolygons();
    vector<Surface> obstacles = myCurrentView->getObstacles();
    vector<PointXY> pathPoints;

    for (unsigned i = 0; i < polygons.size(); i++) {
        vector<Surface> stmp = polygons[i].getSurfaces();
        for (unsigned j = 0; j < stmp.size(); j++) {
            polygonSurfaces.push_back(stmp[j]);
        }
    }

    // Get the plotting range
    double minX = 0, minY = 0, maxX = 0, maxY = 0;
    for (unsigned i = 0; i < obstacles.size(); i++) {
        minX = min(minX, obstacles[i].getX1());
        minX = min(minX, obstacles[i].getX2());
        maxX = max(maxX, obstacles[i].getX1());
        maxX = max(maxX, obstacles[i].getX2());
        minY = min(minY, obstacles[i].getY1());
        minY = min(minY, obstacles[i].getY2());
        maxY = max(maxY, obstacles[i].getY1());
        maxY = max(maxY, obstacles[i].getY2());
    }
    for (unsigned i = 0; i < polygonSurfaces.size(); i++) {
        minX = min(minX, polygonSurfaces[i].getX1());
        minX = min(minX, polygonSurfaces[i].getX2());
        maxX = max(maxX, polygonSurfaces[i].getX1());
        maxX = max(maxX, polygonSurfaces[i].getX2());
        minY = min(minY, polygonSurfaces[i].getY1());
        minY = min(minY, polygonSurfaces[i].getY2());
        maxY = max(maxY, polygonSurfaces[i].getY1());
        maxY = max(maxY, polygonSurfaces[i].getY2());
    }

    if (savedPath.size() > 0) {
        PointAndAngle origin;
        origin.point = PointXY(0,0);
        origin.angle = 0;
        pathPoints = convertDestinationsVectToPointsVect(savedPath, origin);
    }

    for (unsigned i = 0; i < pathPoints.size(); i++) {
        minX = min(minX, pathPoints[i].getX());
        maxX = max(maxX, pathPoints[i].getX());
        minY = min(minY, pathPoints[i].getY());
        maxY = max(maxY, pathPoints[i].getY());
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

    fprintf(fgnup, "plot \"-\" ti \"Polygones\" with linespoints 2 19, \\\n");
    fprintf(fgnup, "\"-\" ti \"Obstacles\" with linespoints 3 19, \\\n");
    fprintf(fgnup, "\"-\" ti \"Path\" with linespoints 1 19, \\\n");



    // Plot Polygons Surfaces
    for (int i = 0; i<int(polygonSurfaces.size()); i++) {
        fprintf(fgnup, "%g ", polygonSurfaces[i].getX1());
        fprintf(fgnup, "%g\n", polygonSurfaces[i].getY1());
        fprintf(fgnup, "%g ", polygonSurfaces[i].getX2());
        fprintf(fgnup, "%g\n\n", polygonSurfaces[i].getY2());
    }
    fprintf(fgnup, "e\n");



    // Plot Obstacles
    for (unsigned i = 0; i < obstacles.size(); i++) {
        fprintf(fgnup, "%g ", obstacles[i].getX1());
        fprintf(fgnup, "%g\n", obstacles[i].getY1());
        fprintf(fgnup, "%g ", obstacles[i].getX2());
        fprintf(fgnup, "%g\n\n", obstacles[i].getY2());
    }
    fprintf(fgnup, "e\n");


    // Plot path
    if (savedPath.size() > 0) {
        for (unsigned i = 0; i < pathPoints.size() - 1; i++) {
            fprintf(fgnup, "%g ", pathPoints[i].getX());
            fprintf(fgnup, "%g\n", pathPoints[i].getY());
            fprintf(fgnup, "%g ", pathPoints[i + 1].getX());
            fprintf(fgnup, "%g\n\n", pathPoints[i + 1].getY());
        }
        //cout << "Path plotted : " << pathPoints.size() << " points" << endl;
        fprintf(fgnup, "e\n");
    } else {
        cerr << "Impossible to plot savedPath, not computed yet ..." << endl;
    }


    fflush(fgnup);
    fclose(fgnup);
}
#endif

