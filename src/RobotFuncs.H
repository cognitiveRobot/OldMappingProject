/*
 *  RobotFuncs: helper functions for robot movement
 *  TODO: clean this up!
 *
 *  Author unknown?
 */
#ifndef _ROBOTFUNCS_H_
#define _ROBOTFUNCS_H_

#include <iostream>
#include <cmath>
#include "Aria.h"

const double DIR_TOLERANCE = 5;
const double MOVE_TOLERANCE = 100;
const int TIMEOUT = 8000;
const int SHORT_PAUSE = 100;

inline int getSonarRange(ArRobot &robot, int num)
{
  int range;

  robot.lock();
  range = robot.getSonarRange(num);
  robot.unlock();

  return range;
}

inline void setVel(ArRobot &robot, double velocity)
{
  robot.lock();
  robot.setVel(velocity);
  robot.unlock();
}

inline void setRotVel(ArRobot &robot, double rotvel)
{
  robot.lock();
  robot.setRotVel(rotvel);
  robot.unlock();
}

inline void setHeading(ArRobot &robot, double heading)
{
  ArTime start;

  robot.lock();
  //cout << "\n" << "Here  " << heading << "\n";
  robot.setDeltaHeading(heading);
  robot.unlock();

  start.setToNow();
  while (1)
  {
    robot.lock();
    if (robot.isHeadingDone(DIR_TOLERANCE))
    {
      //      printf("Finished turn\n");
      robot.unlock();
      break;
    }
    if (start.mSecSince() > TIMEOUT)
    {
      printf("turn timed out\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(SHORT_PAUSE);
  }
}

inline void move(ArRobot &robot, double distance)
{
  ArTime start;

  robot.lock();
  //cout << "\n" << "Here  " << distance << "\n";
  robot.move(distance);
  robot.unlock();

  start.setToNow();
  while (1)
  {
    robot.lock();
    if (robot.isMoveDone(MOVE_TOLERANCE))
    {
      //      printf("Finished turn\n");
      robot.unlock();
      break;
    }
    if (start.mSecSince() > TIMEOUT)
    {
      printf("move timed out\n");
      robot.unlock();
      break;
    }
    robot.unlock();
    ArUtil::sleep(SHORT_PAUSE);
  }
}

inline void moveDistance(ArRobot &robot, double distance)
{
  ArPose prevpos,curpos;
  double distance_Travelled = 0;

  robot.lock();
  prevpos = curpos = robot.getPose();
  robot.unlock();
  setVel(robot,225);

  while (distance - 50 > distance_Travelled)
    {
      robot.lock();
      curpos = robot.getPose();
      robot.unlock();

      distance_Travelled = curpos.findDistanceTo(prevpos);
    }

  robot.stop();
    ArUtil::sleep(SHORT_PAUSE);

}


inline void turnAngle(ArRobot &robot, double angle)
{
  ArPose curpos;
  double angle_Rotated = 0;
  int velocity = 0;

  ArPose startposition;

  if (angle >= 0)
    {
      velocity = 45;
    }
  else
    {
      velocity = -45;
    }


  robot.lock();
  curpos = startposition = robot.getPose();
  robot.unlock();

  setRotVel(robot,velocity);

  while (abs(angle) > abs(angle_Rotated))
    {
      robot.lock();
      curpos = robot.getPose();
      robot.unlock();

      angle_Rotated = curpos.getTh() - startposition.getTh();

//      std::cout << curpos.getTh()<< "\n";
//      std::cout << angle_Rotated << "\n";
      ArUtil::sleep(SHORT_PAUSE);

    }
  robot.stop();

    ArUtil::sleep(SHORT_PAUSE);

}

inline void turn(ArRobot &robot, double angle)
{

  if(angle > 0)
    {
      if(angle <= 150)
	{
	  turnAngle(robot, angle);
	}
      else if ((angle > 150) && (angle <= 180))
	{
	  turnAngle(robot, 90);
	  turnAngle(robot, angle - 90);
	}
      else if ((angle > 180) && (angle < 210))
	{
	  turnAngle(robot, -90);
	  turnAngle(robot, angle - 360 + 90);
	}
      else if (angle >= 210)
	{
	  turnAngle(robot, angle - 360);
	}
    }

  else
    {
      if (angle >= -150)
	{
	  turnAngle(robot, angle);
	}
      else if ((angle < -150) && (angle >= -180))
	{
	  turnAngle(robot, -90);
	  turnAngle(robot, angle + 90);
	}
      else if ((angle < -180) && (angle > -210))
	{
	  turnAngle(robot, 90);
	  turnAngle(robot, angle + 360 - 90);
	}
      else if (angle <= -210)
	{
	  turnAngle(robot, angle + 360);
	}
    }

}
#endif
