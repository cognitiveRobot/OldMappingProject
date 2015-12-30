/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   crossExits_test.cpp
 * Author: Lyn
 *
 * Created on December 17, 2015, 3:42 PM
 */
#include "Aria.h"
#include <iostream>
#include <cstdlib>

using namespace std;
int init_Robot(int argc, char** argv);


ArRobot robot;
ArSick sick;

/*
 * 
 */
int main(int argc, char** argv) 
{

        init_Robot(argc,argv);

         return 0;
}

int init_Robot(int argc, char** argv)
{
            Aria::init();
            ArSimpleConnector connector(&argc, argv);

            if (!connector.parseArgs() || argc > 1) 
            {
                Aria::logOptions();
                Aria::shutdown();
                Aria::exit(1);
            }
            
            robot.addRangeDevice(&sick);
            // Try to connect, if we fail exit
            if (!connector.connectRobot(&robot)) 
            {
                cout << "Could not connect to robot... exiting" << endl;
                Aria::shutdown();
                return 1;
            }
            // Turn on the motors, turn off amigobot sounds
            robot.runAsync(true);
            robot.comInt(ArCommands::ENABLE, 1);
            robot.comInt(ArCommands::SOUNDTOG, 0);
            robot.lock();
            robot.clearDirectMotion();
            robot.unlock();
            
            connector.setupLaser(&sick);
            sick.runAsync();
            if (!sick.blockingConnect()) 
            {
                cout << "Could not connect to SICK laser... exiting" << endl;
                robot.disconnect();
                Aria::shutdown();
                return 1;
            }
            ArUtil::sleep(1500);
}

void moveExcusion()
{
    
}
