/* 
 * File:   Comparison.h
 * Author: mhossain
 *
 * Created on 4 June 2013, 2:12 PM
 */

#ifndef COMPARISON_H
#define	COMPARISON_H

void readInput(char *filename);
void readDPSLAMInput(char *filename);
void readCarmenInput(char *filename);

void readAlbotInput4DPSLAM(int steps);

void readDPSLAMInput4DPSLAM(char *filename);

void readAlbotKBCInput();

void carmenToCarmenConverter();

void carmenToAlbot1Converter();//03-12-13


#endif	/* COMPARISON_H */

