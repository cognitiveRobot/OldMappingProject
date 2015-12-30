#include <iostream>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include "Object.H"
#include <string>
int main (int argc, char* args[])
{
	//read what steps we want to get information from mapsettings
	std::ifstream inputFile ("mapsettings.txt", std::ios::in);
	//output these files to a integers representing the start and end
	int start = 0;
	int end = 0;
	std::string location = "";
	inputFile >> start;
	inputFile >> end;
	inputFile >> location;
	std::string inputDataLocation = "~/GuidedMapping";
	std::ifstream inputFile("~/GuidedMapping/bin/inputData/");
	inputFile.close();
}
