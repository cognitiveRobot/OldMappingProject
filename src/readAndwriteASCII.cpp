#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cmath>
#include <dirent.h>
#include "Object.H"
#include "readAndwriteASCII.H"
#include "PointAndSurface.H"

using namespace std;

/*
      This program reads in an ASCII file and fill up the data into vector of type Object
      Object class consists of id, co-od of p1 & p2, nearness, Occluding status of p1 & p2.

      Zati

	modified by 
	
	mhossain
	7 September, 2010
*/

vector<Object> readASCII (char *filename) 
{

  //vector <double> vec;
  vector<Object> Objects;
  ifstream inputFile (filename, ios::in);
  if (! inputFile)
  {
    cout << "Error opening " <<filename<<" .."<<endl;
    return Objects;
  }
  else
  {

    // start reading from the third data (ignore ASCII header)
    double row, colum, x1, y1, x2, y2;
	//double x, y;
	int ns=0;
    inputFile >> row;            //cout <<" row    = "<<data<<endl;
    inputFile >> colum;            //cout <<" column = "<<data<<endl;
    //inputFile >> data;            //cout <<"push this data = "<<data<<endl;

      inputFile >> x1;          //cout <<"push this data = "<<data<<endl;
      inputFile >> y1;
      inputFile >> x2;
      inputFile >> y2;	
  
/*
	x=x1+x2;
	y=y1+y2;
	double d=sqrt(x*x+y*y);
	if(d<15000)
	ns=1;
	else
	ns=0;*/
    int id=1;
    while(!inputFile.eof())
    {
	 Object s(x1, y1-20, x2, y2-20, id, ns);//0 for group id
	Objects.push_back(s);
    //  vec.push_back(data);
      inputFile >> x1;          //cout <<"push this data = "<<data<<endl;
      inputFile >> y1;
      inputFile >> x2;
      inputFile >> y2;	


      
      id++;   
    }	 
  }
//cout<<"was here"<<endl;
  inputFile.close();
vector<Object> result=setOccludingEdges(Objects);
//cout<<"was here"<<endl;
//double th=600;
//	int idd=1;
//	result[0].setGID(idd);
//	for(int i=0;i<int(result.size());i++)
//	{	
//		if(result[i].distP2ToP1(result[i+1])>th)
//		{
//			result[i].setGID(idd);
//			idd++;
//			result[i+1].setGID(idd);
//		}
//		else
//		{
//			result[i+1].setGID(idd);
//		}
//	}
        //cout<<"finished"<<endl;
  return result;
}

//This program reads in an ASCII file and fill up the data into vector of type double
vector <double> readCoordTrans (char *filename) {

  vector <double> vec;

  ifstream inputFile (filename, ios::in);
  if (! inputFile){
    cout << "Error opening " <<filename<<" .."<<endl;
    exit(1);
  }
  else{

    // start reading from the third data (ignore ASCII header)
    double data;
    inputFile >> data;            //cout <<" row    = "<<data<<endl;
    inputFile >> data;            //cout <<" column = "<<data<<endl;
    inputFile >> data;            //cout <<"push this data = "<<data<<endl;

    while(!inputFile.eof()){
      vec.push_back(data);
      inputFile >> data;          //cout <<"push this data = "<<data<<endl;
    }
  }

  return vec;

}

//This program reads in an ASCII file and fill up the data into vector of type double
vector <Point> readPoints (char *filename) {

  vector <Point> vec;

  ifstream inputFile (filename, ios::in);
  if (! inputFile){
    cout << "Error opening " <<filename<<" .."<<endl;
    exit(1);
  }
  else{

     // start reading from the third data (ignore ASCII header)
    double row, colum, x1, y1;
	//double x, y;
	
    inputFile >> row;            //cout <<" row    = "<<data<<endl;
    inputFile >> colum;            //cout <<" column = "<<data<<endl;
    //inputFile >> data;            //cout <<"push this data = "<<data<<endl;

      inputFile >> x1;          //cout <<"push this data = "<<data<<endl;
      inputFile >> y1;

  
    while(!inputFile.eof())
    {	
	vec.push_back(Point(x1,y1));
   
      inputFile >> x1;          //cout <<"push this data = "<<data<<endl;
      inputFile >> y1;
     
    }	 
  }

  return vec;

}

/* Write a vector of Objects with Object id to a ASCII file
 Author 
	Thomas
	modified by

	mhossain
	7 Sept. 2010
*/	

void writeASCII(vector<Object> Objects, char * filename)
{
    ofstream outFile (filename, ios::out);

    // Output ASCII header (row and column)
    outFile << Objects.size() << " " << 5 << endl;

    // 8 digits should be more than enough
   // outFile << fixed;
    //outFile.precision(10);

    for (int i=0;i<int(Objects.size());i++)
    {
	outFile << Objects[i].getID() << " ";
        outFile << Objects[i].X1() << " ";
        outFile << Objects[i].Y1() << " ";
        outFile << Objects[i].X2() << " ";
        outFile << Objects[i].Y2() << endl;
    }

    outFile.close();
}


//from thomas
// Write a vector of surfaces to a file
void writeASCII(const vector<Surface> & surfaces, const char *filename)
{
    ofstream outFile (filename, ios::out);

    // Output ASCII header (row and column)
    outFile << surfaces.size() << " " << 4 << endl;

    // 8 digits should be more than enough
    outFile << fixed;
    outFile.precision(10);

    for (vector<Surface>::const_iterator it = surfaces.begin(); it!=surfaces.end(); ++it)
    {
        outFile << (*it).getX1() << " ";
        outFile << (*it).getY1() << " ";
        outFile << (*it).getX2() << " ";
        outFile << (*it).getY2() << endl;
    }

    outFile.close();
}

//to write Coordinate Transformation information
void writeASCII (vector <double> vec, int column, char *outputFileName) {

  ofstream outFile (outputFileName, ios::out);

  // output ASCII header (row and column)
  outFile << int(vec.size())/column <<" "<< column <<endl;

  int data_counter = 0;
  int column_counter = 1;

  for (int data=0; data<int(vec.size()); data+=column){
    for (int next=(data_counter*column); next<(column_counter*column); next++){
      outFile << vec[next] <<" ";
    }
    outFile<<endl;

    data_counter++;
    column_counter++;	
  }
}

/*it receives folder name of current directory and then deletes all files
  from that folder
		by mhossain, 
		21 September, 2010
*/
void deleteFiles(char *dirname)
{
	char path[50];
	sprintf(path, "%s%s", "./", dirname);

	DIR *dir = opendir(path);
	if(dir)
	{
		struct dirent *ent;
		while((ent = readdir(dir)) != NULL)
		{
			//puts(ent->d_name);
			char filename[50];
			sprintf(filename, "%s%s%s%s","./",dirname,"/",ent->d_name);
			//cout<<filename<<endl;
			//if(remove(filename)==-1)
			//cout<<"Error deleting file"<<endl;
			remove(filename);
		}
	}
	else
	{
		fprintf(stderr, "Error opening directory\n");
	}

}

