#ifndef RELATIVE_DISTANCE_h
#define RELATIVE_DISTANCE_h

#define ARMA_DONT_PRINT_ERRORS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <fstream>
#include <armadillo>
#include "config.h"

using namespace arma;

#ifndef SOURCE_STRUCT_H
#define SOURCE_STRUCT_H
struct Speaker
{
	float X;
	float Y;	
};
#endif



class RelativeDistace
{
public:

	Speaker sp1, sp2, sp3;

	mat relative_dist;
	
	
	RelativeDistace()
		: relative_dist(2,1) {
	}

	void init(void);
		
	double relative_distance(double Ta, double Tb, double Tc, double depth); 
	
	void setSpeaker_1(double x1, double y1);
	void setSpeaker_2(double x2, double y2);
	void setSpeaker_3(double x3, double y3);

	
private: 
	
	mat leastSqueare(double x1,double y1,double x2,double y2,double x3,double y3,double a,double b,double c);


};
#endif