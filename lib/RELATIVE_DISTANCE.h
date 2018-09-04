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

#define W_speed 1496.73f // T = 25º , V = 1.402385(10^3)+5.038813(T)−5.799136(10^−2)(T^ 2)+3.287156(10^−4)(T^3)−1.398845 (10^−6) (T^4) +2.787860(10^−9)(T^5)

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

	mat relative_distance(2,1);
	
	void init(void);
		
	double relative_distance(double Ta, double Tb, double Tc, double depth); 
	
	void setSpeaker_1(float x1, float y1);
	void setSpeaker_2(float x2, float y2);
	void setSpeaker_3(float x3, float y3);

	
private: 
	
	mat leastSqueare(double x1,double y1,double x2,double y2,double x3,double y3,double a,double b,double c);


};
#endif