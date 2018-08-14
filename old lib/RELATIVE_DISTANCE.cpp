
//g++ example1.cpp -o example -O2 -larmadillo

#define ARMA_DONT_PRINT_ERRORS
#include "RELATIVE_DISTANCE.h"
#include "COMPASS.h"
#include <iostream>
#include <cmath>
#include <armadillo>
#include <fstream>

using namespace std;
using namespace arma;

const float PI = (atan(1)*4);

COMPASS comp;

void rel_distance::init(void){
	relative_distance.ones();
}


double rel_distance::relative_distance(double Ta, double Tb, double Tc, double depth){
	init();
	double dist_a = 0.0, dist_b = 0.0, dist_c = 0.0, rel_distA = 0.0, rel_distB = 0.0, rel_distC = 0.0, XAxis, YAxis;

	dist_a = W_speed * Ta;
	dist_b = W_speed * Tb;
	dist_c = W_speed * Tc;

	rel_distA = sqrt(pow(dist_a,2) + pow(depth,2));
	rel_distB = sqrt(pow(dist_b,2) + pow(depth,2));
	rel_distC = sqrt(pow(dist_c,2) + pow(depth,2));

	relative_distance =  leastSqueare(Pos_x1, Pos_y1, Pos_x2, Pos_y2, Pos_x3, Pos_y3, rel_distA, rel_distB, rel_distC);

	XAxis = relative_distance.col(0);
	YAxis = relative_distance.col(1);

	return sqrt(pow(XAxis,2) + pow(YAxis,2));
}


double rel_distance::real_Bearing(){

	float bearing;
	double XAxis_o, YAxis_o, XAxis_r, YAxis_r, phi, gamma;
	comp.init();
	bearing = comp.get_Bearing();

	XAxis_o = relative_distance.col(0);
	YAxis_o = relative_distance.col(1);

	phi = bearing - 90;

	XAxis_r = XAxis_o*cos(phi) - YAxis_o*sin(phi);
	YAxis_r = XAxis_o*sin(phi) + YAxis_o*con(phi);

	gamma = atan(YAxis_r,XAxis_r) * (180.0/PI);	

	if(XAxis_r > 0 && YAxis_r > 0)
		bearing = 90 - gamma;
	if(XAxis_r < 0 && YAxis_r > 0)
		bearing = 180 - gamma;
	if(XAxis_r < 0 && YAxis_r < 0)
		bearing = 180 + gamma;
	if(XAxis_r > 0 && YAxis_r < 0)
		bearing = 360 - gamma;

	return bearing;
}


void rel_distance::setSpeaker_1(float x1, float y1){
	Pos_x1 = x1;
	Pos_y1 = y1;
}
	

void rel_distance::setSpeaker_2(float x2, float y2){
	Pos_x2 = x2;
	Pos_y2 = y2;
}


void rel_distance::setSpeaker_3(float x3, float y3){
	Pos_x3 = x3;
	Pos_y3 = y3;
}


//int x1,int y1,int x2,int y2,int x3,int y3,int a,int b,int c,int xc,int yc
mat rel_distance::leastSqueare(double x1,double y1,double x2,double y2,double x3,double y3,double a,double b,double c)
{
	mat A(2,2); 
	mat B(2,1);
	mat R(2,1);

	A.ones();
	B.ones();

	A 	<< x3 - x1 << y3 - y1 << endr
		<< x3 - x2 << y3 - y2 << endr;

	A = A * 2;	

	B 	<< (pow(a,2) - pow(c,3)) - (pow(x1,2) - pow(x3,2)) - (pow(y1,2) - pow(y3,2)) <<endr
		<< (pow(b,2) - pow(c,2)) - (pow(x2,2) - pow(x3,2)) - (pow(y2,2) - pow(y3,2)) << endr;

	R = inv(A.t()*A)*A.t()*B;

	return R;
}


