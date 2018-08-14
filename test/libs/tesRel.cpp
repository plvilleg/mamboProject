#define ARMA_DONT_PRINT_ERRORS
#include <iostream>
#include <cmath>
#include <armadillo>
#include <fstream>
#include "COMPASS.h"
#define _USE_MATH_DEFINES

const float PI = (atan(1)*4); 
#define W_speed 1496.73f // T = 25º , V = 1.402385(10^3)+5.038813(T)−5.799136(10^−2)(T^ 2)+3.287156(10^−4)(T^3)−1.398845 (10^−6) (T^4) +2.787860(10^−9)(T^5)

using namespace std;
using namespace arma;

COMPASS comp;
mat D(2,1);
float Pos_x1, Pos_y1, Pos_x2, Pos_y2, Pos_x3, Pos_y3;


void init(void){
	D.ones();
}

mat leastSqueare(float x1,float y1,float x2,float y2,float x3,float y3,float a,float b,float c)
{
	mat A(2,2); 
	mat B(2,1);
	mat R(2,1);

	A.ones();
	B.ones();

	A 	<< x3 - x1 << y3 - y1 << endr
		<< x3 - x2 << y3 - y2 << endr;

	A = A * 2;	

	B 	<< (pow(a,2.0) - pow(c,3.0)) - (pow(x1,2.0) - pow(x3,2.0)) - (pow(y1,2.0) - pow(y3,2.0)) <<endr
		<< (pow(b,2.0) - pow(c,2.0)) - (pow(x2,2.0) - pow(x3,2.0)) - (pow(y2,2.0) - pow(y3,2.0)) << endr;

	R = inv(A.t()*A)*A.t()*B;

	return R;
}

float relative_distance(float Ta, float Tb, float Tc, float depth){
	init();
	float dist_a = 0.0, dist_b = 0.0, dist_c = 0.0, rel_distA = 0.0, rel_distB = 0.0, rel_distC = 0.0, XAxis, YAxis;

	printf("OK3 \n");

	dist_a = W_speed * Ta;
	dist_b = W_speed * Tb;
	dist_c = W_speed * Tc;

	rel_distA = sqrt(pow(dist_a,2) + pow(depth,2));
	rel_distB = sqrt(pow(dist_b,2) + pow(depth,2));
	rel_distC = sqrt(pow(dist_c,2) + pow(depth,2));

	printf("OK4 \n");
	
	D = leastSqueare(Pos_x1, Pos_y1, Pos_x2, Pos_y2, Pos_x3, Pos_y3, rel_distA, rel_distB, rel_distC);

	XAxis = D(0,0);
	YAxis = D(1,0);

	return sqrt(pow(XAxis,2) + pow(YAxis,2));
}


float real_Bearing(){

	float bearing;
	float XAxis_o, YAxis_o, XAxis_r, YAxis_r, phi, gamma;
	comp.init();
	bearing = comp.get_Bearing();

	XAxis_o = D(0,0);
	YAxis_o =D(1,0);

	phi = bearing - 90;

	XAxis_r = XAxis_o*cos(phi) - YAxis_o*sin(phi);
	YAxis_r = XAxis_o*sin(phi) + YAxis_o*cos(phi);

	gamma = atan2(YAxis_r,XAxis_r) * (180.0/PI);	

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


void setSpeaker_1(float x1, float y1){
	Pos_x1 = x1;
	Pos_y1 = y1;
}
	

void setSpeaker_2(float x2, float y2){
	Pos_x2 = x2;
	Pos_y2 = y2;
}


void setSpeaker_3(float x3, float y3){
	Pos_x3 = x3;
	Pos_y3 = y3;
}


//int x1,int y1,int x2,int y2,int x3,int y3,int a,int b,int c,int xc,int yc


int main()
{
	float distance;
	float bearingT;

	printf("OK1 \n");
	
	setSpeaker_1(0,1);
	setSpeaker_2(0.988,-0.1542);
	setSpeaker_3(-0.988,-01542);

	printf("OK2 \n");

	distance = relative_distance(0.0001,0.0002,0.0001,5);	
	
	bearingT = real_Bearing();

	printf("Distancia: %2.3f \n",distance);
	printf("Bearing: %2.3f \n",bearingT);
	return 0;
}