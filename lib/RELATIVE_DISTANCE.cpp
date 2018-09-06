
//g++ example1.cpp -o example -O2 -larmadillo

#define ARMA_DONT_PRINT_ERRORS
#include "RELATIVE_DISTANCE.h"
#include <iostream>
#include <cmath>
#include <fstream>

using namespace std;
using namespace arma;


void RelativeDistace::init(void){
	#if (DEBUG_MODE > 0)
		printf("");
	#endif

	relative_dist.ones();
	sp1.X = 0.0;
	sp1.Y = 0.0;
	sp2.X = 0.0;
	sp2.Y = 0.0;
	sp3.X = 0.0;
	sp3.Y = 0.0;
	
}

void RelativeDistace::setSpeaker_1(double x1, double y1){
	sp1.X = x1;
	sp1.Y = y1;
	#if (DEBUG_MODE > 0)
		printf("");
	#endif
}
	

void RelativeDistace::setSpeaker_2(double x2, double y2){
	sp2.X = x2;
	sp2.X = y2;
	#if (DEBUG_MODE > 0)
		printf("");
	#endif
}


void RelativeDistace::setSpeaker_3(double x3, double y3){
	sp3.X = x3;
	sp3.X = y3;
	#if (DEBUG_MODE > 0)
		printf("");
	#endif
}

double RelativeDistace::relative_distance(double Ta, double Tb, double Tc, double depth){
	
	double dist_a = 0.0, dist_b = 0.0, dist_c = 0.0, rel_distA = 0.0, rel_distB = 0.0, rel_distC = 0.0, XAxis, YAxis;

	dist_a = W_speed * Ta;
	dist_b = W_speed * Tb;
	dist_c = W_speed * Tc;

	rel_distA = sqrt(pow(dist_a,2) + pow(depth,2));
	rel_distB = sqrt(pow(dist_b,2) + pow(depth,2));
	rel_distC = sqrt(pow(dist_c,2) + pow(depth,2));

	relative_dist =  leastSqueare(sp1.X, sp1.Y, sp2.X, sp2.Y, sp3.X, sp3.Y, rel_distA, rel_distB, rel_distC);

	XAxis =  relative_dist(0,0);
	YAxis =  relative_dist(0,1);

	return sqrt(pow(XAxis,2) + pow(YAxis,2));
}

//int x1,int y1,int x2,int y2,int x3,int y3,int a,int b,int c,int xc,int yc
mat RelativeDistace::leastSqueare(double x1,double y1,double x2,double y2,double x3,double y3,double a,double b,double c)
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