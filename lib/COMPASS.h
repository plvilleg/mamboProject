#ifndef COMPASS_H
#define COMPASS_H

#include <iostream>
#include <errno.h>
#include "HMC5843.h"
#include "ADXL345.h"


#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
	int16_t XAxis;
	int16_t YAxis;
	int16_t ZAxis;
};
#endif

class COMPASS
{
public:	
	HMC5843 compass;
	ADXL345 accel;
	
	Vector norm, raw;
	
	int16_t XAxis_Raw, YAxis_Raw, ZAxis_Raw;
	double Gx, Gy, Gz;
	double fXg, fYg, fZg, X_comp, Y_comp,roll,pitch;

	void init();
	void Calibrate(void);
	void setup_Compass(void);
	
	void read_Accel_Mag(void);
	void normalize_Accel(void);
	void filter_Accel(void);
	void Comp_Axis(void);
	
	float get_Comp_heading(void);
	float get_Bearing(void);
	float get_Comp_headingDegrees(void);

	double get_pitch(void);
	double get_roll(void);
	
	


};


#endif