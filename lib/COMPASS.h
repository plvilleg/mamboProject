#ifndef COMPASS_H
#define COMPASS_H

#include <iostream>
#include <errno.h>
//#include "HMC5843.h"
#include "ADXL345.h"

#define ALPHA 0.5

#ifndef ACCELRAW_STRUCT_H
#define ACCELRAW_STRUCT_H
struct AccelRaw
{
	int16_t x;
	int16_t y;
	int16_t z;

};
#endif

#ifndef ACCELG_STRUCT_H 
#define ACCELG_STRUCT_H 
struct AccelG
{
	double x;
	double y;
	double z;
};
#endif

#ifndef ACCELROTATION_STRUCT_H
#define ACCELROTATION_STRUCT_H
struct AccelRotation
{
	double pitch;
	double roll;
};
#endif

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
	//HMC5843 compass;
	ADXL345 accel;
	
	AccelRaw accelRAW;

	void init();
	//void Calibrate(void);
	void setup_Compass(void);
	
	void read_Accel_Mag(void);
	AccelG read_AccelG(void);

	AccelRotation readPitchRoll(void);
	
	
	
	//float get_Comp_heading(void);
	//float get_Bearing(void);
	//float get_Comp_headingDegrees(void);
	

private:

	double fXg;
	double fYg;
	double fZg;

	double xg;
	double yg;
	double zg;
	
	double _xoffset;
	double _yoffset;
	double _zoffset;
	
};


#endif