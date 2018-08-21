#ifndef COMPASS_H
#define COMPASS_H

#include <iostream>
#include <errno.h>
#include "HMC5843.h"
#include "ADXL345.h"
#include <unistd.h>

#define ALPHA 0.5

#define DEBUG_MODE 0

#ifndef RAW_STRUCT_H
#define RAW_STRUCT_H
struct Raw
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
	int16_t X;
	int16_t Y;
	int16_t Z;
};
#endif






class COMPASS
{
public:	
	HMC5843 mag;
	ADXL345 accel;
	
	Raw accelRAW;
	Raw magRAW;

	//Vector magAxis;

	void init();
	bool Calibrate(uint8_t gain, uint8_t n_samples);
	void getOffset();
	void setup_Compass(void);
	
	void read_Accel_Mag(void);
	AccelG read_AccelG(void);

	AccelRotation readPitchRoll(void);
	
	void getcalibratevalues(float *x, float *y, float *z);
	void getMagAxes(double *Bfx, double *Bfy, double *Bfz);
	
	float get_Comp_heading(void);
	

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

	float x_scale, y_scale, z_scale, max_x, max_y, max_z, magOffsetx, magOffsety, magOffsetz; 
	
};


#endif