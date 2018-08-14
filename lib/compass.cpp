#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include "COMPASS.h"
#define _USE_MATH_DEFINES
//g++ compass.cpp  HMC5843.cpp ADXL345.cpp -o compass -O2 -lwiringPi

using namespace std;
const float PI = (atan(1)*4);
const float alpha = 0.5;


void COMPASS::init(){
	
	fXg = 0; fYg = 0; fZg = 0;
	
	setup_Compass();	
	//Calibrate();
	read_Accel_Mag();
	normalize_Accel();
	filter_Accel();
	Comp_Axis();
}


void COMPASS::Calibrate()
{
	int minX = 0;
	int maxX = 0;
	int minY = 0;
	int maxY = 0;
	int offX = 0;
	int offY = 0;
	int i = 0;

	Vector mag;

	printf("Calibrating..!\n");

	for(i = 0; i < 1000 ; i++)
	{
		compass.getHeading(&mag.XAxis, &mag.YAxis, &mag.ZAxis); // Read mag. 
	
		if(mag.XAxis < minX) minX = mag.XAxis;
		if(mag.XAxis > maxX) maxX = mag.XAxis;
		if(mag.YAxis < minY) minY = mag.YAxis;
		if(mag.YAxis > maxY) maxY = mag.YAxis;
	}
	
	offX = (maxX + minX)/2;
	offY = (maxY + minY)/2;

	printf("offX %d\n",offX);
	printf("offY %d\n",offY);
	//compass.setOffset(offX,offY);
	
}

void COMPASS::setup_Compass(){

	cout<<"Starting Compass..!"<<endl;

	HMC5843();
	compass.initialize();
	
	while(!compass.testConnection())
	{
		printf("Check compass wiring!\n");
	}

	compass.initialize();	

	compass.setDataRate(HMC5843_RATE_10);
	compass.setMeasurementBias(HMC5843_BIAS_NORMAL);	

	//compass.setOffset(0,0);

	ADXL345();
	accel.initialize();
                
	while(!accel.testConnection())
	{
		printf("Check accel wiring!\n");
	}

	

	accel.setActivityXEnabled(true);
	accel.setActivityYEnabled(true);
	accel.setActivityThreshold(75);

	accel.setInactivityXEnabled(true);
	accel.setInactivityYEnabled(true);
	accel.setInactivityThreshold(75);
	accel.setInactivityTime(10);
	
	accel.setRange(2);

	accel.setTapAxisZEnabled(true);
	accel.setTapThreshold(50);
	accel.setTapDuration(15);
	accel.setDoubleTapLatency(80);
	accel.setDoubleTapWindow(200);

	accel.setFreefallThreshold(7);
	accel.setFreefallTime(30);

	accel.setIntActivityEnabled(false);
	accel.setIntInactivityEnabled(false);
	accel.setIntFreefallEnabled(false);
	accel.setIntDoubleTapEnabled(false);
	accel.setIntSingleTapEnabled(false);
	
}

void COMPASS::read_Accel_Mag(){
	accel.getAcceleration(&XAxis_Raw, &YAxis_Raw, &ZAxis_Raw); //Read accel.
	compass.getHeading(&raw.XAxis, &raw.YAxis, &raw.ZAxis); // Read mag. 

}


void COMPASS::normalize_Accel(void){
	read_Accel_Mag();
	Gx = ((double)XAxis_Raw) * 0.00390625;
	Gy = ((double)YAxis_Raw) * 0.00390625;
	Gz = ((double)ZAxis_Raw) * 0.00390625;

}

void COMPASS::filter_Accel(void){
	normalize_Accel();
	fXg = Gx * alpha + (fXg * (1.0 - alpha));
	fYg = Gy * alpha + (fYg * (1.0 - alpha));
	fZg = Gz * alpha + (fZg * (1.0 - alpha));
}


double COMPASS::get_pitch(void){
	filter_Accel();
	return (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/PI;
}


double COMPASS::get_roll(void){
	filter_Accel();
	return (atan2(-fYg,fZg)*180.0)/PI;
}


void COMPASS::Comp_Axis(void){

	pitch = get_pitch();
	roll =	get_roll();
	X_comp = (norm.XAxis * cos(pitch)) + (norm.YAxis * sin(roll) * sin(pitch)) + (norm.ZAxis * cos(roll) * sin(pitch));
	Y_comp = (norm.YAxis * cos(roll)) - (norm.ZAxis * sin(roll));
		
}


float COMPASS::get_Comp_heading(){
	float declinationAngle, heading;
	
	declinationAngle = -2.5 * (PI/180);
	heading = atan2(Y_comp,X_comp);
	heading -= declinationAngle;

	if(heading < 0)
		heading += (2*PI);

	if(heading > (2*PI))
		heading -= 2*PI;		


	return heading;
}
		
float COMPASS::get_Comp_headingDegrees(void){
	return get_Comp_heading() * (180/PI);
}

float COMPASS::get_Bearing(){

	read_Accel_Mag();
	normalize_Accel();
	filter_Accel();
	Comp_Axis();

	if(X_comp < 0)
	{
		return 180 - get_Comp_headingDegrees();
	}else 
	if(X_comp > 0 && Y_comp < 0)
	{
		return - get_Comp_headingDegrees();
	} else
	if(X_comp > 0 && Y_comp > 0)
	{
		return 360 - get_Comp_headingDegrees();
	} else 
	if(X_comp = 0 && Y_comp < 0)
	{
		return 90.0;
	} else 
	if(X_comp = 0 && Y_comp > 0)
	{
		return 270.0;
	}
}