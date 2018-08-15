#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include "COMPASS.h"
#define _USE_MATH_DEFINES
//g++ compass.cpp  HMC5843.cpp ADXL345.cpp -o compass -O2 -lwiringPi

using namespace std;
const float PI = (atan(1)*4);



void COMPASS::init(){
	
	//Mag variales
	x_scale = 1.0F;
	y_scale = 1.0F;
	z_scale = 1.0F;
	
	// Accel variable
	fXg = 0; fYg = 0; fZg = 0;
	xg = 0 ; yg = 0; zg = 0;
	_xoffset = -0.023;
	_yoffset = 0;
	_zoffset = 0.03577027;


	
	setup_Compass();	
	
	
}


void COMPASS::setup_Compass(){

	cout<<"Starting Compass..!"<<endl;

	// config HMC5843
	HMC5843();
       	mag.initialize();
        
	while(!mag.testConnection()){
		printf("Check mag wiring!\n");
	}
	printf("HMC5843 detected.!\n");

        // CONFIG_A register
        mag.setDataRate(HMC5843_RATE_10);
        mag.setMeasurementBias(HMC5843_BIAS_NORMAL);

        // CONFIG_B register
        mag.setGain(HMC5843_GAIN_1300);

        // MODE register
        mag.setMode(HMC5843_MODE_CONTINUOUS);

	printf("HMC5843 succesful config.!\n");



	// Config ADXL345
	ADXL345();
	accel.initialize();
                
	while(!accel.testConnection())
	{
		printf("Check accel wiring!\n");
	}
	printf("ADXL345 detected.!\n");
	

	// POWER_CTL register
        accel.setLinkEnabled(false);
        accel.setAutoSleepEnabled(false);
        accel.setMeasureEnabled(true);
        accel.setSleepEnabled(false);
        accel.setWakeupFrequency(0);	

	// DATA_FORMAT register
        accel.setSelfTestEnabled(0);
       	accel.setSPIMode(0);
        accel.setInterruptMode(0);
        accel.setFullResolution(1);
       	accel.setDataJustification(0);
        accel.setRange(3);


	// THRESH_TAP register
        accel.setTapThreshold(1);

   	// DUR register
        accel.setTapDuration(0);
        
        // LATENT register
      	accel.setDoubleTapLatency(0);
        
        // WINDOW register
        accel.setDoubleTapWindow(0);
        
        // THRESH_ACT register
        accel.setActivityThreshold(1);
        
        // THRESH_INACT register
        accel.setInactivityThreshold(15);

        // TIME_INACT register
        accel.setInactivityTime(43);
        
        // ACT_INACT_CTL register
	accel.setActivityAC(false);
        accel.setActivityXEnabled(false);
       	accel.setActivityYEnabled(false);
        accel.setActivityZEnabled(false);
        accel.setInactivityAC(false);
       	accel.setInactivityXEnabled(false);
        accel.setInactivityYEnabled(false);
        accel.setInactivityZEnabled(false);
        
        // THRESH_FF register
        accel.setFreefallThreshold(0x09);
        
        // TIME_FF register
        accel.setFreefallTime(0x46);
        
        // TAP_AXES register
        accel.setTapAxisSuppress(false);
        accel.setTapAxisXEnabled(false);
        accel.setTapAxisYEnabled(false);
        accel.setTapAxisZEnabled(false);


	// BW_RATE register
    	accel.setLowPowerEnabled(false);
      	accel.setRate(10);

	// FIFO_CTL register
        accel.setFIFOMode(2);
        accel.setFIFOTriggerInterruptPin(0);
      	accel.setFIFOSamples(0xF);

	// INT_MAP register
        accel.setIntDataReadyPin(0);
        accel.setIntSingleTapPin(0);
        accel.setIntDoubleTapPin(0);
       	accel.setIntActivityPin(0);
        accel.setIntInactivityPin(0);
        accel.setIntFreefallPin(0);
        accel.setIntWatermarkPin(0);
        accel.setIntOverrunPin(0);

	// OFS* registers
        accel.setOffsetX(-1);
        accel.setOffsetY(0);
        accel.setOffsetZ(8);

	printf("ADXL345 config succesful.!\n");
}



void COMPASS::read_Accel_Mag(){
	accel.getAcceleration(&accelRAW.x, &accelRAW.y, &accelRAW.z); //Read accel.
	mag.getHeading(&magRAW.x, &magRAW.y, &magRAW.z); // Read mag. 

}


AccelG COMPASS::read_AccelG(void){
	read_Accel_Mag();
	
	fXg = ((int)accelRAW.x) * 0.00390625 + _xoffset;
	fYg = ((int)accelRAW.y) * 0.00390625 + _yoffset;
	fZg = ((int)accelRAW.z) * 0.00390625 + _zoffset;

	AccelG res;

	res.x = fXg * ALPHA + (xg * (1.0 - ALPHA));
	xg = res.x;
	
	res.y = fYg * ALPHA + (yg * (1.0 - ALPHA));
	yg = res.y;

	res.z = fZg * ALPHA + (zg * (1.0 - ALPHA));
	zg = res.z;

	return res;
}

AccelRotation COMPASS::readPitchRoll(void){
	AccelG accel;
	accel = read_AccelG();

	AccelRotation rot;
	
	rot.pitch = (atan2(accel.x,sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0) / PI;
	rot.roll = (atan2(accel.y,(sqrt(accel.x * accel.x + accel.z * accel.z))) * 180.0) / PI;


	return rot;
}

void COMPASS::getcalibratevalues(float *x, float *y, float *z){
	read_Accel_Mag();

	*x = ((float)magRAW.x) / x_scale;
	*y = ((float)magRAW.y) / y_scale;
	*z = ((float)magRAW.z) / z_scale;

}