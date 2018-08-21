#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include "COMPASS.h"
#define _USE_MATH_DEFINES
//g++ compass.cpp  HMC5843.cpp ADXL345.cpp -o compass -O2 -lwiringPi

using namespace std;
const float PI = (atan(1)*4);

const int16_t counts_per_milligaus[8] = {
	1620,
	1300,
	970,
	780,
	530,
	460,
	390,
	280
};

void COMPASS::init(){
	
	//Mag variales
	x_scale = 1.0F;
	y_scale = 1.0F;
	z_scale = 1.0F;
	magOffsetx = 0;
	magOffsety = 0;
	magOffsetz = 0;
	
	// Accel variable
	fXg = 0; fYg = 0; fZg = 0;
	xg = 0 ; yg = 0; zg = 0;
	_xoffset = -0.023;
	_yoffset = 0;
	_zoffset = 0.03577027;


	
	setup_Compass();
	
	printf("Calibrating compass.\n");

	while(!Calibrate(HMC5843_GAIN_1300, 250)){
		printf("Calibration failure!\n");
		usleep(1000000);
	}
	printf("Calibration succesfull!\n");

	getOffset();

	printf("Set offsets!\n");
	
	
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
        accel.setOffsetX(-1);//-1
        accel.setOffsetY(0);
        accel.setOffsetZ(8);//8

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
	
	rot.roll = atan2(accel.y,accel.z);

	rot.pitch = atan(((-accel.x)/(accel.y*sin(rot.roll) + accel.z*cos(rot.roll)))); 

	return rot;
}

void COMPASS::getcalibratevalues(float *x, float *y, float *z){
	read_Accel_Mag();

	*x = ((float)magRAW.x) / x_scale;
	*y = ((float)magRAW.y) / y_scale;
	*z = ((float)magRAW.z) / z_scale;

}

bool COMPASS::Calibrate(uint8_t gain, uint8_t n_samples){
	int16_t xyz[3] = {0,0,0};;
	int32_t xyz_total[3] = {0,0,0};
	bool bret = true;
	int32_t low_limit, high_limit;

	if((8 > gain) && (0 < n_samples)){
		mag.setMeasurementBias(HMC5843_BIAS_POSITIVE);
		mag.setGain(gain);
		mag.setMode(HMC5843_MODE_SINGLE);
		
		mag.getHeading(&xyz[0], &xyz[1], &xyz[2]); // Read mag. 
		
		for(uint8_t i = 0; i < n_samples; i++){
				
			printf("Measure positive bias.\n");
			mag.setMeasurementBias(HMC5843_BIAS_POSITIVE);
			mag.setMode(HMC5843_MODE_SINGLE);
			mag.getHeading(&xyz[0], &xyz[1], &xyz[2]);
			
			xyz_total[0] += xyz[0];
			xyz_total[1] += xyz[1];
			xyz_total[2] += xyz[2];

			printf("X: %d \tY: %d \tZ: %d\n",xyz[0],xyz[1],xyz[2]);

			if(-(1 << 12) >= min(xyz[0], min(xyz[1], xyz[2])))
			{
				printf("HMC5843 Self test saturated. Increase range.\n");
				bret = false;
				break;
			}
		}

		#if(DEBUG_MODE > 0)
			printf("X_tot: %d \tY_tot: %d \tZ_tot: %d\n",xyz_total[0],xyz_total[1],xyz_total[2]);
		#endif

		mag.setMeasurementBias(HMC5843_BIAS_NEGATIVE);
		mag.setGain(gain);
		mag.setMode(HMC5843_MODE_SINGLE);
		
		for(uint8_t i = 0; i < n_samples; i++){
			printf("Measure negative bias.\n");
			mag.setMode(HMC5843_MODE_SINGLE);
			mag.getHeading(&xyz[0], &xyz[1], &xyz[2]);
			
			xyz_total[0] -= xyz[0];
			xyz_total[1] -= xyz[1];
			xyz_total[2] -= xyz[2];

			printf("X: %d \tY: %d \tZ: %d\n",xyz[0],xyz[1],xyz[2]);

			if(-(1 << 12) >= min(xyz[0], min(xyz[1], xyz[2])))
			{
				printf("HMC5843 Self test saturated. Increase range.\n");
				bret = false;
				break;
			}
		}

		#if (DEBUG_MODE > 0)
			printf("X_tot: %d \tY_tot: %d \tZ_tot: %d\n",xyz_total[0],xyz_total[1],xyz_total[2]);
			printf("counts_per_milligaus[gain] %d \n", counts_per_milligaus[gain] );

		#endif

		low_limit = SELF_TEST_LOW_LIMIT * counts_per_milligaus[gain] * 2 * n_samples;
		high_limit = SELF_TEST_HIGH_LIMIT * counts_per_milligaus[gain] * 2 * n_samples;
		
		#if (DEBUG_MODE > 0)
			printf("low_limit %d\n",low_limit);
			printf("high_limit %d\n", high_limit);
		#endif
		

		if((true == bret) &&
			(low_limit <= xyz_total[0]) && (high_limit >= xyz_total[0]) &&
			(low_limit <= xyz_total[1]) && (high_limit >= xyz_total[1]) &&
			(low_limit <= xyz_total[2]) && (high_limit >= xyz_total[2])  ){

			x_scale = (counts_per_milligaus[gain] * (HMC5843_X_SELF_TEST_GAUSS * 2)) / (xyz_total[0] / n_samples);
			y_scale = (counts_per_milligaus[gain] * (HMC5843_Y_SELF_TEST_GAUSS * 2)) / (xyz_total[1] / n_samples);
			z_scale = (counts_per_milligaus[gain] * (HMC5843_Z_SELF_TEST_GAUSS * 2)) / (xyz_total[2] / n_samples);
		}else{
			printf("HMC5843 Self test out of range. \n");
			bret = false;
		}
		mag.setMeasurementBias(HMC5843_BIAS_NORMAL);
      		mag.setGain(HMC5843_GAIN_1300);
		mag.setMode(HMC5843_MODE_CONTINUOUS);

	} else {
		printf("HMC5843 Bad parameters.\n");
		bret = false;
	}

		mag.setMeasurementBias(HMC5843_BIAS_NORMAL);
      		mag.setGain(HMC5843_GAIN_1300);
		mag.setMode(HMC5843_MODE_CONTINUOUS);

	return (bret);
}

void COMPASS::getOffset()
{
	float x, y ,z;
	float minX = 0;
	float maxX = 0;
	float minY = 0;
	float maxY = 0;
	float minZ = 0;
	float maxZ = 0;
	int i = 0;

	printf("Obtaining Mag offsets.!\n");

	for(i = 0; i < 1000 ; i++)
	{
		getcalibratevalues(&x, &y, &z); // Read mag. 
	
		if(x < minX) minX = x;
		if(x > maxX) maxX = x;
		if(y < minY) minY = y;
		if(y > maxY) maxY = y;
		if(z < minZ) minZ = z;
		if(z > maxZ) maxZ = z;
	}
	
	magOffsetx = 0;//(maxX + minX)/2;
	magOffsety = 0;//(maxY + minY)/2;
	magOffsetz = 0;//(maxZ + minZ)/2;

	printf("magOffX %2.3f\t", magOffsetx);
	printf("magOffY %2.3f\t", magOffsety);
	printf("magOffZ %2.3f\n", magOffsetz);
		
}

void COMPASS::getMagAxes(double *Bfx, double *Bfy, double *Bfz){

	float Bpx, Bpy, Bpz;

	getcalibratevalues(&Bpx, &Bpy, &Bpz);
	AccelRotation rot = readPitchRoll();

	*Bfx = ((Bpx - magOffsetx)*cos(rot.pitch) + (Bpy- magOffsety)*sin(rot.pitch)*sin(rot.roll) - (Bpz - magOffsetz)*sin(rot.pitch)*cos(rot.roll));	

	*Bfy = (Bpy - magOffsety)*cos(rot.roll) + (Bpz - magOffsetz)*sin(rot.roll);
	
	*Bfz = (Bpx - magOffsetx)*sin(rot.pitch) + (Bpy - magOffsety)*cos(rot.pitch)*sin(rot.roll) + (Bpz - magOffsetz)*cos(rot.pitch)*cos(rot.roll);
	

}

float COMPASS::get_Comp_heading(){
	float declinationAngle, heading;
	double x, y, z;

	getMagAxes(&x, &y, &z);
	
	declinationAngle = -2.23333 * (PI/180);
	
	if(x < 0)
	{
		return 180 - ((atan2(y,x) - declinationAngle) * (180/PI));
	}else 
	if(x > 0 && y < 0)
	{
		return -((atan2(y,x) - declinationAngle) * (180/PI));
	} else
	if(x > 0 && y > 0)
	{
		return 360 - ((atan2(y,x) - declinationAngle) * (180/PI));
	} else 
	if(x = 0 && y < 0)
	{
		return 90.0;
	} else 
	if(x = 0 && y > 0)
	{
		return 270.0;
	}

//	if(heading < 0)
//		heading += (2*PI);
//
//	if(heading > (2*PI))
//		heading -= 2*PI;		
//
	return heading;
}
		
