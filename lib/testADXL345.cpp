#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "ADXL345.h"


//g++ testCompass.cpp compass.cpp HMC5843.cpp ADXL345.cpp -o testcompass -O2 -lwiringPi

//g++ testADXL345.cpp ADXL345.cpp -o testADXL345 -O2 -lwiringPi

int main(){

	ADXL345 accel;
	printf("Test accel..!! \n");
	int16_t x, y, z;
		
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

	
	while(true){
	accel.getAcceleration(&x, &y, &z);

	printf("X: %0.5d\t",(int)x);
	printf("Y: %0.5d\t",(int)y);
	printf("Z: %0.5d\n",(int)z);
	usleep(10000);

	}
	return 0;
}