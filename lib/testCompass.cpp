#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "COMPASS.h"


//g++ testCompass.cpp compass.cpp HMC5843.cpp ADXL345.cpp -o testcompass -O2 -lwiringPi



int main(){

	//ADXL345accel;
	
	printf("Test accel..!! \n");
	
	COMPASS accel;

	accel.init();
	
	AccelG res;
	AccelRotation rot;

	
	//while(true){
	//res = accel.read_AccelG();

//	printf("X: %3.5f\t",res.x);
//	printf("Y: %3.5f\t",res.y);
//	printf("Z: %3.5f\n\n",res.z);
//	usleep(10000);
//
//	}
	
	while(true){
	rot = accel.readPitchRoll();

	printf("Pitch: %3.5f\t",rot.pitch);
	printf("Roll: %3.5f\t",rot.roll);
	printf("\n\n");
	usleep(10000);

	}


	return 0;
}