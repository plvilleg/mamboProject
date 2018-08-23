#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
 
#include "COMPASS.h"


//g++ testCompass.cpp compass.cpp HMC5843.cpp ADXL345.cpp -o testcompass -O2 -lwiringPi

const float PI = (atan(1)*4);

int main(){

	//ADXL345accel;
	
	printf("Test accel..!! \n");

	float x,y,z, bearing;	
	COMPASS accel;
	COMPASS mag;

	
	mag.init();
	//accel.init();
	
	AccelG res;
	AccelRotation rot;

	
	//while(true){
	//res = accel.read_AccelG();

//	printf("X: %3.5f\t",res.x);
//	printf("Y: %3.5f\t",res.y);
//	printf("Z: %3.5f\n\n",res.z);
//	usleep(100000);
//
//	}
	


	while(true){

	rot = mag.readPitchRoll();

	printf("Pitch: %3.5f\t",rot.pitch* (180/PI));
	printf("Roll: %3.5f\t",rot.roll* (180/PI));
	printf("\n\n");


//	mag.getcalibratevalues(&x, &y, &z);
//	printf("Mag compenents: \n");
//	printf("X: %3.5f\t",x);
//	printf("Y: %3.5f\t",y);
//	printf("Z: %3.5f\n",z);
//
//
//	res = accel.read_AccelG();
//	printf("Accel components: \n");
//	printf("X: %3.5f\t",res.x);
//	printf("Y: %3.5f\t",res.y);
//	printf("Z: %3.5f\n\n",res.z);

	bearing = mag.get_Comp_heading();
	printf("Azimuth: \t%3.2f\n",bearing);

	
	usleep(100000);

	}


	return 0;
}