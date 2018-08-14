#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include "COMPASS.h"


//g++ testCompass.cpp compass.cpp HMC5843.cpp ADXL345.cpp -o testcompass -O2 -lwiringPi

//g++ testCompass.cpp HMC5843.cpp -o testcompass -O2 -lwiringPi

int main(){

	COMPASS comp;
	printf("Test compass..!! \n");
	int16_t x, y, z;
	float bearing;

	
	comp.init();
	
	while(true){
	bearing = comp.get_Bearing();

	printf("Bearing: %2.2f \n", bearing);
	}
	return 0;
}