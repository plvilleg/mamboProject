#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "COMPASS.h"

//g++ testCompass.cpp compass.cpp hmc5883l.cpp ADXL345.cpp -o compass -O2 -lwiringPi

int main(){

	COMPASS comp;
	float bearing;
	comp.init();

	while(true){
	bearing = comp.get_Bearing();

	printf("BEARING: %2.2f\n", bearing);
	usleep(30000);
	}


	return 0;
}
