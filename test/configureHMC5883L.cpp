#include <iostream>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include "HMC5883L.h"

HMC5883L mag;


int main(){

	Vector raw;

	//cout<<"Initialize Compass..!"<<endl;

	while(!mag.begin())
	{
		printf("Check wiring!\n");
	}
	
	mag.setRange(HMC5883L_RANGE_1_3GA);

	

	mag.setOperatingMode(HMC5883L_POSITIVE_BIAS);

	mag.setMeasurementMode(HMC5883L_SINGLE);

	mag.setDataRate(HMC5883L_DATARATE_30HZ);

	mag.setSamples(HMC5883L_SAMPLES_8);

	mag.setOffset(0,0);

	raw = mag.readNormalize();

	printf("XAxis: %2.2f\n",raw.XAxis);
	printf("YAxis: %2.2f\n",raw.YAxis);


}
