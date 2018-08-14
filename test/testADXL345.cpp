#include <iostream>
#include <stdio.h>
#include <errno.h>
#include <cmath>
#include <unistd.h>
#include "ADXL345.h"

//g++ examplei2c.cpp -o examplei2c -O2 -lwiringPi

const float PI = atan(1)*4; 
const float alpha = 0.5;

using namespace std;
ADXL345 accel = ADXL345();

int main()
{	
	int XAxis_Raw, YAxis_Raw, ZAxis_Raw;
	double Gx = 0, Gy = 0, Gz = 0;
	double fXg = 0, fYg = 0, fZg = 0, pitch, roll;
	
	
	accel.powerOn() ;
	accel.setRangeSetting(2);
	accel.setActivityXYZ(1,0,0);
	accel.setActivityThreshold(75);
	accel.setTimeInactivity(10);
	accel.setTapDetectionOnXYZ(0,0,1);
	accel.setTapThreshold(50);
	accel.setTapDuration(15);
	accel.setDoubleTapLatency(80);
	accel.setDoubleTapWindow(200);

	accel.setFreeFallThreshold(7);
	accel.setFreeFallDuration(30);

	accel.InactivityINT(1);
	accel.ActivityINT(1);
	accel.FreeFallINT(1);
	accel.doubleTapINT(1);
	accel.singleTapINT(1);


	while(true){
		// RAW Accel values
		accel.readAccel(&XAxis_Raw, &YAxis_Raw, &ZAxis_Raw);
			
		// Normalized values
		Gx = ((double)XAxis_Raw) * 0.00390625;
		Gy = ((double)YAxis_Raw) * 0.00390625;
		Gz = ((double)ZAxis_Raw) * 0.00390625;
		
		//Filter values
		fXg = Gx * alpha + (fXg * (1.0 - alpha));
		fYg = Gy * alpha + (fYg * (1.0 - alpha));
		fZg = Gz * alpha + (fZg * (1.0 - alpha)); 

		
		// Roll & Pitch equations
		pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/PI;
		roll = (atan2(-fYg,fZg)*180.0)/PI;
	

		printf("Gx %f\n",XAxis_Raw*0.0039);
		printf("Gy %f\n",Gy);
		printf("Gz %f\n",Gz);
		printf("Pitch: %f\n",pitch);
		printf("Roll: %f\n",roll);
		usleep(2000);		

	}
	cout << "Exito!! " <<endl;
}