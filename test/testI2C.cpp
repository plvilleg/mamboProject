#include <iostream>
#include <errno.h>
#include "HMC5883L.h"

//g++ examplei2c.cpp -o examplei2c -O2 -lwiringPi

using namespace std;

int main()
{
	HMC5883L compass;
	

	while(!compass.begin());

	Vector raw = compass.readRaw();

	cout<<" X Axis: "<<raw.XAxis<<endl;


	cout << "Exito!! " <<endl;
}