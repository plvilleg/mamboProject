#include <iostream>
#include <errno.h>
#include <unistd.h>
#include "ITG3205.h"

//g++ itg3205test.cpp ITG3205.cpp -o itg3205test -O2 -lwiringPi

using namespace std;

int main()
{
	ITG3205 gyro;
	

	while(!gyro.begin())
	{
		cout<<"Check wiring"<<endl;
	}
	Vector raw;
	cout << "Initialize ITG3205"<<endl;
	while(true)
	{
		
		raw = gyro.readNormalize();
	
		cout<<" X Roll: "<<raw.XRoll<<endl;
		cout<<" Y Pitch: "<<raw.YPitch<<endl;
		usleep(100000);
	}

	cout << "Exito!! " <<endl;
}