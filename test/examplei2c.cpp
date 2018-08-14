#include <wiringPiI2C.h>
#include <iostream>
#include <errno.h>

//g++ examplei2c.cpp -o examplei2c -O2 -lwiringPi

using namespace std;

int main()
{
	int fd, x=0, y=0 , z=0;

	fd = wiringPiI2CSetup(0x1e);

	cout << "Init result: "<<fd<<endl;

	wiringPiI2CWrite(fd,0x3c);
	wiringPiI2CWriteReg8(fd,0x00,0b01110000);
	wiringPiI2CWrite(fd,0x3c);
	wiringPiI2CWriteReg8(fd,0x02,0b00000000);

	for(int i = 0; i < 0x0000ffff; i++)
	{
		wiringPiI2CWrite(fd,0x3d);
		
		//x = wiringPiI2CRead(fd);
		x = wiringPiI2CReadReg8(fd,0x04);
		y = wiringPiI2CReadReg8(fd,0x06);
		z = wiringPiI2CReadReg8(fd,0x08);

		cout << "X result: "<<x<<endl;
		cout << "Y result: "<<y<<endl;
		cout << "Z result: "<<z<<endl;
		
		wiringPiI2CWrite(fd,0x3c);
		wiringPiI2CWriteReg8(fd,0x00,0b01110000);
		wiringPiI2CWrite(fd,0x3c);	
		wiringPiI2CWriteReg8(fd,0x02,0b00000000);

		
		
		if(x == -1)
		{
			cout <<"Error. Errno is " << errno<<endl;
		}
		
		
	}

	return 0;	
}