#include <wiringPiI2C.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include "HMC5883L.h"

#define HMC5883L_TO_READ (6)

int fdHMC5883L = 0;


bool HMC5883L::begin()
{
	fdHMC5883L = wiringPiI2CSetup(HMC5883L_ADDRESS);

	if((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48) | (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) | (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33))
		return false;
	
	
	setRange(HMC5883L_RANGE_1_3GA);
	setMeasurementMode(HMC5883L_CONTINUOS);
	setDataRate(HMC5883L_DATARATE_15HZ);
	setSamples(HMC5883L_SAMPLES_1);

	mgPerDigit = 0.92f;

	return true;
}		


Vector HMC5883L::readRaw(void)
{	
	int16_t x, y, z;
	int16_t temp;

	readRegister8(HMC5883L_REG_OUT_X_M, HMC5883L_TO_READ, _buff);
	temp =((_buff[1] << 8) | (_buff[0]) );
	
	
	x = (int16_t)~(((_buff[1] << 8) | _buff[0]) - 0b0000000000000001);   
	z = (int16_t)~(((_buff[3] << 8) | _buff[2]) - 0b0000000000000001);
	y = (int16_t)~(((_buff[5] << 8) | _buff[4]) - 0b0000000000000001);

	//printf("Two compliement: %X\n",temp);
	//printf("Decimal: %d\n",x);

	v.XAxis = (float) (x - xOffset);
	v.YAxis = (float) (y - yOffset);
	v.ZAxis = (float) z;
	
	return v;
}


Vector HMC5883L::readNormalize(void)
{

	int16_t x, y, z;
	readRegister8(HMC5883L_REG_OUT_X_M, HMC5883L_TO_READ, _buff);

	x = (int16_t)~(((_buff[1] << 8) | _buff[0]) - 0b0000000000000001);   
	z = (int16_t)~(((_buff[3] << 8) | _buff[2]) - 0b0000000000000001);
	y = (int16_t)~(((_buff[5] << 8) | _buff[4]) - 0b0000000000000001);

	v.XAxis = ((float)(x - xOffset) * mgPerDigit);
	v.YAxis = ((float)(y - yOffset) * mgPerDigit);
	v.ZAxis = ((float)z * mgPerDigit);

	return v;
}

void HMC5883L::setOffset(int xo, int yo)
{
	xOffset = xo;
	yOffset = yo;
}


void HMC5883L::setRange(hmc5883l_range_t range)
{
	switch(range)
	{
		case HMC5883L_RANGE_8_1GA:
			mgPerDigit = 4.35f;	
			break;

		case HMC5883L_RANGE_5_6GA:
			mgPerDigit = 3.03f;
			break;

		case HMC5883L_RANGE_4_7GA:
			mgPerDigit = 2.56f;
			break;
		
		case HMC5883L_RANGE_4GA:
			mgPerDigit = 2.27f;
			break;

		case HMC5883L_RANGE_2_5GA:
			mgPerDigit = 1.52f;
			break;

		case HMC5883L_RANGE_1_9GA:
			mgPerDigit = 1.22f;
			break;

		case HMC5883L_RANGE_1_3GA:
			mgPerDigit = 0.92f;
			break;
		
		case HMC5883L_RANGE_0_88GA:
			mgPerDigit = 0.073f;
			break;
		
		default: 
			break;
	}
	
	writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
	
}


hmc5883l_range_t HMC5883L::getRange(void)
{
	uint8_t value;
	readRegister8(HMC5883L_REG_CONFIG_B,1, _buff);
	value = (uint8_t)_buff[0];


	return (hmc5883l_range_t)((value >> 5));
}


void HMC5883L::setMeasurementMode(hmc5883l_measure_mode_t mode)
{
	uint8_t value;
	
	readRegister8(HMC5883L_REG_MODE,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11111100;
	value |= mode;	
	
	writeRegister8(HMC5883L_REG_MODE,value);
}

void HMC5883L::setOperatingMode(hmc5883l_operating_mode_t mode){
	uint8_t value;
	
	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11111100;
	value |= mode;	
	
	writeRegister8(HMC5883L_REG_CONFIG_A,value);
}

hmc5883l_operating_mode_t HMC5883L::getOperatingMode(void){
	uint8_t value;
	
	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b00000011;

	return (hmc5883l_operating_mode_t)value;
}

hmc5883l_measure_mode_t HMC5883L::getMeasurementMode(void)
{
	uint8_t value;
	
	readRegister8(HMC5883L_REG_MODE,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b00000011;

	return (hmc5883l_measure_mode_t)value;
}


void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
	uint8_t value;
	
	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11100011;
	value |= (dataRate << 2);

	writeRegister8(HMC5883L_REG_CONFIG_A, value);
}


hmc5883l_dataRate_t HMC5883L::getDataRate(void)
{
	uint8_t value;
	
	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b00011100;
	value >>= 2;

	return (hmc5883l_dataRate_t)value;
}


void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
	uint8_t value;
	
	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b10011111;
	value |= (samples << 5);
	
	writeRegister8(HMC5883L_REG_CONFIG_A, value);
}


hmc5883l_samples_t HMC5883L::getSamples(void)
{
	uint8_t value;

	readRegister8(HMC5883L_REG_CONFIG_A,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b01100000;
	value >>= 5;

	return (hmc5883l_samples_t)value;
}


void HMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
	wiringPiI2CWrite(fdHMC5883L,HMC5883L_WRITE);

	if(wiringPiI2CWriteReg8(fdHMC5883L,reg,value) < 0)
		printf("Send ERROR.!!\n");
}

uint8_t HMC5883L::fastRegister8(uint8_t reg)
{
	uint8_t value;

	wiringPiI2CWrite(fdHMC5883L,HMC5883L_READ);

	value = (uint8_t) wiringPiI2CReadReg8(fdHMC5883L,reg);
	
	return value;
}


void HMC5883L::readRegister8(uint8_t address, int num, uint8_t _buff[]) {
		
	if(num == 1){
		_buff[0] = wiringPiI2CReadReg8(fdHMC5883L,address);
	} else if(num == HMC5883L_TO_READ){
		_buff[0] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_X_M);
		_buff[1] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_X_L);
		_buff[2] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_Z_M);
		_buff[3] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_Z_L);
		_buff[4] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_Y_M);
		_buff[5] = wiringPiI2CReadReg8(fdHMC5883L,HMC5883L_REG_OUT_Y_L);
	}else
		printf("Error reading..!\n");
}





