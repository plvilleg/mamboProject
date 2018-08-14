#include <wiringPiI2C.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include "ITG3205.h"

int fd = 0;


bool ITG3205::begin()
{
	fd = wiringPiI2CSetup(ITG3205_ADDRESS);

	resetITG3205();
	
	if((fastRegister8(ITG3205_WHO_AM_I) != 0x68))
		return false;
	
	setDLPF_CFG(ITG3205_DLPF_CFG_5_HZ_1_khz);
	setPowerMode(ITG3205_INTERNAL_OX);
	setOffset(120, 20, 93);
	setSamplesRate(7);

	return true;
}		


Vector ITG3205::readRaw(void)
{	
	v.XRoll = readRegister16(ITG3205_GYRO_XOUT_H);
	v.YPitch = readRegister16(ITG3205_GYRO_YOUT_H);
	v.ZYaw = readRegister16(ITG3205_GYRO_ZOUT_H);
	
	return v;
}


Vector ITG3205::readNormalize(void)
{
	
	v.XRoll = (readRegister16(ITG3205_GYRO_XOUT_H) - XRoll_Offset) / 14.375;
	
	v.YPitch = (readRegister16(ITG3205_GYRO_YOUT_H) - YPitch_Offset) / 14.375;
	
	v.ZYaw = (readRegister16(ITG3205_GYRO_ZOUT_H) - ZYaw_Offset) / 14.375;
	
	return v;
}


void ITG3205::setOffset(int xo, int yo, int zo)
{
	XRoll_Offset 	= xo;
	YPitch_Offset 	= yo;
	ZYaw_Offset	= zo;
}


void ITG3205::setDLPF_CFG(itg3205_dlpf_cfg_t dlpf)
{
	uint8_t value;
	
	value = ITG3205_FS_SEL << 3;
	
	value |= dlpf;
	
	writeRegister8(ITG3205_DLPF_FS, value);
	
}


itg3205_dlpf_cfg_t ITG3205::getDLPF_CFG(void)
{
	uint8_t value;
	value = readRegister8(ITG3205_DLPF_FS);
	value &= 0b00000111;

	return (itg3205_dlpf_cfg_t)value;
}


void ITG3205::setPowerMode(itg3205_power_mode_t mode)
{
	uint8_t value;

	if(ITG3205_SLEEP == mode)
	{
		value = readRegister8(ITG3205_PWR_MGM);
		value &= 0b10111111;
		value |= (mode << 6);	
		writeRegister8(ITG3205_PWR_MGM,value);

	}else
	{
		value = readRegister8(ITG3205_PWR_MGM);
		value &= 0b10111000;
		value |= mode;	
		writeRegister8(ITG3205_PWR_MGM,value);
	}
}

itg3205_power_mode_t ITG3205::getPowerMode(void)
{
	uint8_t value;
	
	value = readRegister8(ITG3205_PWR_MGM);
	value &= 0b01000111;

	if( (value >> 3) == 0b00001000 )
		return ITG3205_SLEEP;
	else
		return (itg3205_power_mode_t)value;
}




void ITG3205::setSamplesRate(uint8_t divider)
{
	writeRegister8(ITG3205_SMPLRT_DIV, divider);
}


int ITG3205::getSamplesRate(void)
{
	uint8_t value;

	value = readRegister8(ITG3205_SMPLRT_DIV);


	if(getDLPF_CFG() == ITG3205_DLPF_CFG_256_HZ_8_khz)
		return 8000/(value + 1);
	else
		return 1000/(value + 1);
	
}

void ITG3205::resetITG3205(void)
{
	writeRegister8(ITG3205_PWR_MGM,0b10000000);
}


void ITG3205::writeRegister8(uint8_t reg, uint8_t value)
{
	if(wiringPiI2CWriteReg8(fd,reg,value) < 0)
		printf("Send ERROR.!!\n");
}

uint8_t ITG3205::fastRegister8(uint8_t reg)
{
	uint8_t value;

	value = wiringPiI2CReadReg8(fd,reg);
	
	return value;
}


uint8_t ITG3205::readRegister8(uint8_t reg)
{
	uint8_t value;

	value = wiringPiI2CReadReg8(fd,reg);
	
	return value;
}


int16_t ITG3205::readRegister16(uint8_t reg)
{
	int16_t value;
	
	value = wiringPiI2CReadReg16(fd,reg);
	
	return value;
}