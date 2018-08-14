
#include "HMC5843.h"

#define HMC5843_TO_READ (6)

int fdHMC5843 = 0;


bool HMC5843::begin()
{
	fdHMC5843 = wiringPiI2CSetup(HMC5843_ADDRESS);

	if((fastRegister8(HMC5843_REG_IDENT_A) != 0x48) | (fastRegister8(HMC5843_REG_IDENT_B) != 0x34) | (fastRegister8(HMC5843_REG_IDENT_C) != 0x33))
		return false;
	
	
	setGain(HMC5843_GAIN_1_5GA);
	setMeasurementMode(HMC5843_CONTINUOS);
	setDataRate(HMC5843_DATARATE_10HZ);
	//setSamples(HMC5843_SAMPLES_1);

	mgPerDigit = 0.92f;

	return true;
}		


Vector HMC5843::readRaw(void)
{	
	int16_t x, y, z;
	int16_t temp;

	readRegister8(HMC5843_REG_OUT_X_MSB, HMC5843_TO_READ, _buff);
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


Vector HMC5843::readNormalize(void)
{

	int16_t x, y, z;
	readRegister8(HMC5843_REG_OUT_X_MSB, HMC5843_TO_READ, _buff);

	x = (int16_t)~(((_buff[1] << 8) | _buff[0]) - 0b0000000000000001);   
	z = (int16_t)~(((_buff[3] << 8) | _buff[2]) - 0b0000000000000001);
	y = (int16_t)~(((_buff[5] << 8) | _buff[4]) - 0b0000000000000001);

	v.XAxis = ((float)(x - xOffset) * mgPerDigit);
	v.YAxis = ((float)(y - yOffset) * mgPerDigit);
	v.ZAxis = ((float)z * mgPerDigit);

	return v;
}

void HMC5843::setOffset(int xo, int yo)
{
	xOffset = xo;
	yOffset = yo;
}


void HMC5843::setGain(HMC5843_gain_t gain)
{
	switch(gain)
	{
		case HMC5843_Gain_6_5GA:
			mgPerDigit = 4.35f;	
			break;

		case HMC5843_GAIN_4_5GA:
			mgPerDigit = 3.03f;
			break;

		case HMC5843_Gain_3_8GA:
			mgPerDigit = 2.56f;
			break;
		
		case HMC5843_GAIN_2_2GA:
			mgPerDigit = 2.27f;
			break;

		case HMC5843_GAIN_2_0GA:
			mgPerDigit = 1.52f;
			break;

		case HMC5843_GAIN_1_5GA:
			mgPerDigit = 1.22f;
			break;

		case HMC5843_GAIN_1_0GA:
			mgPerDigit = 0.92f;
			break;
		
		case HMC5843_GAIN_0_7GA:
			mgPerDigit = 0.073f;
			break;
		
		default: 
			break;
	}
	
	writeRegister8(HMC5843_REG_CONFIG_B, gain << 5);
	
}


HMC5843_gain_t HMC5843::getGain(void)
{
	uint8_t value;
	readRegister8(HMC5843_REG_CONFIG_B,1, _buff);
	value = (uint8_t)_buff[0];


	return (HMC5843_gain_t)((value >> 5));
}


void HMC5843::setMeasurementMode(HMC5843_measure_mode_t mode)
{
	uint8_t value;
	
	readRegister8(HMC5843_REG_MODE,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11111100;
	value |= mode;	
	
	writeRegister8(HMC5843_REG_MODE,value);
}

void HMC5843::setOperatingMode(HMC5843_operating_mode_t mode){
	uint8_t value;
	
	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11111100;
	value |= mode;	
	
	writeRegister8(HMC5843_REG_CONFIG_A,value);
}

HMC5843_operating_mode_t HMC5843::getOperatingMode(void){
	uint8_t value;
	
	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b00000011;

	return (HMC5843_operating_mode_t)value;
}

HMC5843_measure_mode_t HMC5843::getMeasurementMode(void)
{
	uint8_t value;
	
	readRegister8(HMC5843_REG_MODE,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b00000011;

	return (HMC5843_measure_mode_t)value;
}


void HMC5843::setDataRate(HMC5843_dataRate_t dataRate)
{
	uint8_t value;
	
	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b11100011;
	value |= (dataRate << 2);

	writeRegister8(HMC5843_REG_CONFIG_A, value);
}


HMC5843_dataRate_t HMC5843::getDataRate(void)
{
	uint8_t value;
	
	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b00011100;
	value >>= 2;

	return (HMC5843_dataRate_t)value;
}


void HMC5843::setSamples(HMC5843_samples_t samples)
{
	uint8_t value;
	
	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);

	value = (uint8_t)_buff[0];
	value &= 0b10011111;
	value |= (samples << 5);
	
	writeRegister8(HMC5843_REG_CONFIG_A, value);
}


HMC5843_samples_t HMC5843::getSamples(void)
{
	uint8_t value;

	readRegister8(HMC5843_REG_CONFIG_A,1,_buff);
	
	value = (uint8_t)_buff[0];
	value &= 0b01100000;
	value >>= 5;

	return (HMC5843_samples_t)value;
}


void HMC5843::writeRegister8(uint8_t reg, uint8_t value)
{
	wiringPiI2CWrite(fdHMC5843,HMC5843_WRITE);

	if(wiringPiI2CWriteReg8(fdHMC5843,reg,value) < 0)
		printf("Send ERROR.!!\n");
}

uint8_t HMC5843::fastRegister8(uint8_t reg)
{
	uint8_t value;

	wiringPiI2CWrite(fdHMC5843,HMC5843_READ);

	value = (uint8_t) wiringPiI2CReadReg8(fdHMC5843,reg);
	
	return value;
}


void HMC5843::readRegister8(uint8_t address, int num, uint8_t _buff[]) {
		
	if(num == 1){
		_buff[0] = wiringPiI2CReadReg8(fdHMC5843,address);
	} else if(num == HMC5843_TO_READ){
		_buff[0] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_X_MSB);
		_buff[1] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_X_LSB);
		_buff[2] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_Y_MSB);
		_buff[3] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_Y_LSB);
		_buff[4] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_Z_MSB);
		_buff[5] = wiringPiI2CReadReg8(fdHMC5843,HMC5843_REG_OUT_Z_LSB);
	}else
		printf("Error reading..!\n");
}





