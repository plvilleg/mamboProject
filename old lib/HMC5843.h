#ifndef HMC5843_h
#define HMC5843_h

#include <wiringPiI2C.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define HMC5843_ADDRESS			(0X1E)
#define HMC5843_REG_CONFIG_A	(0X00)
#define HMC5843_REG_CONFIG_B	(0X01)
#define HMC5843_REG_MODE		(0X02)
#define HMC5843_REG_OUT_X_MSB	(0X03)
#define HMC5843_REG_OUT_X_LSB	(0X04)
#define HMC5843_REG_OUT_Y_MSB	(0X05)
#define HMC5843_REG_OUT_Y_LSB	(0X06)
#define HMC5843_REG_OUT_Z_MSB	(0X07)
#define HMC5843_REG_OUT_Z_LSB	(0X08)
#define HMC5843_REG_STATUS		(0X09)
#define HMC5843_REG_IDENT_A		(0X0A)
#define HMC5843_REG_IDENT_B		(0X0B)
#define HMC5843_REG_IDENT_C		(0X0C)

#define HMC5843_READ			(0X3D)
#define HMC5843_WRITE			(0X3C)	


//typedef enum
//{
//	HMC5843_SAMPLES_8	= 0b11,
//	HMC5843_SAMPLES_4	= 0b10,
//	HMC5843_SAMPLES_2	= 0b01,
//	HMC5843_SAMPLES_1	= 0b00
//} HMC5843_samples_t;

typedef enum
{
	HMC5843_DATARATE_50HZ		= 0b110,
	HMC5843_DATARATE_20HZ		= 0b101,
	HMC5843_DATARATE_10HZ 		= 0b100,
	HMC5843_DATARATE_5HZ		= 0b011,
	HMC5843_DATARATE_2HZ		= 0b010,
	HMC5843_DATARATE_1HZ		= 0b001,
	HMC5843_DATARATE_0_5HZ		= 0b000
} HMC5843_dataRate_t;

typedef enum
{
	HMC5843_NEGATIVE_BIAS		= 0b10,
	HMC5843_POSITIVE_BIAS		= 0b01,
	HMC5843_NORMAL_MODE			= 0b00
} HMC5843_measure_mode_t;

typedef enum
{
	HMC5843_GAIN_6_5GA	= 0b111,
	HMC5843_GAIN_4_5GA	= 0b110,
	HMC5843_GAIN_3_8GA	= 0b101,
	HMC5843_GAIN_3_2GA	= 0b100,
	HMC5843_GAIN_2_0GA	= 0b011,
	HMC5843_GAIN_1_5GA	= 0b010,
	HMC5843_GAIN_1_0GA	= 0b001,
	HMC5843_GAIN_0_7GA	= 0b000
} HMC5843_gain_t;

typedef enum
{
	HMC5843_SLEEP		= 0b11,
	HMC5843_IDLE		= 0b10,
	HMC5843_SINGLE		= 0b01,
	HMC5843_CONTINUOS	= 0b00
} HMC5843_operating_mode_t;




#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
	float XAxis;
	float YAxis;
	float ZAxis;
};
#endif

class HMC5843
{
	public:
		bool begin(void);

		Vector readRaw(void);
		Vector readNormalize(void);

		void setOffset(int xo, int yo);
		
		void setGain(HMC5843_gain_t gain);
		HMC5843_gain_t getGain(void);

		void setMeasurementMode(HMC5843_measure_mode_t mode);
		HMC5843_measure_mode_t getMeasurementMode(void);

		void setOperatingMode(HMC5843_operating_mode_t mode);
		HMC5843_operating_mode_t getOperatingMode(void);

		void setDataRate(HMC5843_dataRate_t dataRate);
		HMC5843_dataRate_t getDataRate(void);

		void setSamples(HMC5843_samples_t samples);
		HMC5843_samples_t getSamples(void);

	private:
		
		float mgPerDigit;
		Vector v;
		int xOffset, yOffset;
		uint8_t _buff[6] ;

		
		void writeRegister8(uint8_t reg, uint8_t value);
		
		uint8_t fastRegister8(uint8_t reg);		
		void readRegister8(uint8_t address, int num, uint8_t _buff[]);
};

#endif














