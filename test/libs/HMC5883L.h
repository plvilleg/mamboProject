#ifndef HMC5883L_h
#define HMC5883L_h

#include <wiringPiI2C.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>


#define HMC5883L_ADDRESS	(0X1E)
#define HMC5883L_REG_CONFIG_A	(0X00)
#define HMC5883L_REG_CONFIG_B	(0X01)
#define HMC5883L_REG_MODE	(0X02)
#define HMC5883L_REG_OUT_X_M	(0X03)
#define HMC5883L_REG_OUT_X_L	(0X04)
#define HMC5883L_REG_OUT_Z_M	(0X05)
#define HMC5883L_REG_OUT_Z_L	(0X06)
#define HMC5883L_REG_OUT_Y_M	(0X07)
#define HMC5883L_REG_OUT_Y_L	(0X08)
#define HMC5883L_REG_STATUS	(0X09)
#define HMC5883L_REG_IDENT_A	(0X0A)
#define HMC5883L_REG_IDENT_B	(0X0B)
#define HMC5883L_REG_IDENT_C	(0X0C)
#define HMC5883L_READ		(0X3D)
#define HMC5883L_WRITE		(0X3C)	


typedef enum
{
	HMC5883L_SAMPLES_8	= 0b11,
	HMC5883L_SAMPLES_4	= 0b10,
	HMC5883L_SAMPLES_2	= 0b01,
	HMC5883L_SAMPLES_1	= 0b00
} hmc5883l_samples_t;

typedef enum
{
	HMC5883L_DATARATE_75HZ		= 0b110,
	HMC5883L_DATARATE_30HZ		= 0b101,
	HMC5883L_DATARATE_15HZ 		= 0b100,
	HMC5883L_DATARATE_7_5HZ		= 0b011,
	HMC5883L_DATARATE_3HZ		= 0b010,
	HMC5883L_DATARATE_1_5HZ		= 0b001,
	HMC5883L_DATARATE_0_75HZ	= 0b000
} hmc5883l_dataRate_t;

typedef enum
{
	HMC5883L_RANGE_8_1GA	= 0b111,
	HMC5883L_RANGE_5_6GA	= 0b110,
	HMC5883L_RANGE_4_7GA	= 0b101,
	HMC5883L_RANGE_4GA	= 0b100,
	HMC5883L_RANGE_2_5GA	= 0b011,
	HMC5883L_RANGE_1_9GA	= 0b010,
	HMC5883L_RANGE_1_3GA	= 0b001,
	HMC5883L_RANGE_0_88GA	= 0b000
} hmc5883l_range_t;

typedef enum
{
	HMC5883L_IDLE		= 0b10,
	HMC5883L_SINGLE		= 0b01,
	HMC5883L_CONTINUOS	= 0b00
} hmc5883l_measure_mode_t;

typedef enum
{
	HMC5883L_NEGATIVE_BIAS		= 0b10,
	HMC5883L_POSITIVE_BIAS		= 0b01,
	HMC5883L_NORMAL_MODE		= 0b00
} hmc5883l_operating_mode_t;


#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
	float XAxis;
	float YAxis;
	float ZAxis;
};
#endif

class HMC5883L
{
	public:
		bool begin(void);

		Vector readRaw(void);
		Vector readNormalize(void);

		void setOffset(int xo, int yo);
		
		void setRange(hmc5883l_range_t range);
		hmc5883l_range_t getRange(void);

		void setMeasurementMode(hmc5883l_measure_mode_t mode);
		hmc5883l_measure_mode_t getMeasurementMode(void);

		void setOperatingMode(hmc5883l_operating_mode_t mode);
		hmc5883l_operating_mode_t getOperatingMode(void);

		void setDataRate(hmc5883l_dataRate_t dataRate);
		hmc5883l_dataRate_t getDataRate(void);

		void setSamples(hmc5883l_samples_t samples);
		hmc5883l_samples_t getSamples(void);

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














