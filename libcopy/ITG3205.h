#ifndef ITG3205_h
#define ITG3205_h

#include <wiringPiI2C.h>
#include <iostream>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>


#define ITG3205_ADDRESS		(0X68)
#define ITG3205_WHO_AM_I	(0X00)
#define ITG3205_SMPLRT_DIV	(0X15)
#define ITG3205_DLPF_FS		(0X16)
#define ITG3205_INT_CFG		(0X17)
#define ITG3205_INT_STATUS	(0X1A)
#define ITG3205_TEMP_OUT_H	(0X1B)
#define ITG3205_TEMP_OUT_L	(0X1C)
#define ITG3205_GYRO_XOUT_H	(0X1D)
#define ITG3205_GYRO_XOUT_L	(0X1E)
#define ITG3205_GYRO_YOUT_H	(0X1F)
#define ITG3205_GYRO_YOUT_L	(0X20)
#define ITG3205_GYRO_ZOUT_H	(0X21)
#define ITG3205_GYRO_ZOUT_L	(0X22)
#define ITG3205_PWR_MGM		(0X3E)
#define ITG3205_FS_SEL		(0b11)
#define ITG3205_READ		(0XFF)	
#define ITG3205_WRITE		(0XFF)	


typedef enum
{
	ITG3205_DLPF_CFG_5_HZ_1_khz	= 0b110,
	ITG3205_DLPF_CFG_10_HZ_1_khz	= 0b101,
	ITG3205_DLPF_CFG_20_HZ_1_khz	= 0b100,
	ITG3205_DLPF_CFG_42_HZ_1_khz	= 0b011,
	ITG3205_DLPF_CFG_98_HZ_1_khz	= 0b010,
	ITG3205_DLPF_CFG_188_HZ_1_khz	= 0b001,
	ITG3205_DLPF_CFG_256_HZ_8_khz	= 0b000
} itg3205_dlpf_cfg_t;

typedef enum
{
	ITG3205_SLEEP		= 0b1,
	ITG3205_INTERNAL_OX	= 0b000,
	ITG3205_PLL_X_REF	= 0b001,
	ITG3205_PLL_Y_REF	= 0b010,
	ITG3205_PLL_Z_REF	= 0b011,
	ITG3205_PLL_32_768_KHZ	= 0b100,
	ITG3205_PLL_19_2_MHZ	= 0b101
} itg3205_power_mode_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
	int XRoll;
	int YPitch;
	int ZYaw;
};
#endif

class ITG3205
{
	public:
		bool begin(void);

		Vector readRaw(void);
		Vector readNormalize(void);

		void setOffset(int xo, int yo, int zo);
		
		void setDLPF_CFG(itg3205_dlpf_cfg_t dlpf);
		itg3205_dlpf_cfg_t getDLPF_CFG(void);

		void setPowerMode(itg3205_power_mode_t mode);
		itg3205_power_mode_t getPowerMode(void);

		void setSamplesRate(uint8_t divider);
		int getSamplesRate(void);

		void resetITG3205(void);

	private:
		
		Vector v;
		int XRoll_Offset, YPitch_Offset, ZYaw_Offset;

		
		void writeRegister8(uint8_t reg, uint8_t value);
		uint8_t readRegister8(uint8_t reg);
		uint8_t fastRegister8(uint8_t reg);		
		int16_t readRegister16(uint8_t reg);
};

#endif