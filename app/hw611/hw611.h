/*
HW611 Drive Program for nrf52832
HW611(or maybe called as BMP280)is a model use i2c to get the pressure and temperature.
This program is for the mcu nrf52832 from Nordic. The twi(i2c) drive code is from IIKMSIK(from China) and the original MPU6050 function for stm32 is from Internet(Unknown Author)
I transplant all the code and make it can work on the nrf52832, and translate some note from Chinese into English
This program publish in GitHub and use the open source license "The 2-Clause BSD License"
*/
/*
Copyright <2024> <Huxiao>

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ¡°AS IS¡± AND ANY EXPRESS OR IMPLIED WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/*
ATTENTION!
The TWI DEVICE ID and PIN SET of I2C is in the BEGIN of hw611.c
*/
#ifndef HW611_H_
#define HW611_H_

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "nrf_drv_twi.h"


#define HW611_ADDRESS						0x76		//default device adress(not need to change)
#define HW611_RESET_VALUE					0xB6		//reset command

#define HW611_CHIPID_REG                    0xD0  /*Chip ID Register */
#define HW611_RESET_REG                     0xE0  /*Softreset Register */
#define HW611_STATUS_REG                    0xF3  /*Status Register */
#define HW611_CTRLMEAS_REG                  0xF4  /*Ctrl Measure Register */
#define HW611_CONFIG_REG                    0xF5  /*Configuration Register */
#define HW611_PRESSURE_MSB_REG              0xF7  /*Pressure MSB Register */
#define HW611_PRESSURE_LSB_REG              0xF8  /*Pressure LSB Register */
#define HW611_PRESSURE_XLSB_REG             0xF9  /*Pressure XLSB Register */
#define HW611_TEMPERATURE_MSB_REG           0xFA  /*Temperature MSB Reg */
#define HW611_TEMPERATURE_LSB_REG           0xFB  /*Temperature LSB Reg */
#define HW611_TEMPERATURE_XLSB_REG          0xFC  /*Temperature XLSB Reg */
//sign of trans
#define	HW611_MEASURING					0x01
#define	HW611_IM_UPDATE					0x08

/*calibration parameters */
#define HW611_DIG_T1_LSB_REG                0x88
#define HW611_DIG_T1_MSB_REG                0x89
#define HW611_DIG_T2_LSB_REG                0x8A
#define HW611_DIG_T2_MSB_REG                0x8B
#define HW611_DIG_T3_LSB_REG                0x8C
#define HW611_DIG_T3_MSB_REG                0x8D
#define HW611_DIG_P1_LSB_REG                0x8E
#define HW611_DIG_P1_MSB_REG                0x8F
#define HW611_DIG_P2_LSB_REG                0x90
#define HW611_DIG_P2_MSB_REG                0x91
#define HW611_DIG_P3_LSB_REG                0x92
#define HW611_DIG_P3_MSB_REG                0x93
#define HW611_DIG_P4_LSB_REG                0x94
#define HW611_DIG_P4_MSB_REG                0x95
#define HW611_DIG_P5_LSB_REG                0x96
#define HW611_DIG_P5_MSB_REG                0x97
#define HW611_DIG_P6_LSB_REG                0x98
#define HW611_DIG_P6_MSB_REG                0x99
#define HW611_DIG_P7_LSB_REG                0x9A
#define HW611_DIG_P7_MSB_REG                0x9B
#define HW611_DIG_P8_LSB_REG                0x9C
#define HW611_DIG_P8_MSB_REG                0x9D
#define HW611_DIG_P9_LSB_REG                0x9E
#define HW611_DIG_P9_MSB_REG                0x9F

/*******************************COMPENSATE**********************************/

#define	dig_T1			hw611->T1	
#define	dig_T2			hw611->T2	
#define	dig_T3			hw611->T3	

#define	dig_P1			hw611->P1
#define	dig_P2			hw611->P2
#define	dig_P3			hw611->P3
#define	dig_P4			hw611->P4
#define	dig_P5			hw611->P5
#define	dig_P6			hw611->P6
#define	dig_P7			hw611->P7
#define	dig_P8			hw611->P8
#define	dig_P9			hw611->P9
/************************************************CUT****************************************/



/*
WORKING MODE 
HW611_SLEEP_MODE||HW611_FORCED_MODE||HW611_NORMAL_MODE
*/
typedef enum {
	HW611_SLEEP_MODE = 0x0,
	HW611_FORCED_MODE = 0x1,	//or 0x2
	HW611_NORMAL_MODE = 0x3
} HW611_WORK_MODE;

//oversampling factor for pressure
typedef enum 
{
	HW611_P_MODE_SKIP = 0x0,	/*skipped*/
	HW611_P_MODE_1,			/*x1*/
	HW611_P_MODE_2,			/*x2*/
	HW611_P_MODE_3,			/*x4*/
	HW611_P_MODE_4,			/*x8*/
	HW611_P_MODE_5			    /*x16*/
} HW611_P_OVERSAMPLING;	

//oversampling factor for temperature
typedef enum {
	HW611_T_MODE_SKIP = 0x0,	/*skipped*/
	HW611_T_MODE_1,			/*x1*/
	HW611_T_MODE_2,			/*x2*/
	HW611_T_MODE_3,			/*x4*/
	HW611_T_MODE_4,			/*x8*/
	HW611_T_MODE_5			    /*x16*/
} HW611_T_OVERSAMPLING;
									
//IIR filter time constant
typedef enum {
	HW611_FILTER_OFF = 0x0,	/*filter off*/
	HW611_FILTER_MODE_1,		/*0.223*ODR*/	/*x2*/
	HW611_FILTER_MODE_2,		/*0.092*ODR*/	/*x4*/
	HW611_FILTER_MODE_3,		/*0.042*ODR*/	/*x8*/
	HW611_FILTER_MODE_4		/*0.021*ODR*/	/*x16*/
} HW611_FILTER_COEFFICIENT;

//standby time
typedef enum {
	HW611_T_SB1 = 0x0,	    /*0.5ms*/
	HW611_T_SB2,			/*62.5ms*/
	HW611_T_SB3,			/*125ms*/
	HW611_T_SB4,			/*250ms*/
	HW611_T_SB5,			/*500ms*/
	HW611_T_SB6,			/*1000ms*/
	HW611_T_SB7,			/*2000ms*/
	HW611_T_SB8,			/*4000ms*/
} HW611_T_SB;


typedef struct  
{
	/* T1~P9 compensation factor */
	uint16_t T1;
	int16_t	T2;
	int16_t	T3;
	uint16_t P1;
	int16_t	P2;
	int16_t	P3;
	int16_t	P4;
	int16_t	P5;
	int16_t	P6;
	int16_t	P7;
	int16_t	P8;
	int16_t	P9;
} HW611;


typedef struct
{
	HW611_P_OVERSAMPLING P_Osample;
	HW611_T_OVERSAMPLING T_Osample;
	HW611_WORK_MODE		WORKMODE;
} HW611_OVERSAMPLE_MODE;
	
typedef struct
{
	HW611_T_SB 				T_SB;
	HW611_FILTER_COEFFICIENT 	FILTER_COEFFICIENT;
} HW611_CONFIG;

//function can use in main()
void twi_master_init(void);
uint8_t hw611_get_id(void);
void hw611_init(void);
double hw611_get_pressure(void);
double hw611_get_temperature(void);


#endif


