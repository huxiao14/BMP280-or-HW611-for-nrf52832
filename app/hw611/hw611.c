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

#include "hw611.h"

/*
ATTENTION!
You SHOULD check this part, make sure it can be used in your MCU and circuit
when you change TWI_INSTANCE_ID, you should change the sdk_config.h as same time 
*/
//TWI drive device id£¬0:TWI0  1:TWI1
#define TWI_INSTANCE_ID     1
#define TWI_SCL_H           22         //HW611 I2C SCL
#define TWI_SDA_H           23         //HW611 I2C SDA


HW611 hw611_inst; 
HW611* hw611 = &hw611_inst;		//save compensation factor in rom
int32_t t_fine;			//compensate t


static volatile bool h_xfer_done = false;//The sign for the twi action done
static const nrf_drv_twi_t h_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID); //init the twi sign

/*
twi handler function
no need to use in main.c
*/
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    //check the type of twi event
	  switch (p_event->type)
    {
        //done event
			  case NRF_DRV_TWI_EVT_DONE:
            h_xfer_done = true;//set the action done sign TRUE
            break;
        default:
            break;
    }
}

/*
init twi device
should use in main() or BSP_INIT(when use rtos)
*/
void twi_master_init(void)
{
    ret_code_t err_code;
    //twi config struct
    const nrf_drv_twi_config_t twi_config = {
       .scl                = TWI_SCL_H,  //set TWI SCL
       .sda                = TWI_SDA_H,  //set TWI SDA
       .frequency          = NRF_DRV_TWI_FREQ_100K, //TWI speed
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH, //TWI priority
       .clear_bus_init     = false//not send 9 clock when init
    };
    //init
    err_code = nrf_drv_twi_init(&h_twi, &twi_config, twi_handler, NULL);
	  //check error code
    APP_ERROR_CHECK(err_code);
    //enable twi
    nrf_drv_twi_enable(&h_twi);
}

/*
write a byte to register function
send "value" to the register
return true when writing success
*/ 
bool hw611_write_byte(uint8_t register_address, uint8_t value) //register write == byte write in this case
{
	  ret_code_t err_code;
	  uint8_t tx_buf[2];
	
	  //init buffer
		tx_buf[0] = register_address;
    tx_buf[1] = value;
	  //set sign
		h_xfer_done = false;
		//send command
    err_code = nrf_drv_twi_tx(&h_twi, HW611_ADDRESS, tx_buf, 2, false);
	  //wait for the sign change
		while (h_xfer_done == false){}
	  if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;	
}

/*
read the value of the register
destination[out] is the pointer to where the date will be saved in RAM
number_of_bytes is the len you want to read
return true means success
used in next function
*/ 
bool hw611_register_read(uint8_t register_address, uint8_t * destination, uint8_t number_of_bytes)
{
	  ret_code_t err_code;
	  //set sign
		h_xfer_done = false;
		//send the address
	  err_code = nrf_drv_twi_tx(&h_twi,HW611_ADDRESS, &register_address, 1, true);
	  //wait
		while (h_xfer_done == false){}
    if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		//set
		h_xfer_done = false;
		//read
	  err_code = nrf_drv_twi_rx(&h_twi, HW611_ADDRESS, destination, number_of_bytes);
		//wait
		while (h_xfer_done == false){}
		if (NRF_SUCCESS != err_code)
    {
        return false;
    }
		return true;
}

/*
read a byte
failed sign is 0xFF(because the register value is default 0xFF)
*/
uint8_t hw611_read_byte(uint8_t reg)
{
	uint8_t rec_data;
	if(hw611_register_read(reg,&rec_data,1))return rec_data;
	return 0xFF;
}

/*
get the hw611 id
can use to check if the hw611 correctly connected with MCU
*/
uint8_t hw611_get_id()
{
	return hw611_read_byte(HW611_CHIPID_REG);
}


/*
Set the oversamping factor
MODE can be HW611_SLEEP_MODE||HW611_FORCED_MODE||HW611_NORMAL_MODE
*/
void hw611_set_tem_oversamp(HW611_OVERSAMPLE_MODE * Oversample_Mode)
{
	uint8_t Regtmp;
	Regtmp = ((Oversample_Mode->T_Osample)<<5)|
			 ((Oversample_Mode->P_Osample)<<2)|
			 ((Oversample_Mode)->WORKMODE);
	
	hw611_write_byte(HW611_CTRLMEAS_REG,Regtmp);
}


/*
Set standby time and the factor of fliter
*/
void hw611_set_standby_fliter(HW611_CONFIG * HW611_Config)
{
	uint8_t Regtmp;

	Regtmp = ((HW611_Config->T_SB)<<5)|((HW611_Config->FILTER_COEFFICIENT)<<2);
	
	hw611_write_byte(HW611_CONFIG_REG,Regtmp);
}

/*
INIT THE HW611
should use after init twi device
*/
void hw611_init(void)
{
	uint8_t Lsb,Msb;
	
	/********************read the compensation factor*********************/
	//temperature
	Lsb = hw611_read_byte(HW611_DIG_T1_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_T1_MSB_REG);
	hw611->T1 = (((uint16_t)Msb)<<8) + Lsb;			//high + low
	Lsb = hw611_read_byte(HW611_DIG_T2_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_T2_MSB_REG);
	hw611->T2 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = hw611_read_byte(HW611_DIG_T3_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_T3_MSB_REG);
	hw611->T3 = (((uint16_t)Msb)<<8) + Lsb;		
	
	//pressure
	Lsb = hw611_read_byte(HW611_DIG_P1_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P1_MSB_REG);
	hw611->P1 = (((uint16_t)Msb)<<8) + Lsb;		
	Lsb = hw611_read_byte(HW611_DIG_P2_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P2_MSB_REG);
	hw611->P2 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P3_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P3_MSB_REG);
	hw611->P3 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P4_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P4_MSB_REG);
	hw611->P4 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P5_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P5_MSB_REG);
	hw611->P5 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P6_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P6_MSB_REG);
	hw611->P6 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P7_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P7_MSB_REG);
	hw611->P7 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P8_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P8_MSB_REG);
	hw611->P8 = (((uint16_t)Msb)<<8) + Lsb;	
	Lsb = hw611_read_byte(HW611_DIG_P9_LSB_REG);
	Msb = hw611_read_byte(HW611_DIG_P9_MSB_REG);
	hw611->P9 = (((uint16_t)Msb)<<8) + Lsb;	
	/******************************************************/
	hw611_write_byte(HW611_RESET_REG,HW611_RESET_VALUE);	//reset
	
	HW611_OVERSAMPLE_MODE			HW611_OVERSAMPLE_MODEStructure;
	HW611_OVERSAMPLE_MODEStructure.P_Osample = HW611_P_MODE_3;
	HW611_OVERSAMPLE_MODEStructure.T_Osample = HW611_T_MODE_1;
	HW611_OVERSAMPLE_MODEStructure.WORKMODE  = HW611_NORMAL_MODE;
	hw611_set_tem_oversamp(&HW611_OVERSAMPLE_MODEStructure);
	
	HW611_CONFIG					HW611_CONFIGStructure;
	HW611_CONFIGStructure.T_SB = HW611_T_SB1;
	HW611_CONFIGStructure.FILTER_COEFFICIENT = HW611_FILTER_MODE_4;
	//HW611_CONFIGStructure.SPI_EN = DISABLE;
	
	hw611_set_standby_fliter(&HW611_CONFIGStructure);
}

/*
check the status of HW611
flag can be HW611_MRASURING || HW611_IM_UPDATE
*/
bool hw611_get_status(uint8_t status_flag)
{
	uint8_t flag;
	flag = hw611_read_byte(HW611_STATUS_REG);
	if (flag&status_flag)return true;
	else return false;
}

/**************************COMPENSATE MODE*************************************/
/*
This part is original code that have't checked
wait for your COMMIT!
*/
#ifdef USE_FIXED_POINT_COMPENSATE
// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC. 
// t_fine carries fine temperature as global value
BMP280_S32_t bmp280_compensate_T_int32(BMP280_S32_t adc_T)
{
	BMP280_S32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((BMP280_S32_t)dig_T1<<1))) * ((BMP280_S32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((BMP280_S32_t)dig_T1)) * ((adc_T>>4) - ((BMP280_S32_t)dig_T1))) >> 12) * 
	((BMP280_S32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
BMP280_U32_t bmp280_compensate_P_int64(BMP280_S32_t adc_P)
{
	BMP280_S64_t var1, var2, p;
	var1 = ((BMP280_S64_t)t_fine) - 128000;
	var2 = var1 * var1 * (BMP280_S64_t)dig_P6;
	var2 = var2 + ((var1*(BMP280_S64_t)dig_P5)<<17);
	var2 = var2 + (((BMP280_S64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (BMP280_S64_t)dig_P3)>>8) + ((var1 * (BMP280_S64_t)dig_P2)<<12);
	var1 = (((((BMP280_S64_t)1)<<47)+var1))*((BMP280_S64_t)dig_P1)>>33;
	if (var1 == 0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((BMP280_S64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((BMP280_S64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((BMP280_S64_t)dig_P7)<<4);
	return (BMP280_U32_t)p;
}


/***********************************CUT*************************************/
#else
/**************************?*************************************/
// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
// t_fine carries fine temperature as global value
double hw611_compensate_T_double(int32_t adc_T)
{
	double var1, var2, T;
	var1 = (((double)adc_T)/16384.0 - ((double)dig_T1)/1024.0) * ((double)dig_T2);
	var2 = ((((double)adc_T)/131072.0 - ((double)dig_T1)/8192.0) *
	(((double)adc_T)/131072.0 - ((double) dig_T1)/8192.0)) * ((double)dig_T3);
	t_fine = (int32_t)(var1 + var2);
	T = (var1 + var2) / 5120.0;
	return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double hw611_compensate_P_double(int32_t adc_P)
{
	double var1, var2, p;
	var1 = ((double)t_fine/2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dig_P5) * 2.0;
	var2 = (var2/4.0)+(((double)dig_P4) * 65536.0);
	var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0)*((double)dig_P1);
	if (var1 == 0.0)
	{
	return 0; // avoid exception caused by division by zero
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dig_P8) / 32768.0;
	p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
	return p;
}
#endif



/*
Main Part!
Get the real value from the HW611
Can easy use after init twi and HW611
*/
double hw611_get_pressure(void)
{
	uint8_t xlsb,lsb,msb;
	int32_t bit32;
	double pressure;
	//double pressure
	while(hw611_get_status(HW611_MEASURING)!=false);
	while(hw611_get_status(HW611_IM_UPDATE)!=false);
	xlsb = hw611_read_byte(HW611_PRESSURE_XLSB_REG);
	lsb = hw611_read_byte(HW611_PRESSURE_LSB_REG);
	msb = hw611_read_byte(HW611_PRESSURE_MSB_REG);
	bit32 = (((int32_t)(msb<<12))|((int32_t)(lsb<<4))|(xlsb>>4));//make a float
	pressure = hw611_compensate_P_double(bit32);
	return pressure;
}


//return temperature
double hw611_get_temperature(void)
{
	uint8_t xlsb,lsb,msb;
	int32_t bit32;
	double temperature;
	//double temp
	while(hw611_get_status(HW611_MEASURING)!=false);
	while(hw611_get_status(HW611_IM_UPDATE)!=false);
	xlsb = hw611_read_byte(HW611_TEMPERATURE_XLSB_REG);
	lsb = hw611_read_byte(HW611_TEMPERATURE_LSB_REG);
	msb = hw611_read_byte(HW611_TEMPERATURE_MSB_REG);
	bit32 = (((int32_t)(msb<<12))|((int32_t)(lsb<<4))|(xlsb>>4));
	temperature = hw611_compensate_T_double(bit32);
	return temperature;
}
