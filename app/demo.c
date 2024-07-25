#include <string.h>
#include <stdbool.h>   
#include <stdint.h>
#include <stdio.h>
#include "nrf.h"
#include "bsp.h"
#include "nrf_delay.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"
#include "hw611.h"

#define NRF_LOG_FLOAT_MAKER "%s%d.%02d"

static void log_init(void)
{
	  ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


int main(void)
{
	log_init();
	bsp_board_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS); 
	twi_master_init();
	NRF_LOG_INFO("TWI INIT SUCESS");
	nrf_delay_ms(3000);
	NRF_LOG_INFO("hw611 id:0x%x",hw611_get_id());
	nrf_delay_ms(1000);
	hw611_init();
	double hw_p,hw_t;
	while(true)  
	{  
		hw_p = hw611_get_pressure();
		hw_t = hw611_get_temperature();
		NRF_LOG_INFO("pressure:"NRF_LOG_FLOAT_MAKER",temperature:"NRF_LOG_FLOAT_MAKER,NRF_LOG_FLOAT(hw_p),NRF_LOG_FLOAT(hw_t));
		nrf_delay_ms(1000);
	}
}
