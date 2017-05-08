/*
* LegController.c
*
* Created: 12.10.2016 14:45:19
* Author : Alexander Miller
*/

#pragma region DEFINES



#pragma endregion DEFINES

#pragma region INCLUDES

#include <avr/io.h>
#include <avr/eeprom.h>
#include <math.h>
#include "../include/ATXMEGA32A4U.h"
#include "../include/INA3221.h"


#pragma endregion INCLUDES

#pragma region GLOBAL VAR



#pragma endregion GLOBAL VAR

#pragma region FUNCTIONS




#pragma endregion FUNCTIONS


int main(void)
{
	init_system_clock(); //Initialize system clock
	//init_pll(); //Initialize PLL
	
	
	delay(1000); //Delay to reduce risk of eeprom coruption during programming



	init_watchdog();
	init_gpio(); //Initialize GPIO
	init_LED(); //Initialize LED
	init_twiE_MASTER(); //Initialize MASTER TWI
	init_twiC_SLAVE(); //Initialize SLAVE TWI
	init_UART(); //Initialize UART
	init_eeprom(); //Initialize EEPROM Data
	init_servo(); //Initialize servos
	
	asm("wdr"); //Reset Watchdog

	//twi_master_send_data(INA_CFG_R,(uint16_t) (INA_RST_B)); //RESET
	//twi_master_send_data(INA_CFG_R, 0b0111010111111111); //CONFIG
	//uart_send_word(twi_master_read_data(INA_MANUFACTURER_ID_R));
	//uart_send_word(twi_master_read_data(INA_DIE_ID_R));

	//ina3221_set_config(INA_CFG_R, (uint16_t) (INA_RST_B)); //Reset
	//ina3221_set_config(INA_CFG_R, 0b0111010111111111); //Config
	//uart_send_word(ina3221_read_value(INA_MANUFACTURER_ID_R)); 
	//uart_send_word(ina3221_read_value(INA_DIE_ID_R));
	
	uint16_t hue = 0;
	float huehue = 0;
	uint16_t a=0;
	while (1)
	{
		asm("wdr"); //Reset Watchdog

		twi_slave_get_data();
		//led_set_color(hue,1,0.05);

		hue = ((uint16_t)huehue)%360;
		huehue = huehue + 0.1;
		if (huehue > 360)
		{
			huehue = 0;
		}
		delay(1);
		//a = twi_master_read_data(INA_C1_SV_R);
		//
		//a = abs(a);
		//a = a>>3;
		//a = a*40/100;
		////uart_send_string("A: ");
		//uart_send_number(a);
//
		//a = twi_master_read_data(INA_C2_SV_R);
		//
		//a = abs(a);
		//a = a>>3;
		//a = a*40/100;
		////uart_send_string(" B: ");
		//uart_send_string(",");
		//uart_send_number(a);
//
		//a = twi_master_read_data(INA_C3_SV_R);
		//
		//a = abs(a);
		//a = a>>3;
		//a = a*40/100;
		////uart_send_string(" C: ");
		//uart_send_string(",");
		//uart_send_number(a);
		//
		//uart_send('\r');
		//uart_send('\n');

		//while (1)
		//{
			//asm("wdr");
		//}
	}
}

//int16_t a = twi_master_read_data(INA_C1_SV_R);
//
//a = abs(a);
//a = a>>3;
//a = a*40/100;
//
//char str[15];
//sprintf(str, "%d", a);
//uart_send_string(str);
//uart_send_string(" mA");
//uart_send('\n');