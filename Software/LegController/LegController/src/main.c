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
	
	//uint16_t hue = 0;
	//float huehue = 0;
	while (1)
	{
		asm("wdr"); //Reset Watchdog

		twi_slave_get_data();
		
		//led_set_color(hue,1,0.05);
		//hue = ((uint16_t)huehue)%360;
		//huehue = huehue + 0.1;
		//if (huehue > 360)
		//{
			//huehue = 0;
		//}
		
	}
}