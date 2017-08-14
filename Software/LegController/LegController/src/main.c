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

/**
 * @fn	int main(void)
 *
 * @brief	Main entry-point for this application
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 */

int main(void)
{
	init_system_clock(); //Initialize system clock
	//init_pll(); //Initialize PLL
	
	
	delay(1000); //Delay to reduce risk of eeprom coruption during programming



	init_watchdog(); //Initialize Watchdog
	init_gpio(); //Initialize GPIO
	init_LED(); //Initialize LED
	init_twiE_MASTER(); //Initialize MASTER TWI
	init_twiC_SLAVE(); //Initialize SLAVE TWI
	init_UART(); //Initialize UART
	init_eeprom(); //Initialize EEPROM Data
	init_servo(); //Initialize servos
	
	asm("wdr"); //Reset Watchdog
	

	while (1)
	{
		asm("wdr"); //Reset Watchdog

		twi_slave_get_data(); //check for communication -> receive commands -> execute commands
	
	}
}