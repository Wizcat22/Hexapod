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
#include "../include/INA3221.h"
#include "../include/ATXMEGA32A4U.h"

#pragma endregion INCLUDES

#pragma region GLOBAL VAR




#pragma endregion GLOBAL VAR

#pragma region FUNCTIONS



#pragma endregion FUNCTIONS


int main(void)
{
	init_system_clock(); //Initialize system clock
	//init_pll(); //Initialize PLL
	init_gpio(); //Initialize GPIO
	init_LED(); //Initialize LED
	init_servo(); //Initialize servos
	init_twiE_MASTER(); //Initialize MASTER TWI
	init_twiC_SLAVE(); //Initialize SLAVE TWI
	init_UART(); //Initialize UART
	

	twi_master_send_data(INA_CFG_R,(uint16_t) (INA_RST_B));
	twi_master_send_data(INA_CFG_R, 0b0100111111111111);
	twi_master_read_data(INA_MANUFACTURER_ID_R);
	uart_send('\n');
	twi_master_read_data(INA_DIE_ID_R);
	uart_send('\n');
	//int16_t STROM = twi_master_read_data(INA_C1_SV_R);
	//float erg = STROM /2500;
	//uint8_t asd = (uint8_t) (floor(erg));
	//uart_send('\n');



	while (1)
	{
		twi_slave_get_data();
		servo_set_position();

	}
}

