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
#include <math.h>
#include "../include/INA3221.h"
#include "../include/ATXMEGA32A4U.h"

#pragma endregion INCLUDES

#pragma region GLOBAL VAR

float alpha = 0.0f;
float alpha1 = 0.0f;
float beta = 0.0f;
float beta1 = 0.0f;
float gammad = 0.0f;
float gamma1 = 0.0f;
int16_t xPos = 0;
int16_t yPos = 0;
int16_t zPos = 0;
const float A1 = 52;
const float A2 = 69;
const float A3 = 88;
float L1 = 0;
float L2 = 0;
float L3 = 0;
const float zOffset = 88;

#pragma endregion GLOBAL VAR

#pragma region FUNCTIONS

//Inverse Kinematic
void IK(){




//ALPHA
alpha = atan2(xPos, A1 + A2 + yPos);

//BETA
L1 = zOffset - zPos;
L2 = A2 + yPos;
L3 = sqrt(L1 * L1 + L2 * L2);

beta = acos(L1 / L3);

beta = beta + acos((A2 * A2 - A3 * A3 + L3 * L3) / (2 * A2 * L3));

//GAMMA
gammad = acos((A3 * A3 - L3 * L3 + A2 * A2) / (2 * A3 * A2));

//RAD TO DEG


alpha = (alpha * 180 / M_PI - alpha1) * -1;
beta = (beta * 180 / M_PI - beta1 - 90) * 1;
gammad = (gammad * 180 / M_PI - gamma1 - 90) * -1;

servo_set_position(alpha,beta,gammad);



}


#pragma endregion FUNCTIONS


int main(void)
{
	init_system_clock(); //Initialize system clock
	//init_pll(); //Initialize PLL
	init_watchdog();
	init_gpio(); //Initialize GPIO
	init_LED(); //Initialize LED
	init_servo(); //Initialize servos
	init_twiE_MASTER(); //Initialize MASTER TWI
	init_twiC_SLAVE(); //Initialize SLAVE TWI
	init_UART(); //Initialize UART
	
	asm("wdr");

	twi_master_send_data(INA_CFG_R,(uint16_t) (INA_RST_B));
	twi_master_send_data(INA_CFG_R, 0b0110010111111111);
	//uart_send_word(twi_master_read_data(INA_MANUFACTURER_ID_R)); 
	//uart_send_word(twi_master_read_data(INA_DIE_ID_R));
	

	uint16_t hue = 0;
	while (1)
	{
		asm("wdr");

		twi_slave_get_data();
		IK();
		led_set_color(hue,1,1);
		hue = (hue+1)%360;
		delay(10);
		

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