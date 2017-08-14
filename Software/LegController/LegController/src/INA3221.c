/*
* INA3221.c
*
* Created: 26.02.2017 19:26:23
*  Author: Alexander Miller
*/

#pragma region INCLUDES

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "../include/ATXMEGA32A4U.h"
#include "../include/INA3221.h"

#pragma endregion INCLUDES

#pragma region VARIABLES

/** @brief	The configuration */
uint16_t config = 0xFFFF & (INA_CH2_EN_B | INA_AVG_MODE_1024_B | INA_SV_CONV_TIME_140us_B | INA_OP_MODE_SV_SINGLE_SHOT_B); //config = 0b0010111000000001;
/** @brief	The current limit for ground contact */
uint16_t limit = 125;


#pragma endregion VARIABLES

#pragma region FUNCTIONS

/**
 * @fn	void ina3221_set_config(uint16_t config)
 *
 * @brief	Set cinfiguration of INA3221
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	config	The configuration.
 */

void ina3221_set_config(uint16_t config){
	twi_master_send_data(INA_CFG_R,config);
}

/**
 * @fn	int16_t ina3221_read_value(char reg)
 *
 * @brief	Read one value from INA3221
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	reg	The register to read from.
 *
 * @return	An int16_t.
 */

int16_t ina3221_read_value(char reg){
	return twi_master_read_data(reg);
}

/**
 * @fn	uint16_t ina3221_get_current(uint16_t channel)
 *
 * @brief	Get shunt voltage of channel x
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	channel	The channel.
 *
 * @return	The shuntvoltage.
 */

uint16_t ina3221_get_current(uint16_t channel){
	
	//first value
	uint16_t current1 = 0;
	//second value
	uint16_t current2 = 0;
	//trigger one measurement
	ina3221_trigger_measurement();
	//calculate current and save the value
	current1 = ina3221_calculate_current(channel);
	//trigger one measurement
	ina3221_trigger_measurement();
	//calculate current and save the value
	current2 = ina3221_calculate_current(channel);
	
	//while the difference between the two measurements is to big (measurement error)
	while (!(abs(current1-current2) < 50))
	{
		//save last value
		current2 = current1;
		//trigger measurement
		ina3221_trigger_measurement();
		//save new value
		current1 = ina3221_calculate_current(channel);
	}
	
	//return average current
	return (current1+current2)/2;
}

/**
 * @fn	uint16_t ina3221_calculate_current(uint16_t channel)
 *
 * @brief	Calculate the current with shuntvoltage reading
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	channel	The channel.
 *
 * @return	Current in ma.
 */

uint16_t ina3221_calculate_current(uint16_t channel){
	uint16_t current = 0;
	//read shunt voltage
	current = ina3221_read_value(channel);
	//take absolute value (ina3221 has signed values)
	current = abs(current);
	//shift the value 3 times to the right
	current = current>>3;
	//calc the current with I=LSB*shuntvoltage / resistor
	current = current *40/100;
	
	return current;
}

/**
 * @fn	void ina3221_init()
 *
 * @brief	Initializes the sensor
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void ina3221_init(){
	//reset the sensor
	ina3221_set_config((uint16_t) (INA_RST_B)); //Reset
	//set config
	ina3221_set_config(config); //Config
}

/**
 * @fn	void ina3221_trigger_measurement()
 *
 * @brief	Trigger one measurement
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void ina3221_trigger_measurement(){
	//send configuration to trigger one measurement
	ina3221_set_config(config);
	//wait until the measurement is ready
	while ((ina3221_read_value(INA_MASK_ENABLE_R)&(1)) ==0)
	{

	}
}

/**
 * @fn	uint8_t ina3221_check_ground()
 *
 * @brief	Check for ground contact by measuring the current flow
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @return	Flag 1=contact 0=no contact.
 */

uint8_t ina3221_check_ground(){

	//get flowing current
	uint16_t current = ina3221_get_current(INA_C2_SV_R);
	
	//if the current value exceeds the threshold
	if (current >limit)
	{
		//return ground contact
		return 1;
	}

	//return no contact
	return 0;

}

#pragma endregion FUNCTIONS