/*
 * INA3221.c
 *
 * Created: 26.02.2017 19:26:23
 *  Author: Alexander Miller
 */
 
#pragma region INCLUDES

 #include <avr/io.h>
 #include <util/delay.h>
 #include <math.h>
 #include "../include/ATXMEGA32A4U.h"
 #include "../include/INA3221.h"

#pragma endregion INCLUDES

#pragma region VARIABLES

uint16_t config = 0b0111111111111011;
uint16_t measurements[] = {0,0,0};

#pragma endregion VARIABLES

#pragma region FUNCTIONS

void ina3221_set_config(uint16_t config){
	twi_master_send_data(INA_CFG_R,config);
}

int16_t ina3221_read_value(char reg){
	return twi_master_read_data(reg);
}

void ina3221_get_current(uint16_t data[]){
	
	uint16_t current = 0;
	ina3221_trigger_measurement();
	data[0] = ina3221_calculate_current(INA_C1_SV_R);
	data[1] = ina3221_calculate_current(INA_C2_SV_R);
	data[2] = ina3221_calculate_current(INA_C3_SV_R);
	
}

uint16_t ina3221_calculate_current(uint16_t channel){
	uint16_t current = 0;
	current = ina3221_read_value(channel);
	current = abs(current);
	current = current>>3;
	current = current *40/100;
	return current;
}

void ina3221_init(){
	ina3221_set_config((uint16_t) (INA_RST_B)); //Reset
	ina3221_set_config(config); //Config
	//uart_send_word(ina3221_read_value(INA_MANUFACTURER_ID_R));
	//uart_send_word(ina3221_read_value(INA_DIE_ID_R));
}

void ina3221_trigger_measurement(){
	ina3221_set_config(config);
	while ((ina3221_read_value(INA_MASK_ENABLE_R)&(1<<0)) ==0)
	{
		delay(1);	
	}
}

uint8_t ina3221_check_ground(){

//TODO

return 0;

}

#pragma endregion FUNCTIONS