/*
 * ATXMEGA32A4U.h
 *
 * Created: 05.03.2017 12:33:21
 *  Author: Alexander Miller
 */ 


#ifndef ATXMEGA32A4U_H_
#define ATXMEGA32A4U_H_

#pragma region DEFINES


#define F_TWI_NS 100000UL
#define F_TWI_HS 400000UL

#pragma endregion DEFINES

#pragma region FUNCTIONS

void init_system_clock(void);
void init_pll(void);
void init_gpio(void);
void init_servo(void);
void init_twiE_MASTER(void);
void init_twiC_SLAVE(void);
void init_UART(void);
void uart_send(char data);
void led_set_color(uint16_t H, float S, float V);
void init_LED(void);
void twi_slave_get_data(void);
void twi_master_send_data(char reg,uint16_t data);
int16_t twi_master_read_data(char reg);
void servo_set_position(void);
void delay(int ms);


#pragma endregion FUNCTIONS


#endif /* ATXMEGA32A4U_H_ */