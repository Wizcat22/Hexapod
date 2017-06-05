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
void init_watchdog(void);
void init_pll(void);
void init_gpio(void);
void init_servo(void);
void init_twiE_MASTER(void);
void init_twiC_SLAVE(void);
void init_UART(void);
void init_eeprom(void);
void uart_send(char data);
void uart_send_word(uint16_t data);
void uart_send_string(char s[]);
void uart_send_number(int num);
void led_set_color(uint16_t H, float S, float V);
void init_LED(void);
void twi_slave_get_data(void);
uint8_t twi_slave_get_byte(void);
uint16_t twi_slave_get_word(void);
void twi_master_send_data(char reg,uint16_t data);
int16_t twi_master_read_data(char reg);
void servo_set_deg(int8_t s0, int8_t s1, int8_t s2);
void leg_set_position(int8_t s0, int8_t s1, int8_t s2);
void leg_sense_terrain(int8_t xPos, int8_t yPos, int8_t zPos);
void delay(int ms);


#pragma endregion FUNCTIONS


#endif /* ATXMEGA32A4U_H_ */