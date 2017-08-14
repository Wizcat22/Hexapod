/*
 * ATXMEGA32A4U.h
 *
 * Created: 05.03.2017 12:33:21
 *  Author: Alexander Miller
 */ 


#ifndef ATXMEGA32A4U_H_
#define ATXMEGA32A4U_H_

#pragma region DEFINES

/**
 * @def	F_TWI_NS
 *
 * @brief	A macro that defines twi normal speed (100kHz)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define F_TWI_NS 100000UL

/**
 * @def	F_TWI_HS
 *
 * @brief	A macro that defines twi high speed (400kHz)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

#define F_TWI_HS 400000UL



#pragma endregion DEFINES

#pragma region FUNCTIONS

/**
 * @fn	void init_system_clock(void);
 *
 * @brief	Initializes the system clock
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_system_clock(void);

/**
 * @fn	void init_watchdog(void);
 *
 * @brief	Initializes the watchdog
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_watchdog(void);

/**
 * @fn	void init_pll(void);
 *
 * @brief	Initializes the pll
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_pll(void);

/**
 * @fn	void init_gpio(void);
 *
 * @brief	Initializes the gpio
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_gpio(void);

/**
 * @fn	void init_servo(void);
 *
 * @brief	Initializes the servo
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_servo(void);

/**
 * @fn	void init_twiE_MASTER(void);
 *
 * @brief	Initializes the twi e as master
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_twiE_MASTER(void);

/**
 * @fn	void init_twiC_SLAVE(void);
 *
 * @brief	Initializes the twi c as slave
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_twiC_SLAVE(void);

/**
 * @fn	void init_UART(void);
 *
 * @brief	Initializes the uart
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_UART(void);

/**
 * @fn	void init_eeprom(void);
 *
 * @brief	Initializes the eeprom
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_eeprom(void);

/**
 * @fn	void uart_send(char data);
 *
 * @brief	Send one char via uart
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	data	The data.
 */

void uart_send(char data);

/**
 * @fn	void uart_send_word(uint16_t data);
 *
 * @brief	Send one word (uint16) via uart
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	data	The data.
 */

void uart_send_word(uint16_t data);

/**
 * @fn	void uart_send_string(char s[]);
 *
 * @brief	Send string via uart
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	s	The String to send via uart.
 */

void uart_send_string(char s[]);

/**
 * @fn	void uart_send_number(int num);
 *
 * @brief	Send a number as a string
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	num	Number to send as string.
 */

void uart_send_number(int num);

/**
 * @fn	void led_set_color(uint16_t H, float S, float V);
 *
 * @brief	Set LED color (HSV)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	H	Hue.
 * @param	S	Saturation.
 * @param	V	Value.
 */

void led_set_color(uint16_t H, float S, float V);

/**
 * @fn	void init_LED(void);
 *
 * @brief	Initializes the LED Timer
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void init_LED(void);

/**
 * @fn	void twi_slave_get_data(void);
 *
 * @brief	Handles TWI communication (receive Commands and paramenters)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 */

void twi_slave_get_data(void);

/**
 * @fn	uint8_t twi_slave_get_byte(void);
 *
 * @brief	Get one byte as TWI slave
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @return	Received byte.
 */

uint8_t twi_slave_get_byte(void);

/**
 * @fn	uint16_t twi_slave_get_word(void);
 *
 * @brief	Get word via TWI slave.
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @return	Received word.
 */

uint16_t twi_slave_get_word(void);

/**
 * @fn	void twi_master_send_data(char reg,uint16_t data);
 *
 * @brief	Send data as TWI master
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	reg 	The register to write to.
 * @param	data	The data.
 */

void twi_master_send_data(char reg,uint16_t data);

/**
 * @fn	int16_t twi_master_read_data(char reg);
 *
 * @brief	Read data (16bit) as TWI master
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	reg	The register.
 *
 * @return	The received data.
 */

int16_t twi_master_read_data(char reg);

/**
 * @fn	void servo_set_deg(int8_t s0, int8_t s1, int8_t s2);
 *
 * @brief	Set servo position via the angle
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	s0	Angle of servo 0.
 * @param	s1	Angle of servo 1.
 * @param	s2	Angle of servo 2.
 */

void servo_set_deg(int8_t s0, int8_t s1, int8_t s2);

/**
 * @fn	void leg_set_position(int8_t xPos, int8_t yPos, int8_t zPos);
 *
 * @brief	Set TCP position. Calculates the angles (inverse kinematics)
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	xPos	x-coordinate.
 * @param	yPos	y-coordinate.
 * @param	zPos	z-coordinate.
 */

void leg_set_position(int8_t xPos, int8_t yPos, int8_t zPos);

/**
 * @fn	void leg_sense_terrain(int8_t xPos, int8_t yPos, int8_t zPos);
 *
 * @brief	Set TCP position and check ground contact
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	xPos	The x position.
 * @param	yPos	The y position.
 * @param	zPos	The z position.
 */

void leg_sense_terrain(int8_t xPos, int8_t yPos, int8_t zPos);

/**
 * @fn	void delay(int ms);
 *
 * @brief	Delays execution for x milliseconds
 *
 * @author	Alexander Miller
 * @date	14.08.2017
 *
 * @param	ms	The time in milliseconds.
 */

void delay(int ms);


#pragma endregion FUNCTIONS


#endif /* ATXMEGA32A4U_H_ */