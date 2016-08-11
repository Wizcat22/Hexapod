/*
* Raspberry_PI_Servo_Hat.c
*
* Created: 23.05.2016 17:38:40
* Author : Alexander Miller
*/


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

/**********************************************************************************************//**
 * @def	SLAVE_ADDRESS
 *
 * @brief	A macro that defines the i2c slave address.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define SLAVE_ADDRESS 0x42 //TWI Slave address

/**********************************************************************************************//**
 * @def	NUM_SERVOS
 *
 * @brief	A macro that defines the maximum number of servos.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define NUM_SERVOS 24 // 20 is the recommended maximum value (Servos get slower because of longer standby time (>20ms))

/**********************************************************************************************//**
 * @def	SERVO_STD_VAL
 *
 * @brief	A macro that defines servo standard value.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define SERVO_STD_VAL 187 //STANDARD SERVO POSITION (1,5ms)

/**********************************************************************************************//**
 * @def	SERVO_MIN_VAL
 *
 * @brief	A macro that defines servo minimum value.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define SERVO_MIN_VAL 125 //MINUMUM (-45°) SERVO POSITION (1ms)

/**********************************************************************************************//**
 * @def	SERVO_MAX_VAL
 *
 * @brief	A macro that defines servo maximum value.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define SERVO_MAX_VAL 250 //MAXIMUM (+45°) SERVO POSITION (2ms)

/**********************************************************************************************//**
 * @def	LED1_ON
 *
 * @brief	A macro to turn LED 1 on.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED1_ON PORTD|=(1<<PD0) //MACRO: Turn LED1 on

/**********************************************************************************************//**
 * @def	LED2_ON
 *
 * @brief	A macro to turn LED 2 on.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED2_ON PORTD|=(1<<PD1) //MACRO: Turn LED2 on

/**********************************************************************************************//**
 * @def	LED3_ON
 *
 * @brief	A macro to turn LED 3 on.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED3_ON PORTD|=(1<<PD2) //MACRO: Turn LED3 on

/**********************************************************************************************//**
 * @def	LED1_OFF
 *
 * @brief	A macro to turn LED 1 off.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED1_OFF PORTD&=~(1<<PD0) //MACRO: Turn LED1 off

/**********************************************************************************************//**
 * @def	LED2_OFF
 *
 * @brief	A macro to turn LED 2 off.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED2_OFF PORTD&=~(1<<PD1) //MACRO: Turn LED2 off

/**********************************************************************************************//**
 * @def	LED3_OFF
 *
 * @brief	A macro to turn LED 3 off.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

#define LED3_OFF PORTD&=~(1<<PD2) //MACRO: Turn LED3 off



/** @brief	CHAR ARRAY WITH SERVO DATA. */
unsigned volatile char data[NUM_SERVOS];

/** @brief	loop count. */
unsigned volatile char loop = 0;

/**********************************************************************************************//**
 * @fn	void init_twi(void)
 *
 * @brief	Init twi.
 * 			Configure TWI interface as slave.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

void init_twi(void){

	//Set address of i2c slave and enable general call
	TWAR = ((SLAVE_ADDRESS<<1) | (1<<TWGCE));
	TWCR |= (1<<TWEN) | (1<<TWEA);
	TWCR &= ~((1<<TWSTA) | (1<<TWSTO)) ;

}

/**********************************************************************************************//**
 * @fn	void init_outputs(void)
 *
 * @brief	Init outputs.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

void init_outputs(void){

	DDRA = 0xFF; //Servo Outputs: PA0=SERVO_19 - PA7 = SERVO_12
	DDRB = 0x1F; //Servo Outputs: PB0=SERVO_20 - PB4 = SERVO_24
	DDRC = 0xFC; //Servo Outputs: PC2=SERVO_6 - PC7 = SERVO_11
	DDRD = 0xFF; //Servo Outputs: PD3=SERVO_1 - PD7 = SERVO_5 ; LED Outputs: PD0 = LED_1 - PD2 = LED_3

}



/**********************************************************************************************//**
 * @fn	void init_timer(void)
 *
 * @brief	Init timer.
 * 			Configure Timer0 to work in Fast-PWM-Mode with TOP at 0xFF and OCRx update at TOP.
 * 			The Prescaler is set to 64.
 * 			Time per overflow = 2,048ms 
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

void init_timer(void){

	//TIMER0 (8bit) , Mode 3 - Fast PWM TOP = 0xFF ,OCRx update at TOP, prescaler = 64 , Time per overflow = 0.002048 sec. = 2.048 ms (at 8Mhz Clock)
	TCCR0A |= (1<<WGM01) | (1<<WGM00);
	TCCR0B |= (1<<CS01) | (1<<CS00);

	//Enable TIMER0 Interrupts (Compare Match A/B and Overflow)
	TIMSK0 |= (1<<OCIE0A) | (1<<OCIE0B) | (1<<TOIE0);
}


/**********************************************************************************************//**
 * @fn	ISR(TIMER0_COMPA_vect)
 *
 * @brief	Interrupt Service Routine to end Servopulse 1 - 12
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 *
 **************************************************************************************************/

ISR(TIMER0_COMPA_vect){

	PORTD &= ~(0xF8) ;
	PORTC &= ~(0xFC) ;
	PORTA &= ~(0x80) ;

}

/**********************************************************************************************//**
 * @fn	ISR(TIMER0_COMPB_vect)
 *
 * @brief	Interrupt Service Routine to end Servopulse 13 - 24
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 *
 **************************************************************************************************/

ISR(TIMER0_COMPB_vect){

	PORTA &= ~(0x7F) ;
	PORTB &= ~(0x1F) ;

}


//ISR TIOV -> SET SERVO OUTPUTS AND SET CORRESPONDING OC0A/OC0B VALUES

/**********************************************************************************************//**
 * @fn	ISR(TIMER0_OVF_vect)
 *
 * @brief	Interrupt Service Routine for timer overflow.
 * 			This ISR starts the servo signals for two servos and sets OCRx values to the length of the pulses.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 *
 **************************************************************************************************/

ISR(TIMER0_OVF_vect){

	//Start pulse for two servos
	switch (loop)
	{
		case 0:
		PORTD |= (1<<PD3); //SERVO 1
		PORTA |= (1<<PA6); //SERVO 13
		break;
		case 1:
		PORTD |= (1<<PD4); //SERVO 2
		PORTA |= (1<<PA5); //SERVO 14
		break;
		case 2:
		PORTD |= (1<<PD5); //SERVO 3
		PORTA |= (1<<PA4); //SERVO 15
		break;
		case 3:
		PORTD |= (1<<PD6); //SERVO 4
		PORTA |= (1<<PA3); //SERVO 16
		break;
		case 4:
		PORTD |= (1<<PD7); //SERVO 5
		PORTA |= (1<<PA2); //SERVO 17
		break;
		case 5:
		PORTC |= (1<<PC2); //SERVO 6
		PORTA |= (1<<PA1); //SERVO 18
		break;
		case 6:
		PORTC |= (1<<PC3); //SERVO 7
		PORTA |= (1<<PA0); //SERVO 19
		break;
		case 7:
		PORTC |= (1<<PC4); //SERVO 8
		PORTB |= (1<<PB0); //SERVO 20
		break;
		case 8:
		PORTC |= (1<<PC5); //SERVO 9
		PORTB |= (1<<PB1); //SERVO 21
		break;
		case 9:
		PORTC |= (1<<PC6); //SERVO 10
		PORTB |= (1<<PB2); //SERVO 22
		break;
		case 10:
		PORTC |= (1<<PC7); //SERVO 11
		PORTB |= (1<<PB3); //SERVO 23
		break;
		case 11:
		PORTA |= (1<<PA7); //SERVO 12
		PORTB |= (1<<PB4); //SERVO 24
		break;
		default: break;
	}
	loop = (loop+1)%(NUM_SERVOS/2);
	OCR0A = data[loop];
	OCR0B = data[loop+(NUM_SERVOS/2)];

}

/**********************************************************************************************//**
 * @fn	void init_all(void)
 *
 * @brief	Initializes all components.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 **************************************************************************************************/

void init_all(void){

	//SET INITIAL SERVO POSITION (1,5ms)
	for (int i=0;i<NUM_SERVOS;i++)
	{
		data[i] = SERVO_STD_VAL;
	}

	init_twi();

	init_outputs();

	init_timer();

	sei();

}

/**********************************************************************************************//**
 * @fn	int main(void)
 *
 * @brief	Main entry-point for this application.
 *
 * @author	Alexander Miller
 * @date	11.08.2016
 *
 * @return	Exit-code for the process - 0 for success, else an error code.
 **************************************************************************************************/

int main(void)
{
	init_all();
	unsigned char data_counter = 0;
	unsigned char init_received = 0;
	unsigned char twi_data = 0;

	while (1)
	{
		//Wait for IC2 communication
		while (!(TWCR & (1<<TWINT))){}


		switch(TWSR){
			case 0x60: //Received own address and write bit, ACK returned
			LED1_ON;
			break;
			case 0x70: //Received general call and write bit, ACK returned
			LED1_ON;
			break;
			case 0x80: //Addressed with own address and data byte received, ACK returned
			
			case 0x90: //Addressed with general call and data byte received, ACK returned
			LED2_ON;
			twi_data = TWDR;
			//wait for 22 which indicates the first servo value
			if (twi_data == 22)
			{
				init_received = 0;
				data_counter = 0;
			}
			//wait for 11 which indicates the last servo value
			if(twi_data == 11){
				init_received = 1;
				data_counter = 0;
			}
			//if 22 was received previously set the received value as servo position
			if (init_received && twi_data != 22 && twi_data != 11)
			{
				data[data_counter] = twi_data;
				data_counter = (data_counter+1)%NUM_SERVOS;
			}

			LED2_OFF;
			break;
			case 0xA0: //Received STOP condition
			LED1_OFF;
			break;
		}
		TWCR |= (1<<TWINT);

	}
}

