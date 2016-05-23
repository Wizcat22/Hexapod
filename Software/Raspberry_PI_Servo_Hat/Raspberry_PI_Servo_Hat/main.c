/*
* Raspberry_PI_Servo_Hat.c
*
* Created: 23.05.2016 17:38:40
* Author : Alexander Miller
*/

//INCLUDE
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>
//------------------------

//DEFINE
#define SLAVE_ADDRESS 0x42 //TWI Slave address
#define NUM_SERVOS 24 // 20 is the recommended maximum value (Servos get slower because of longer standby time (>20ms))
#define SERVO_STD_VAL 94 //STANDARD SERVO POSITION (1,5ms)
//------------------------

//GLOBAL
unsigned volatile char data[NUM_SERVOS]; //CHAR ARRAY WITH SERVO DATA
unsigned volatile char loop = 0; //loop count
//------------------------


//init twi/i2c
void init_twi(void){

	//Set address of i2c slave
	TWAR |= (SLAVE_ADDRESS<<1) | (1<<TWGCE);
	TWCR |= (1<<TWEN) | (1<<TWEA);
	TWCR &= ~((1<<TWSTA) | (1<<TWSTO)) ;

}
//------------------------

//INIT OUTPUTS
void init_outputs(void){

	DDRA = 0xFF; //Servo Outputs: PA0=SERVO_19 - PA7 = SERVO_12
	DDRB = 0x1F; //Servo Outputs: PB0=SERVO_20 - PB4 = SERVO_24
	DDRC = 0xFC; //Servo Outputs: PC2=SERVO_6 - PC7 = SERVO_11
	DDRD = 0xFF; //Servo Outputs: PD3=SERVO_1 - PD7 = SERVO_5 ; LED Outputs: PD0 = LED_1 - PD2 = LED_3

}
//------------------------


//INIT TIMER
void init_timer(void){

	//TIMER0 (8bit) , Mode 3 - Fast PWM TOP = 0xFF ,Prescaler = 256 , Time per overflow = 0.004096 sec. = 4.096 ms (16Mhz Clock)
	TCCR0A |= (1<<WGM01) | (1<<WGM00);
	TCCR0B |= (1<<CS02);

	//Enable TIMER0 Interrupts (Compare Match A/B and Overflow)
	TIMSK0 |= (1<<OCIE0A) | (1<<OCIE0B) | (1<<TOIE0);
}
//------------------------

//ISR OC0A -> RESET SERVOS 1 - 12
ISR(TIMER0_COMPA_vect){

	PORTD &= ~(0xF8) ;
	PORTC &= ~(0xFC) ;
	PORTA &= ~(0x80) ;

}
//------------------------

//ISR OC0A -> RESET SERVOS 13 - 24
ISR(TIMER0_COMPB_vect){

	PORTA &= ~(0x7F) ;
	PORTB &= ~(0x1F) ;

}
//------------------------

//ISR TIOV -> SET SERVO OUTPUTS AND SET CORRESPONDING OC0A/OC0B VALUES
ISR(TIMER0_OVF_vect){

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
//------------------------

//INIT ALL 
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
//------------------------

//MAIN
int main(void)
{
	init_all();
	int data_counter = 0;


	//main -> get data (twi)
	while (1)
	{
		while (!(TWCR & (1<<TWINT))){}

		switch(TWSR){
			case 0x60: //Received own address and write bit, ACK returned
			break;
			case 0x70: //Received general call and write bit, ACK returned
			break;
			case 0x80: //Addressed with own address and data byte received, ACK returned
			data[data_counter] = TWDR;
			data_counter = (data_counter+1)%NUM_SERVOS;
			break;
			case 0x90: //Addressed with general call and data byte received, ACK returned
			data[data_counter] = TWDR;
			data_counter = (data_counter+1)%NUM_SERVOS;
			break;
		}
		
		//TWCR &= ~(1<<TWINT);
		TWCR |= (1<<TWINT); //reset von TWINT durch setzen????
		
		




	}
}
//------------------------
