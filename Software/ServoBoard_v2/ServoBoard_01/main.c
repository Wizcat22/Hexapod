/*
* ServoBoard_01.c
*
* Created: 14.04.2016 13:38:01
* Author : Alexander Miller
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/twi.h>

#define SLAVE_ADDRESS 0x42 //TWI Slave address
#define NUM_SERVOS 10//Warning best operation at 10 servos or less, 20 is the recommended maximum value (Servos get slower because of longer standby time (>20ms))
//char array with data for each pin, default = 94 ~ 1,5ms
unsigned volatile char data[NUM_SERVOS];
//loop count
unsigned volatile char loop = 0;

//init twi/i2c
void init_twi(void){

	//Set address of i2c slave
	TWAR |= (SLAVE_ADDRESS<<1) | (1<<TWGCE);
	TWCR |= (1<<TWEN) | (1<<TWEA);
	TWCR &= ~((1<<TWSTA) | (1<<TWSTO)) ;

}

//init pins
void init_pins(void){

	//Pins PD0 - PD7 set as output for servos
	DDRD = 0xFF;


}


//init timer
void init_timer(void){

	//TIMER0 (8bit) , Mode 3 - Fast PWM TOP = 0xFF ,Prescaler = 256 , Time per overflow = 0.004096 sec. = 4.096 ms (16Mhz Clock)
	TCCR0A |= (1<<WGM01) | (1<<WGM00);
	TCCR0B |= (1<<CS02);

	//Enable TIMER0 Interrupts (Compare Match A/B and Overflow)
	TIMSK0 |= (1<<OCIE0A) | (1<<OCIE0B) | (1<<TOIE0);
}


//isr oc0a -> set pins  1-10 to 0
ISR(TIMER0_COMPA_vect){

	PORTD &= ~(0xF0) ;

}
//isr oc0b -> set pins 10-20 to 0
ISR(TIMER0_COMPB_vect){

	PORTD &= ~(0x0F) ;

}
//isr tiov -> set pins  1-20 to 1 and set oc0a/oc0b values
ISR(TIMER0_OVF_vect){

	switch (loop)
	{
		case 0:
		PORTD |= 0b00010001;
		break;
		case 1:
		PORTD |= 0b00100010;
		break;
		case 2:
		PORTD |= 0b01000100;
		break;
		case 3:
		PORTD |= 0b10001000;
		break;
		default: break;
	}
	loop = (loop+1)%(NUM_SERVOS/2);
	OCR0A = data[loop];
	OCR0B = data[loop+(NUM_SERVOS/2)];

}

//init all
void init_all(void){

	for (int i=0;i<NUM_SERVOS;i++)
	{
		if (i%2)
		{
			data[i] = 94;
		}
		else
		{
			data[i] = 64;
		}
		
	}

	init_twi();

	init_pins();

	init_timer();

	sei();


}



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

