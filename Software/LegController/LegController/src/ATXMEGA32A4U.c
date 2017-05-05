/*
* ATXMEGA32A4U.c
*
* Created: 05.03.2017 12:33:01
*  Author: Alexander Miller
*/

#pragma region DEFINES

#define F_CPU 16000000UL

#define LED_SATURATION 1.0f
#define LED_BRIGTHNESS 0.005f

#define C_RED 0
#define C_YELLOW 60
#define C_GREEN 120
#define C_CYAN 180
#define C_BLUE 240
#define C_MAGENTA 300

#define HEIGHT 88
#define A1 52
#define A2 69
#define A3 88

#pragma endregion DEFINES



#pragma region INCLUDES

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include "../include/ATXMEGA32A4U.h"
#include "../include/INA3221.h"

#pragma endregion INCLUDES

#pragma region VARIABLES

uint8_t slave_address = 0x13; //I2C SLAVE ADDRESS

int8_t servo_cal[] = {0,0,0};

int8_t lastAlpha = 0;
int8_t lastBeta = 0;
int8_t lastGamma = 0;

#pragma endregion VARIABLES

#pragma region FUNCTIONS

void init_system_clock(void){
	OSC.XOSCCTRL = OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_XTAL_16KCLK_gc; //Set external oscillator frequency range and select external oscillator incl. start-up time
	OSC.CTRL |= OSC_XOSCEN_bm; //Enable external oscillator as clock source
	while (!(OSC_STATUS & OSC_XOSCRDY_bm)){} //Wait until the external clock is ready
	CCP = CCP_IOREG_gc; //Disable interrupts for 4 clock cycles and protect I/O
	CLK.CTRL = CLK_SCLKSEL_XOSC_gc; //Select the external oscillator as clock source

}

void init_pll(void){

	//NOT WORKING??
	OSC.PLLCTRL = OSC_PLLSRC_XOSC_gc | OSC_PLLFAC1_bm; //Set PLL clock reference (external osc) and multiplication factor (2x)
	OSC.CTRL |= OSC_PLLEN_bm; // Enable PLL
	while (!(OSC.STATUS & OSC_PLLRDY_bm)){}
	CCP = CCP_IOREG_gc;
	CLK.CTRL = CLK_SCLKSEL_PLL_gc;

}

void init_watchdog(void){

	CCP = CCP_IOREG_gc; //Disable interrupts for 4 clock cycles and protect I/O
	WDT.CTRL = WDT_PER_8KCLK_gc | WDT_ENABLE_bm | WDT_CEN_bm; //Enable watchdog and set timeout to 8 sek. @ 3.3V
	while(WDT.STATUS & WDT_SYNCBUSY_bm ){} //Wait for WD to synchronize with new settings

}

void init_eeprom(void){

	for (uint8_t p=0;p<3;p++)
	{
		uint8_t temp = eeprom_read_byte((uint8_t *) p);
		if (temp != 0xFF)
		{
			servo_cal[p] = temp ;
		}
	}



}

void init_gpio(void){

	//Servo
	PORTD.DIR |= (1<<0)|(1<<1)|(1<<2);

	//LED
	//PORTC.DIR |= (1<<4)|(1<<5);
	PORTC.REMAP = PORT_TC0A_bm | PORT_TC0B_bm | PORT_TC0C_bm;
	PORTC.DIR |= (1<<4)|(1<<5)|(1<<6);

	//USART
	PORTC.OUT |= (1<<3);
	PORTC.DIR |= (1<<3);

	//ADDR PINS
	PORTD.DIR &= ~(1<<3) | ~(1<<4) | ~(1<<5);
	PORTD.PIN3CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN4CTRL = PORT_OPC_PULLUP_gc;
	PORTD.PIN5CTRL = PORT_OPC_PULLUP_gc;

}

void init_servo(void){
	//Init timer0 (16bit)
	TCD0.PER = 40000; //Set Timer0 top value (16Mhz & 8 prescaler = 20ms)
	TCD0.CTRLA = TC0_CLKSEL2_bm; //Set Timer0 clock source and prescaler (8)
	TCD0.CTRLB = TC0_WGMODE0_bm | TC0_WGMODE1_bm |TC0_CCAEN_bm| TC0_CCBEN_bm| TC0_CCCEN_bm; //Enable singleslope mode and enable pins OC0A - OC0C for pwm

	servo_set_deg(0,0,0);

}

void init_twiE_MASTER(void){

	TWIE_MASTER_BAUD = (F_CPU / (2* F_TWI_HS)) - 5; //SET TWI_E BAUD
	TWIE_MASTER_CTRLA = TWI_MASTER_ENABLE_bm; //ENABLE TWI_E MASTER
	TWIE_MASTER_STATUS = TWI_MASTER_BUSSTATE_IDLE_gc; //SET TWI_E STATUS TO IDLE


}

void init_twiC_SLAVE(void){

	//slave_address = ((PORTD.IN & 0x38) << 1); //Get ADDR
	TWIC_SLAVE_ADDR = (slave_address<<1); //Set Slave ADDR
	TWIC_SLAVE_CTRLA = TWI_SLAVE_ENABLE_bm; //ENABLE TWI_C SLAVE

}

void init_UART(void){

	////BAUD=2000000 , CLK=32000000 , BSCALE=0 , BSEL=0 , CHARSIZE=8bit
	//
	//
	//USARTC0.CTRLC = USART_CHSIZE_8BIT_gc; //SET CHARSIZE
	//USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm ; //Enable TX/RX



	//BAUD=19200 , CLK=16000000 , BSCALE=2 , BSEL=12 , CHARSIZE=8bit

	USARTC0.BAUDCTRLA = 12; //SET BSEL
	USARTC0.BAUDCTRLB = USART_BSCALE1_bm; //SET BSCALE
	USARTC0.CTRLC = USART_CHSIZE_8BIT_gc; //SET CHARSIZE
	USARTC0.CTRLB = USART_TXEN_bm | USART_RXEN_bm ; //Enable TX/RX

}

void uart_send(char data){

	while (!(USARTC0_STATUS & USART_DREIF_bm)){} //Wait until DATA-Register is empty
	USARTC0.DATA = data; //Send data

}

void uart_send_word(uint16_t data){

	uart_send(data>>8); //send high-byte
	uart_send(data); // send low-byte
	uart_send('\n');
}

void uart_send_string(char s[]){
	int x =0;
	while (s[x] != '\0')
	{
		uart_send(s[x]);
		x++;
	}
	//uart_send('\n');
}

void uart_send_number(int32_t num){

	char str[20];
	sprintf(str, "%d", num);
	uart_send_string(str);

}

void led_set_color(uint16_t H, float S, float V){

	H = H%360;

	float R = 0.0;
	float G = 0.0;
	float B = 0.0;

	uint8_t h = H / 60;
	float f = ((float)H/60 - h);
	float p = V * (1-S);
	float q = V * (1-S*f);
	float t = V * (1-S*(1-f));

	switch(h){

		case 0: R=V; G=t; B=p;
		break;
		case 1: R=q; G=V; B=p;
		break;
		case 2: R=p; G=V; B=t;
		break;
		case 3: R=p; G=q; B=V;
		break;
		case 4: R=t; G=p; B=V;
		break;
		case 5: R=V; G=p; B=q;
		break;
		case 6: R=V; G=t; B=p;
		break;
		default: R=0; G=0; B=0;

	}


	TCC0.CCABUF = (uint16_t)(16000*R);
	TCC0.CCBBUF = (uint16_t)(16000*G);
	TCC0.CCCBUF = (uint16_t)(16000*B);
	//TCC0.CTRLFSET |= (1<<3);
	



}

void init_LED(void){

	//Init timerC0 (16bit), PRESCALER=8, FREQUENCY=125Hz
	TCC0.PER = 16000;
	TCC0.CTRLA = TC0_CLKSEL2_bm;; //PRESCALER=8
	TCC0.CTRLB = TC0_WGMODE0_bm | TC0_WGMODE1_bm | TC0_CCAEN_bm | TC0_CCBEN_bm | TC0_CCCEN_bm; //SINGLESLOPE AND CHANNELS A,B and C ENABLED

	led_set_color(60*((PORTD.IN & 0x38)>>3)-60,1,0.005);

}

void twi_slave_get_data(void){

	uint8_t data_byte[] = {0,0,0};
	uint16_t data_word[] = {0,0,0};

	if ((TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){ //If transaction happened
		if (TWIC_SLAVE_DATA == (slave_address<<1)) //If the received address is correct (should be always the case)
		{
			TWIC_SLAVE_CTRLB = 0b00000011; //Send ack
			data_byte[0]= twi_slave_get_byte(); //Get "Register"-address

			//0 = init
			//1 = Set led color
			//2 = Set servo degree
			//3 = Set leg position
			//4 = Set servo calibration value and save in eeprom 
			//5 = Reset




			switch (data_byte[0])
			{
				case 0: //Init
				led_set_color(C_GREEN,LED_SATURATION,LED_BRIGTHNESS);
				break;
				case 1: //Set led color
				data_word[0] = twi_slave_get_word(); //Get color-value
				led_set_color(data_word[0],LED_SATURATION,LED_BRIGTHNESS);
				break;
				case 2: //2 = Set servo degree
				data_byte[0] = twi_slave_get_byte(); //Servo 0 position
				data_byte[1] = twi_slave_get_byte(); //Servo 1 position
				data_byte[2] = twi_slave_get_byte(); //Servo 2 position
				servo_set_deg(data_byte[0],data_byte[1],data_byte[2]);
				break;
				case 3: //3 = Set leg position
				data_byte[0] = twi_slave_get_byte(); //Servo 0 position
				data_byte[1] = twi_slave_get_byte(); //Servo 1 position
				data_byte[2] = twi_slave_get_byte(); //Servo 2 position
				leg_set_position(data_byte[0],data_byte[1],data_byte[2]);
				break;
				case 4: //4 = Set servo calibration value and save in eeprom
				for (char i=0; i<3;i++)
				{
					data_byte[i] = twi_slave_get_byte(); //Servo calibration value
					eeprom_write_byte((uint8_t *) i,data_byte[i]); //Save servo calibration value to eeprom
					servo_cal[i] = data_byte[i]; //Set servo calibration value
				}
				break;
				case 5: //5 = Reset
				TWIC_SLAVE_CTRLB = 0b00000010; //Send ack
				while (1){} // Wait until Watchdog-reset
				break;
				//case value:
				///* Your code here */
				//break;
				default:
				/* Your code here */
				break;
			}
			TWIC_SLAVE_CTRLB = 0b00000010; //Send ack

		}



	}



}


uint8_t twi_slave_get_byte(void){

	uint8_t data = 0;
	while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
	data = TWIC_SLAVE_DATA;
	TWIC_SLAVE_CTRLB = 0b00000011; //Send ack
	return data;

}

uint16_t twi_slave_get_word(void){

	uint16_t data = 0;
	uint8_t high = 0;
	uint8_t low = 0;

	
	high = twi_slave_get_byte();
	low = twi_slave_get_byte();

	data = (low + (high<<8));
	return data;

}

void twi_master_send_data(char reg,uint16_t data){

	TWIE_MASTER_ADDR = (INA3221_ADD << 1) + 0 ; //Address with write-bit
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_DATA = reg; //Register writen to
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_DATA = (data >> 8); //HIGH-Byte
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_DATA = (data & 0xFF); //LOW-Byte
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_CTRLC |= TWI_MASTER_CMD_STOP_gc; //Issue STOP-condition
}

int16_t twi_master_read_data(char reg){

	uint8_t high = 0;
	uint8_t low = 0;

	TWIE_MASTER_ADDR = (INA3221_ADD << 1) + 0 ; //Address with write-bit
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_DATA = reg; //Register read from
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	TWIE_MASTER_ADDR = (INA3221_ADD << 1) + 1 ; //Address with read-bit
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	high = TWIE_MASTER_DATA;
	TWIE_MASTER_CTRLC |= TWI_MASTER_CMD_RECVTRANS_gc;
	while(!(TWIE_MASTER_STATUS & (TWI_MASTER_WIF_bm | TWI_MASTER_CLKHOLD_bm))){}
	low = TWIE_MASTER_DATA;
	TWIE_MASTER_CTRLC |= TWI_MASTER_CMD_STOP_gc; //Issue STOP-condition

	return (low + (high<<8));
}

void leg_set_position(int8_t xPos, int8_t yPos, int8_t zPos){ // -127 - 127
	
	float a = 0.0f; //Alpha
	float b = 0.0f; //Beta
	float c = 0.0f; //Gamma

	int8_t alpha = 0;
	int8_t beta = 0;
	int8_t gamma = 0;

	float l1 = 0.0f;
	float l2 = 0.0f;
	float l3 = 0.0f;
	
	//ALPHA
	a = atan2(-xPos, A1 + A2 - yPos);

	//BETA
	l1 = HEIGHT - zPos;
	l2 = A2 - yPos;
	l3 = sqrt(l1 * l1 + l2 * l2);

	b = acos(l1 / l3);

	b = b + acos((A2 * A2 - A3 * A3 + l3 * l3) / (2 * A2 * l3));

	//GAMMA
	c = acos((A3 * A3 - l3 * l3 + A2 * A2) / (2 * A3 * A2));

	//RAD TO DEG


	alpha = (int8_t)(a * 180 / M_PI);
	beta = (int8_t)(b * 180 / M_PI - 90);
	gamma = (int8_t)(c * 180 / M_PI - 90);

	if ((alpha == 0 && beta == 0 && gamma == 0)&& (xPos != 0 && yPos != 0 && zPos != 0))
	{
		servo_set_deg(lastGamma,lastBeta,lastAlpha);
	}
	else{
	servo_set_deg(gamma,beta,alpha);
	lastAlpha = alpha;
	lastBeta = beta;
	lastGamma = gamma;

	}

	
}



void servo_set_deg(int8_t s0, int8_t s1, int8_t s2){ // -90 - 90
	

	s0=s0+servo_cal[0];
	s1=s1+servo_cal[1];
	s2=s2+servo_cal[2];

	//Check if values are in range
	if (s0 < -90)
	{
		s0 = -90;
	}
	else if (s0 > 90)
	{
		s0 = 90;
	}

	if (s1 < -90)
	{
		s1 = -90;
	}
	else if (s1 > 90)
	{
		s1 = 90;
	}

	if (s2 < -90)
	{
		s2 = -90;
	}
	else if (s2 > 90)
	{
		s2 = 90;
	}


	TCD0.CCABUF = (uint16_t)(22.22222222222 * s0 + 3000);
	TCD0.CCBBUF = (uint16_t)(22.22222222222 * s1 + 3000);
	TCD0.CCCBUF = (uint16_t)(22.22222222222 * s2 + 3000);
}

void delay(int ms){
	for (int i=0;i<ms;i++)
	{
		_delay_ms(1);
	}
}

#pragma endregion FUNCTIONS