/*
* ATXMEGA32A4U.c
*
* Created: 05.03.2017 12:33:01
*  Author: Alexander Miller
*/

#define F_CPU 16000000UL

#pragma region INCLUDES

#include <avr/io.h>
#include <util/delay.h>
#include "../include/ATXMEGA32A4U.h"
#include "../include/INA3221.h"

#pragma endregion INCLUDES

#pragma region VARIABLES

uint8_t slave_address = 0x11; //I2C SLAVE ADDRESS

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
WDT.CTRL = WDT_PER_2KCLK_gc | WDT_ENABLE_bm | WDT_CEN_bm;
/* Wait for WD to synchronize with new settings. */
while(WDT.STATUS & WDT_SYNCBUSY_bm ){
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
	TCD0.PER = 40000; //Set Timer0 top value
	TCD0.CTRLA = TC0_CLKSEL2_bm; //Set Timer0 clock source and prescaler
	TCD0.CTRLB = TC0_WGMODE0_bm | TC0_WGMODE1_bm |TC0_CCAEN_bm| TC0_CCBEN_bm| TC0_CCCEN_bm; //Enable singleslope mode and enable pins OC0A - OC0C for pwm

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
uart_send('\n');
}

void uart_send_number(int32_t num){

char str[20];
sprintf(str, "%d", num);
uart_send_string(str);

}

void led_set_color(uint16_t H, float S, float V){

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

	if ((TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm))
	{
		if (TWIC_SLAVE_DATA == (slave_address<<1))
		{
			int8_t s0 = 0;
			int8_t s1 = 0;
			int8_t s2 = 0;


			TWIC_SLAVE_CTRLB = 0b00000011;
			while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
			switch(TWIC_SLAVE_DATA)
			{
				case 1:
				TWIC_SLAVE_CTRLB = 0b00000011;
				while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
				led_set_color(TWIC_SLAVE_DATA,1,0.005f);
				TWIC_SLAVE_CTRLB = 0b00000010;
				break;
				case 2:
				TWIC_SLAVE_CTRLB = 0b00000011;
				while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
				s0 = TWIC_SLAVE_DATA;
				TWIC_SLAVE_CTRLB = 0b00000011;
				while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
				s1 = TWIC_SLAVE_DATA;
				TWIC_SLAVE_CTRLB = 0b00000011;
				while (!(TWIC_SLAVE_STATUS & TWI_SLAVE_CLKHOLD_bm)){}
				s2 = TWIC_SLAVE_DATA;
				TWIC_SLAVE_CTRLB = 0b00000010;
				servo_set_position(s0,s1,s2);


			}
		}
		
	}


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

void servo_set_position(int8_t s0, int8_t s1, int8_t s2){
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