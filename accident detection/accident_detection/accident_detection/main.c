/*
 * accident_detection.c
 *
 * Created: 06-12-2021 21:07:23
 * Author : Dell
 */ 

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>

/*LCD functions declarations and pins on PORTB*/

#define CMD     0
#define DATA    1
#define RS      PB0
#define EN      PB1
#define LCD_DIR DDRB
#define LCD_OUT PORTB

void LCD_delay(uint8_t n);
void LCD_pulse(void);
void LCD_send_info(uint8_t, uint8_t);
void LCD_init(void);
void LCD_clear();
void LCD_send_string(uint8_t *);
void LCD_set_cursor(uint8_t, uint8_t);
void LCD_createChar(uint8_t, uint8_t *);

/**********************************************/

/*UART functions declarations*/

uint16_t BR = 51;

void UART_Tx_Init();
void UART_Tx(uint8_t);

void UART_Rx_Init(void);
uint8_t UART_Rx(void);

/*****************************/

/*extract data functions declarations*/

uint8_t lat[15] = "";
uint8_t lon[15] = "";
uint8_t GPRMC[100] = "";

void extract_lat_lon(void);

void display_lat_lon(uint8_t);

/*************************************/

/*select functions declarations*/

#define GSM 0
#define GPS 1

void select_init();
void select_gsm_gps(uint8_t select);

/*******************************/

/*GSM send message*/

void send_msg_GSM();

void display_msg_LCD(uint8_t);

/*****************/

void check_accident();

int main(void)
{
	LCD_init();
	
	UART_Rx_Init();
	
	UART_Tx_Init();
	
	select_init();
	
	DDRA &= ~(1<<PA0);  /*setting accident detect pin as input*/
	
	while(1)
	{	
		check_accident();    /*detecting accident*/
		
		_delay_ms(1500);
		
		LCD_clear();
		
		select_gsm_gps(GPS); /*selecting GSM*/
		
		extract_lat_lon();   /*getting current location and display it on LCD*/
		
		_delay_ms(1500);
		
		select_gsm_gps(GSM); /*selecting GSM*/
		
		LCD_clear();
		
		send_msg_GSM();      /*sending alert message using GSM module*/
		
		_delay_ms(1500);
		
		LCD_clear();
	}
}

/*UART functions definitions*/

void UART_Rx_Init(void)
{
	UCSRB |= (1<<RXEN);                 /*enable the receiver*/
	
	UCSRC |= (1<<URSEL);                /*selecting UCSRC register*/
	
	UCSRC |= ((1<<UCSZ0) + (1<<UCSZ1)); /*8 bit character size*/
	
	UBRRL = 51;
}

void UART_Tx_Init()
{	
	UCSRB |= (1<<TXEN); /*Tx enable*/
	
	UCSRC |= (1<<URSEL); /*selecting UCSRC*/
	
	UCSRC |= ((1<<UCSZ0) + (1<<UCSZ1)); /*8bit char size*/
	
	UCSRC &= ~(1<<UCPOL); /*clock polarity*/
	
	UBRRH &= ~(1<<URSEL); /*selecting UBRRH*/
	
	UBRRH &= 0xF0;
	UBRRL = (uint8_t) (BR); /*9600 baud rate*/
}

uint8_t UART_Rx(void)
{
	while((UCSRA & (1<<RXC))==0x00);    /*wait till all data is received*/
	
	return UDR;
}

void UART_Tx(uint8_t data)
{
	while(1)
	{
		if((UCSRA & (1<<UDRE))==0x00)
		{
			continue;
		}
		else
		{
			break;
		}
	}
	
	UDR = data;
}

/****************************/


/*LCD functions definition*/

void LCD_delay(uint8_t n)
{
	uint8_t k=0;

	for(k=0; k<n; k++)
	{
		_delay_ms(1);
	}
}

void LCD_pulse(void)
{
	LCD_OUT |= (1<<EN);

	LCD_delay(1);

	LCD_OUT &= ~(1<<EN);

	LCD_delay(1);
}

void LCD_send_info(uint8_t info, uint8_t mode)
{
	if(mode==DATA)
	{
		LCD_OUT |=  (1<<RS);
	}
	else if(mode==CMD)
	{
		LCD_OUT &= ~(1<<RS);
	}
	else
	{
		/*do nothing*/
	}

	uint8_t low_nibble   =  (info<<4) & 0xf0 ;
	uint8_t high_nibble  =   info & 0xf0;

	LCD_OUT &= 0x0f; /*clear upper bits*/

	LCD_OUT |= high_nibble;
	LCD_pulse();
	LCD_delay(1);

	LCD_OUT &= 0x0f; /*clear upper bits*/

	LCD_OUT |= low_nibble;
	LCD_pulse();
	LCD_delay(1);
}

void LCD_init(void)
{
	LCD_DIR |=  0xff;
	LCD_OUT &= ~0xff;

	LCD_delay(150);

	LCD_send_info(0x33,CMD);
	LCD_delay(50);

	LCD_send_info(0x32,CMD);
	LCD_delay(1);

	LCD_send_info(0x28,CMD);
	LCD_delay(1);

	LCD_send_info(0x01,CMD);
	LCD_delay(20);

	LCD_send_info(0x06,CMD);
	LCD_delay(1);

	LCD_send_info(0x80,CMD);
	LCD_delay(1);

	LCD_send_info(0x0C,CMD);
	LCD_delay(1);
}

void LCD_send_string(uint8_t *s)
{
	while(*s)
	{
		LCD_send_info(*s, DATA);
		s++;
	}

}

/*r and c are 0 indexed*/

void LCD_set_cursor(uint8_t r, uint8_t c)
{
	if(r==0)
	{
		LCD_send_info(0x80+c, CMD);
	}
	else if(r==1)
	{
		LCD_send_info(0xC0+c, CMD);
	}
	else if(r==2)
	{
		LCD_send_info(0x94+c, CMD);
	}
	else if(r==3)
	{
		LCD_send_info(0xD4+c, CMD);
	}
	else
	{
		/*do nothing*/
	}

}

void LCD_clear()
{
	LCD_send_info(0x01, CMD);

	LCD_delay(20);
}

void LCD_createChar(uint8_t location, uint8_t charmap[])
{
	LCD_send_info(0x40 + location*8, CMD);

	uint8_t i = 0;
	for (i=0; i<8; i++)
	{
		LCD_send_info(charmap[i], DATA);
	}
}

/**************************/

void select_init()
{
	DDRB |= (1<<PB2);	
}

void select_gsm_gps(uint8_t select)
{
	if(select==GPS)
	{
		PORTB |=  (1<<PB2);
	}
	else if(select==GSM)
	{
		PORTB &= ~(1<<PB2);
	}
	else
	{
		/*do nothing*/
	}
}

/*extract data functions definitions*/

void extract_lat_lon(void)
{	
	uint8_t uart_data = 0, i=0, j=0, k=0;
	uint8_t compare[10] = "", l=0;
	
	display_lat_lon('g');
	
	while(1)
	{
		uart_data = UART_Rx();
		
		if(uart_data=='$')
		{
			uart_data = UART_Rx();
			
			while(uart_data!='$')
			{			
				GPRMC[i] = uart_data;
				
				i++;
					
				uart_data = UART_Rx();
			}
			
			break;
		}
		else
		{
			/*do nothing*/
		}
	}
	
	for(l=0; l<=4; l++)
	{
		compare[l] = GPRMC[l]; 
	}
	
	if(strcmp(compare,"GPRMC")==0)
	{
		for(j=19,k=0 ; j<=29 ; j++, k++)
		{
			lat[k] = GPRMC[j];
		}
	
		for(j=31,k=0 ; j<=42 ; j++, k++)
		{
			lon[k] = GPRMC[j];
		}			
	}
	
	display_lat_lon('c');
}

void display_lat_lon(uint8_t opt)
{
	if(opt=='g')
	{
		LCD_set_cursor(0,0);
		LCD_send_string("  GETTING LOCATION  ");
	
		LCD_set_cursor(1,6);
		LCD_send_string("FROM GPS");		
	}
	else if(opt=='c')
	{
		LCD_set_cursor(2,0);
		LCD_send_string("Lat. : ");
		
		LCD_send_string(lat);
		
		LCD_set_cursor(3,0);
		LCD_send_string("Lon. : ");
		
		LCD_send_string(lon);	
	}
	else
	{
		/*do nothing*/
	}

}

/************************************/

/*GSM send message*/

void send_msg_GSM()
{
	display_msg_LCD('s');
	
	uint8_t cmd1[] = "AT+CMGF=1";
	uint8_t cmd2[] = "AT+CMGS=\"+917765814478\"";
	uint8_t msg[50] = "Accident happens at ";
	
	strcat(msg,lat);
	strcat(msg,lon);
	
	uint8_t i=0;
	
	for(i=0; i<sizeof(cmd1); i++)
	{
		UART_Tx(cmd1[i]);
	}
	
	_delay_ms(500);
	
	for(i=0; i<sizeof(cmd2); i++)
	{
		UART_Tx(cmd2[i]);
	}
	
	_delay_ms(500);
	
	for(i=0; i<sizeof(msg); i++)
	{
		UART_Tx(msg[i]);
	}	
	
	display_msg_LCD('d');
}

void display_msg_LCD(uint8_t opt)
{
	if(opt=='s')
	{
		LCD_set_cursor(0,0);
		
		LCD_send_string("  SENDING MESSAGE   ");
	}
	else if(opt=='d')
	{
		LCD_set_cursor(0,0);
		
		LCD_send_string("    MESSAGE SENT    ");
	}
	else
	{
		/*do nothing*/
	}
	
	LCD_set_cursor(2,0);
	LCD_send_string("PHONE NO. :");
	
	LCD_set_cursor(3,0);
	LCD_send_string("+9177658144478");
}

/******************/

void check_accident()
{
	while((PINA & (1<<PA0))==0x00)  /*accident is not detected*/
	{
		LCD_set_cursor(0,0);
		LCD_send_string(" ACCIDENT DETECTION ");
		
		LCD_set_cursor(1,0);
		LCD_send_string("       SYSTEM       ");
		
		LCD_set_cursor(3,0);
		LCD_send_string("NO ACCIDENT DETECTED");
	}
	
	LCD_set_cursor(3,0);
	LCD_send_string("  ACCIDENT DETECTED ");		
}