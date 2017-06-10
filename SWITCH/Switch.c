/*
 * Switch.c
 *
 * Created: 2017-06-10 10:12:18
 *  Author: Tunachi
 */ 
 #include <avr/io.h>
 #include <util/delay.h>
  #include "../LED/LED.h"
 #include "Switch.h"

 void Switch_Init(void)
 {
	BUTTON_PORT |= BUTTON_1;
	BUTTON_DDR &= ~BUTTON_1;

	BUTTON_PORT |= BUTTON_2;
	BUTTON_DDR &= ~BUTTON_2;

	BUTTON_PORT |= BUTTON_3;
	BUTTON_DDR &= ~BUTTON_3;
 }

uint8_t Switch_Dawn(void)
 {
	if(BUTTON_1_DOWN)
	{
		_delay_ms(80);
		if(BUTTON_1_DOWN)
			return 1;
	}
	return 0;
 }

uint8_t Switch_Pressed(uint8_t button)
{
	if(!(BUTTON_PIN & button))
	{
		_delay_ms(80);
		if(!(BUTTON_PIN & button))
			return 1;
	}
	return 0;
}




