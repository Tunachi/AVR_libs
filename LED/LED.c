/*
 * LED.c
 *
 * Created: 2017-06-10 07:23:44
 *  Author: Tunachi
 */
  
#include <avr/io.h>
#include "LED.h"

void LED_Init(void)
{
	LED_DDR |= LED1;
}