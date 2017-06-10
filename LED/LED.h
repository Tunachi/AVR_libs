/*
 * LED.h
 *
 * Created: 2017-06-10 07:24:13
 *  Author: Tunachi
 */ 


#ifndef LED_H_
#define LED_H_

#define LED_DIR 0	//1 oznacza stan wysoki z uC do zapalenia diody

#define LED1 (1<<PD6)
#define LED_PORT PORTD
#define LED_DDR DDRD

#if LED_DIR == 1
#define LED_ON  LED_PORT |= LED1
#define LED_OFF LED_PORT &= ~LED1
#define LED_TOG LED_PORT ^= LED1
#else
#define LED_ON  LED_PORT &= ~LED1
#define LED_OFF LED_PORT |= LED1 
#define LED_TOG LED_PORT ^= LED1
#endif


void LED_Init(void);

#endif /* LED_H_ */