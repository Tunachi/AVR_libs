/*
 * Switch.h
 *
 * Created: 2017-06-10 10:12:39
 *  Author: Tunachi
 */ 


#ifndef SWITCH_H_
#define SWITCH_H_

#define BUTTON_1 (1<<PD5)
#define BUTTON_2 (1<<PD4)
#define BUTTON_3 (1<<PD3)
#define BUTTON_PORT PORTD
#define BUTTON_PIN PIND
#define BUTTON_DDR DDRD
#define BUTTON_1_DOWN !(BUTTON_PIN & BUTTON_1)
#define BUTTON_2_DOWN !(BUTTON_PIN & BUTTON_2)
#define BUTTON_3_DOWN !(BUTTON_PIN & BUTTON_3)

void Switch_Init(void);
uint8_t Switch_Dawn(void);
uint8_t Switch_Pressed(uint8_t button);

#endif /* SWITCH_H_ */