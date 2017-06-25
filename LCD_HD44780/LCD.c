/*
 * LCD.c
 *
 * Created: 2017-06-10 16:17:27
 *  Author: Tunachi
 */ 

 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/pgmspace.h>
 #include <avr/eeprom.h>
 #include <stdlib.h>
 #include "LCD.h"

 static void LCD_InitPort(void);
 static inline void LCD_DataDirOut(void);
 static inline void LCD_DataDirIn(void);
 static inline void LCD_SendHalf(uint8_t data);
 static inline uint8_t LCD_ReadHalf(void);
 static uint8_t LCD_ReadByte(void);
 static uint8_t LCD_Check_BF(void);
 static void LCD_WraiteByte(uint8_t data);
 void LCD_WriteCmd(uint8_t cmd);
 void LCD_WriteData(uint8_t data);

 /*------------------------------ Control Pins macro definitions ----------------------*/
 //RS = 1 - command register
 #define SET_RS LCD_PORT |= (1<<LCD_RS)
 //RS = 0 - data register
 #define CLR_RS LCD_PORT &= ~(1<<LCD_RS)

 //RW = 1 - read from command or data register
 #define SET_RW LCD_PORT |= (1<<LCD_RW)
 //RW = 0 - write to command or data register
 #define CLR_RW LCD_PORT &= ~(1<<LCD_RW)

 #define SET_E LCD_PORT |= (1<<LCD_E)
 #define CLR_E LCD_PORT &= ~(1<<LCD_E)
 /*------------------------------------------------------------------------------------*/
 static void LCD_InitPort(void)
 {
	//LCD Pins as outputs
	LCD_DDR |= (1<<LCD_RS) | (1<<LCD_RW) | (1<<LCD_E);
	LCD_DDR |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);

	//Low state on LCD control Pins
	LCD_PORT &= ~((1<<LCD_RS) | (1<<LCD_RW) | (1<<LCD_E));
 }

 static inline void LCD_DataDirOut(void)
 {
	LCD_DDR |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);
 }

 static inline void LCD_DataDirIn(void)
 {
	LCD_DDR &= ~((1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7));
 }

 static inline void LCD_SendHalf(uint8_t data)
 {
	if(data & (1<<0))
		LCD_PORT |= (1<<LCD_D4);
	else
		LCD_PORT &= ~(1<<LCD_D4);
	if(data & (1<<1))
		LCD_PORT |= (1<<LCD_D5);
	else
		LCD_PORT &= ~(1<<LCD_D5);
	if(data & (1<<2))
		LCD_PORT |= (1<<LCD_D6);
	else
		LCD_PORT &= ~(1<<LCD_D6);
	if(data & (1<<3))
		LCD_PORT |= (1<<LCD_D7);
	else
		LCD_PORT &= ~(1<<LCD_D7);
 }

 static inline uint8_t LCD_ReadHalf(void)
 {
	uint8_t result = 0;

	if(LCD_PIN & (1<<LCD_D4)) result |= (1<<0);
	if(LCD_PIN & (1<<LCD_D5)) result |= (1<<1);
	if(LCD_PIN & (1<<LCD_D6)) result |= (1<<2);
	if(LCD_PIN & (1<<LCD_D7)) result |= (1<<3);

	return result;
 }

 static uint8_t LCD_ReadByte(void)
 {
	uint8_t result = 0;

	LCD_DataDirIn();
	SET_RW;

	SET_E;
	result |= LCD_ReadHalf() << 4;
	CLR_E;

	SET_E;
	result |= LCD_ReadHalf();
	CLR_E;

	return result;
 }

 static uint8_t LCD_Check_BF(void)
 {
	CLR_RS;
	return LCD_ReadByte();
 }

 static void LCD_WraiteByte(uint8_t data)
 {
	LCD_DataDirOut();

	CLR_RW;

	SET_E;
	LCD_SendHalf(data >> 4);
	CLR_E;

	SET_E;
	LCD_SendHalf(data);
	CLR_E;

	while(LCD_Check_BF() & (1<<7))
		;
 }

 void LCD_WriteCmd(uint8_t cmd)
 {
	CLR_RS;
	LCD_WraiteByte(cmd);
 }

 void LCD_WriteData(uint8_t data)
 {
	SET_RS;
	LCD_WraiteByte(data);
 }

 void LCD_Init(void)
 {
	LCD_InitPort();

	_delay_ms(40);
	LCD_SendHalf((FUNCTION_SET | INTERFACE_8BIT) >> 4);
	_delay_us(40);
	LCD_SendHalf((FUNCTION_SET | INTERFACE_4BIT) >> 4);
	_delay_us(40);
	LCD_WriteCmd(FUNCTION_SET | INTERFACE_4BIT | DISPLAY_2_LINE | FONT_5X8);
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | CURSOR_OFF | BLINK_CURSOR_OFF);
	LCD_WriteCmd(CLEAR_DISPLAY);
	LCD_WriteCmd(ENTRY_MODE_SET | CURSOR_OR_DISPLAY_DIR_RIGHT | CURSOR_IS_MOVING);
	
 }

 void LCD_Clear(void)
 {
	LCD_WriteCmd(CLEAR_DISPLAY);
 }

 void LCD_Home(void)
 {
	LCD_WriteCmd(CLEAR_DISPLAY | RETURN_HOME);
 }

 void LCD_CursorOn(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | CURSOR_ON);
 }

 void LCD_CursorOff(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | CURSOR_OFF);
 }

 void LCD_CursorBlinkOn(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | BLINK_CURSOR_ON);
 }

 void LCDCursorBlinkOff(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | BLINK_CURSOR_OFF);
 }

 void LCD_Str(char *str)
 {
	register char symbol;

	while((symbol = *(str++)))
		LCD_WriteData(((symbol >= 0x80) && (symbol <= 0x87)) ? (symbol & 0x07) : symbol);
 }

 void LCD_Str_P(const char *str)
 {
	register char symbol;

	while((symbol = pgm_read_byte(str++)))
		LCD_WriteData(((symbol >= 0x80) && (symbol <= 0x87)) ? (symbol & 0x07) : symbol);
 }

 void LCD_Str_E(const char *str)
 {
	register char symbol;

	while(1)
	{
		symbol = eeprom_read_byte((uint8_t *)(str++));
		if(!symbol || symbol == 0xFF)
			break;
		else
			LCD_WriteData(((symbol >= 0x80) && (symbol <= 0x87)) ? (symbol & 0x07) : symbol);
	}
 }

 void LCD_Int(int value)
 {
	char bufor[17];
	LCD_Str(itoa(value, bufor, 10));
 }

 void LCD_Hex(int value)
 {
	char bufor[17];
	LCD_Str("0x");
	LCD_Str(itoa(value, bufor, 16));
 }