/*
 * LCD.c
 *
 * Created: 2017-06-10 16:17:27
 *  Author: Tunachi
 */ 

 /*****************************************************************************
                  Include files
*****************************************************************************/
 #include <avr/io.h>
 #include <util/delay.h>
 #include <avr/pgmspace.h>
 #include <avr/eeprom.h>
 #include <stdlib.h>
 #include "LCD.h"
 /*****************************************************************************
                  Local types, enums definitions
*****************************************************************************/


/*****************************************************************************
                  Local symbolic constants
*****************************************************************************/
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
 /*****************************************************************************
                  Exported object definitions
*****************************************************************************/


/*****************************************************************************
                  Local object definitions
*****************************************************************************/


/*****************************************************************************
                  Local defined macros
*****************************************************************************/


/*****************************************************************************
                  Local function prototypes
*****************************************************************************/
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
 /*****************************************************************************
                  Function definitions
*****************************************************************************/

/*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_InitPort
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_InitPort initialize LCD pins and set low state on LCD control pins
*
*---------------------------------------------------------------------------*/
 static void LCD_InitPort(void)
 {
	//LCD Pins as outputs
	LCD_DDR |= (1<<LCD_RS) | (1<<LCD_RW) | (1<<LCD_E);
	LCD_DDR |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);

	//Low state on LCD control Pins
	LCD_PORT &= ~((1<<LCD_RS) | (1<<LCD_RW) | (1<<LCD_E));
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_DataDirOut
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_DataDirOut set data pins(D4..D7) direction as output
*
*---------------------------------------------------------------------------*/
 static inline void LCD_DataDirOut(void)
 {
	LCD_DDR |= (1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_DataDirIn
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_DataDirIn set data pins(D4..D7) direction as input
*
*---------------------------------------------------------------------------*/
 static inline void LCD_DataDirIn(void)
 {
	LCD_DDR &= ~((1<<LCD_D4) | (1<<LCD_D5) | (1<<LCD_D6) | (1<<LCD_D7));
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_SendHalf
*
* FUNCTION ARGUMENTS:
*    data - value that will be send to LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_SendHalf set data pins to send four bits to the LCD
*
*---------------------------------------------------------------------------*/
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

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_ReadHalf
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    result - four bits read from LCD
*
* FUNCTION DESCRIPTION:
*
*    LCD_ReadHalf check data pins and four bits from LCD
*
*---------------------------------------------------------------------------*/
 static inline uint8_t LCD_ReadHalf(void)
 {
	uint8_t result = 0;

	if(LCD_PIN & (1<<LCD_D4)) result |= (1<<0);
	if(LCD_PIN & (1<<LCD_D5)) result |= (1<<1);
	if(LCD_PIN & (1<<LCD_D6)) result |= (1<<2);
	if(LCD_PIN & (1<<LCD_D7)) result |= (1<<3);

	return result;
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_ReadByte
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    result - byte read form LCD
*
* FUNCTION DESCRIPTION:
*
*    LCD_ReadByte read byte from LCD
*
*---------------------------------------------------------------------------*/
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

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Check_BF
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    Return "0" if LCD can accept next command 
*
* FUNCTION DESCRIPTION:
*
*    LCD_Check_BF check if LCD is busy
*
*---------------------------------------------------------------------------*/
 static uint8_t LCD_Check_BF(void)
 {
	CLR_RS;
	return LCD_ReadByte();
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_WraiteByte
*
* FUNCTION ARGUMENTS:
*    data - value that will be written do the LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_WraiteByte write data byte to the LCD
*
*---------------------------------------------------------------------------*/
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

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_WriteCmd
*
* FUNCTION ARGUMENTS:
*    cmd - command that will be wtitten to the LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_WriteCmd write command to the LCD
*
*---------------------------------------------------------------------------*/
 void LCD_WriteCmd(uint8_t cmd)
 {
	CLR_RS;
	LCD_WraiteByte(cmd);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_WriteData
*
* FUNCTION ARGUMENTS:
*    data - data that will be wtitten to the LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_WriteData write data to the LCD
*
*---------------------------------------------------------------------------*/
 void LCD_WriteData(uint8_t data)
 {
	SET_RS;
	LCD_WraiteByte(data);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Init
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Init initialize LCD
*
*---------------------------------------------------------------------------*/
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

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Clear
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Clear clear LCD display
*
*---------------------------------------------------------------------------*/
 void LCD_Clear(void)
 {
	LCD_WriteCmd(CLEAR_DISPLAY);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Home
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Home set cursor in home position(0,0)
*
*---------------------------------------------------------------------------*/
 void LCD_Home(void)
 {
	LCD_WriteCmd(CLEAR_DISPLAY | RETURN_HOME);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_CursorOn
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_CursorOn turn On cursor (cursor is visible)
*
*---------------------------------------------------------------------------*/
 void LCD_CursorOn(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | CURSOR_ON);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_CursorOff
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_CursorOff turn Off cursor (cursor is not visible)
*
*---------------------------------------------------------------------------*/
 void LCD_CursorOff(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | CURSOR_OFF);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_CursorBlinkOn
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_CursorBlinkOn turn On cursor blinking
*
*---------------------------------------------------------------------------*/
 void LCD_CursorBlinkOn(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | BLINK_CURSOR_ON);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCDCursorBlinkOff
*
* FUNCTION ARGUMENTS:
*    None
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCDCursorBlinkOff turn Off cursor blinking
*
*---------------------------------------------------------------------------*/
 void LCDCursorBlinkOff(void)
 {
	LCD_WriteCmd(DISPLAY_ON_OFF_CONTROL | DISPLAY_ON | BLINK_CURSOR_OFF);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Str
*
* FUNCTION ARGUMENTS:
*    str - pointer to string that will be displayed on LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Str write string on the LCD
*
*---------------------------------------------------------------------------*/
 void LCD_Str(char *str)
 {
	register char symbol;

	while((symbol = *(str++)))
		LCD_WriteData(((symbol >= 0x80) && (symbol <= 0x87)) ? (symbol & 0x07) : symbol);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Str_P
*
* FUNCTION ARGUMENTS:
*    str - pointer to string from flash memory that will be displayed on LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Str_P write string from flash memory on LCD
*
*---------------------------------------------------------------------------*/
 void LCD_Str_P(const char *str)
 {
	register char symbol;

	while((symbol = pgm_read_byte(str++)))
		LCD_WriteData(((symbol >= 0x80) && (symbol <= 0x87)) ? (symbol & 0x07) : symbol);
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Str_E
*
* FUNCTION ARGUMENTS:
*    str - pointer to string from EEPROM memory that will be displayed on LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Str_E write string from EEPROM memory on LCD
*
*---------------------------------------------------------------------------*/
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

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Int
*
* FUNCTION ARGUMENTS:
*    value - number that will be displayed on LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Int convert decimal value to string and display it on LCD
*
*---------------------------------------------------------------------------*/
 void LCD_Int(int value)
 {
	char bufor[17];
	LCD_Str(itoa(value, bufor, 10));
 }

 /*----------------------------------------------------------------------------
*
* FUNCTION NAME: LCD_Hex
*
* FUNCTION ARGUMENTS:
*    value - number that will be displayed on LCD
*
* RETURN VALUE:
*    None
*
* FUNCTION DESCRIPTION:
*
*    LCD_Hex convert hexadecimal value to string and display it on LCD
*
*---------------------------------------------------------------------------*/
 void LCD_Hex(int value)
 {
	char bufor[17];
	LCD_Str("0x");
	LCD_Str(itoa(value, bufor, 16));
 }
 