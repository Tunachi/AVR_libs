/*
 * LCD.h
 *
 * Created: 2017-06-10 16:17:40
 *  Author: Tunachi
 */ 


#ifndef LCD_H_
#define LCD_H_

/*------------------------------ display resolution ----------------------*/
#define LCD_X 20
#define LCD_Y 4

/*------------------------------ display pins -------------------------------*/
#define LCD_PORT	PORTD
#define LCD_PIN		PIND
#define LCD_DDR		DDRD
#define LCD_RS		PD0
#define LCD_RW		PD1
#define LCD_E		PD2
#define LCD_D4		PD3
#define LCD_D5		PD4
#define LCD_D6		PD5
#define LCD_D7		PD6

/*------------------------------ display line DDRAM address------------------*/
#if ((LCD_X == 20) && (LCD_Y == 4))
#define LCD_LINE1	0x00
#define LCD_LINE2	0x40
#define LCD_LINE3	0x14
#define LCD_LINE4	0x54
#endif

/*------------------------------ LCD instruction defines --------------------*/
#define CLEAR_DISPLAY						0x01
/*---------------------------------------------------------------------------*/
#define RETURN_HOME							0x02
/*---------------------------------------------------------------------------*/
#define ENTRY_MODE_SET						0x04
#define CURSOR_OR_DISPLAY_DIR_LEFT			0x00
#define CURSOR_OR_DISPLAY_DIR_RIGHT			0x02
#define DISPLAY_IS_MOVING					0x01
#define CURSOR_IS_MOVING					0x00
/*---------------------------------------------------------------------------*/
#define DISPLAY_ON_OFF_CONTROL				0x08
#define DISPLAY_ON							0x04
#define DISPLAY_OFF							0x00
#define CURSOR_ON							0x02
#define CURSOR_OFF							0x00
#define BLINK_CURSOR_ON						0x01
#define BLINK_CURSOR_OFF					0x00
/*---------------------------------------------------------------------------*/
#define CURSOR_OR_DISPLAY_SFIFT				0x10
#define DISPLAY_SHIFT						0x08
#define CURSOR_MOVING						0x00
#define DIR_LEFT							0x00
#define DIR_RIGHT							0x04
/*---------------------------------------------------------------------------*/
#define FUNCTION_SET						0x20
#define INTERFACE_4BIT						0x00
#define INTERFACE_8BIT						0x10
#define DISPLAY_2_LINE						0x08
#define DISPLAY_1_LINE						0x00
#define FONT_5X8							0x00
#define FONT_5X11							0x04
/*---------------------------------------------------------------------------*/
#define CGRAM_ADDRESS						0x40
/*---------------------------------------------------------------------------*/
#define DDRAM_ADDRESS						0x80
/*---------------------------------------------------------------------------*/



void LCD_Init(void);
void LCD_Clear(void);
void LCD_Home(void);
void LCD_CursorOn(void);
void LCD_CursorOff(void);
void LCD_CursorBlinkOn(void);
void LCDCursorBlinkOff(void);
void LCD_Str(char *str);
void LCD_Str_P(const char *str);
void LCD_Str_E(const char *str);
void LCD_Int(int value);
void LCD_Hex(int value);

#endif /* LCD_H_ */