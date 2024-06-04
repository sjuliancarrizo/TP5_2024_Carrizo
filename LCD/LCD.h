#ifndef __LCD16X4_H
#define __LCD16X4_H

#ifdef __cplusplus
extern "C" {
#endif

/** Includes ---------------------------------------------------------------- */
#include "string.h"
#include "stddef.h"
#include "stdint.h"

#define _CENTRAR            			0
#define _IZQ							1

/** Display size ------------------------------------------------------------ */
#define LCD_LINES						2		// Configuro display, 2 o 4 lineas?

#if	(LCD_LINES == 2)
#define LCD_START_LINE_1				0x00
#define LCD_START_LINE_2				0x40
#else
#define LCD_START_LINE_1				0x80
#define LCD_START_LINE_2				0xc0
#define LCD_START_LINE_3				0x94
#define LCD_START_LINE_4				0xd4
#endif

/** Delay value ------------------------------------------------------------- */
#define LCD_DELAY_POWER_ON   			15		// Delay power on
#define LCD_DELAY_BUSY_FLAG    			4		// Delay until address counter updated after busy flag is cleared4
#define LCD_DELAY_ENABLE_PULSE			50		// Pulso de enable

/** Instructions bit location ----------------------------------------------- */
#define LCD_CLEAR_DISPLAY				0x01
#define LCD_CURSOR_HOME					0x02
#define LCD_CHARACTER_ENTRY_MODE		0x04
#define LCD_DISPLAY_CURSOR_ON_OFF		0x08
#define LCD_DISPLAY_CURSOR_SHIFT 		0x10
#define LCD_FUNCTION_SET				0x20
#define LCD_SET_CGRAM_ADDRESS	 		0x40
#define LCD_SET_DDRAM_ADDRESS	 		0x80

/* Character entry mode instructions */
#define LCD_INCREMENT					0x02	// Initialization setting
#define LCD_DECREMENT					0x00
#define LCD_DISPLAY_SHIFT_ON			0x01
#define LCD_DISPLAY_SHIFT_OFF			0x00	// Initialization setting

/* Display cursor on off instructions */
#define LCD_DISPLAY_ON	 				0x04
#define LCD_DISPLAY_OFF	 				0x00	// Initialization setting
#define LCD_CURSOR_UNDERLINE_ON	 		0x02
#define LCD_CURSOR_UNDERLINE_OFF		0x00	// Initialization setting
#define LCD_CURSOR_BLINK_ON	 			0x01
#define LCD_CURSOR_BLINK_OFF	 		0x00	// Initialization setting

/* Display cursor shift instructions */
#define LCD_DISPLAY_SHIFT				0x08
#define LCD_CURSOR_MOVE					0x00
#define LCD_RIGHT_SHIFT					0x04
#define LCD_LEFT_SHIFT					0x00

/* Function set instructions */
#define LCD_8BIT_INTERFACE				0x10	// Initialization setting
#define LCD_4BIT_INTERFACE				0x00
#define LCD_2LINE_MODE					0x08
#define LCD_1LINE_MODE					0x00	// Initialization setting
#define LCD_5X10DOT_FORMAT				0x04
#define LCD_5X7DOT_FORMAT				0x00	// Initialization setting

/* Busy flag bit location */
#define LCD_BUSY_FLAG					0x80

/** LCD display and cursor attributes --------------------------------------- */
#define LCD_DISPLAY_OFF_CURSOR_OFF_BLINK_OFF	(LCD_DISPLAY_OFF| LCD_CURSOR_UNDERLINE_OFF | LCD_CURSOR_BLINK_OFF)
#define LCD_DISPLAY_ON_CURSOR_OFF_BLINK_OFF		(LCD_DISPLAY_ON |  LCD_CURSOR_UNDERLINE_OFF | LCD_CURSOR_BLINK_OFF)
#define LCD_DISPLAY_ON_CURSOR_OFF_BLINK_ON		(LCD_DISPLAY_ON | LCD_CURSOR_UNDERLINE_OFF | LCD_CURSOR_BLINK_ON)
#define LCD_DISPLAY_ON_CURSOR_ON_BLINK_OFF		(LCD_DISPLAY_ON | LCD_CURSOR_UNDERLINE_ON | LCD_CURSOR_BLINK_OFF)
#define LCD_DISPLAY_ON_CURSOR_ON_BLINK_ON		(LCD_DISPLAY_ON | LCD_CURSOR_UNDERLINE_ON | LCD_CURSOR_BLINK_ON)

/** Public function prototypes ---------------------------------------------- */
void LCD_init(void);
void LCD_write_command(uint8_t cmd);
void LCD_write_data(uint8_t data);
void LCD_clrscr(void);
void LCD_home(void);
void LCD_gotoxy(uint8_t x, uint8_t y);
uint8_t LCD_getxy(void);
void LCD_entry_inc(void);
void LCD_entry_dec(void);
void LCD_entry_inc_shift(void);
void LCD_entry_dec_shift(void);
void LCD_display_on(void);
void LCD_display_off(void);
void LCD_cursor_on(void);
void LCD_cursor_off(void);
void LCD_blink_on(void);
void LCD_blink_off(void);
void LCD_display_shift_left(void);
void LCD_display_shift_right(void);
void LCD_cursor_shift_left(void);
void LCD_cursor_shift_right(void);
void LCD_putc(char c);
void LCD_puts(char *s);
void LCD_create_custom_char(uint8_t location, uint8_t* data_bytes);
void LCD_put_custom_char(uint8_t x, uint8_t y, uint8_t location);
void LCD_WriteString(uint8_t x, uint8_t y, char *Texto);

#ifdef __cplusplus
}
#endif

#endif /* LCD_LCD16X4_H_ */
