/**
 ******************************************************************************
 * @file		LCD.c
 * @author		Bruno Palacios
 * @date		6 February 2021
 * @note		Re-write form Yohanes Erwin Setiawan AVR LCD library
 ******************************************************************************
 */

/** Includes ---------------------------------------------------------------- */
#include "LCD.h"

#include "../Global/global.h"						// Definición de delay_ms, entre otros.


/** Local Defines: Port and pin definition for 4-bit mode ---------------------------------- */

#define	USE_RW				0			// 0: no se utiliza R/W - 1: Se utiliza R/W

//--------- CONTROL PINs (RS, RW, EN) ------------
#define LCD_RCC_GPIO_CONTROL RCC_AHB1Periph_GPIOA

#define LCD_GPIO_RS			GPIOB
#define LCD_GPIO_EN			GPIOB

#define LCD_PIN_RS			GPIO_Pin_15
#define LCD_PIN_EN			GPIO_Pin_4

#if (USE_RW == 1)

#define LCD_GPIO_RW			GPIO?		// Si no se usa (lo mas ocmun)
#define LCD_PIN_RW			GPIO_Pin_?

#endif

//--------- DATA PINs (D4~D7) ------------
#define LCD_RCC_GPIO_DATA_1	(RCC_AHB1Periph_GPIOA)
#define LCD_RCC_GPIO_DATA_2	(RCC_AHB1Periph_GPIOB)
#define LCD_RCC_GPIO_DATA_3	(RCC_AHB1Periph_GPIOC)

#define LCD_GPIO_D4			GPIOB
#define LCD_GPIO_D5			GPIOB
#define LCD_GPIO_D6			GPIOA
#define LCD_GPIO_D7			GPIOC

#define LCD_PIN_D4			GPIO_Pin_14		// 4-bit mode LSB
#define LCD_PIN_D5			GPIO_Pin_5
#define LCD_PIN_D6			GPIO_Pin_10
#define LCD_PIN_D7			GPIO_Pin_4		// 4-bit mode MSB


/** Private function prototypes --------------------------------------------- */

static uint8_t 				display_cursor_on_off_control;
static GPIO_InitTypeDef 	GPIO_InitStruct;


/** Private function prototypes --------------------------------------------- */
static void 	LCD_CLK(void);
static void 	LCD_write(uint8_t data, uint8_t rs);

#if (USE_RW == 1)
static uint8_t 	LCD_read(uint8_t rs);
static uint8_t 	LCD_wait_busy(void);
#endif


/** Public functions -------------------------------------------------------- */
/**
 ******************************************************************************
 * @brief	Initialize the LCD 16x2 with 4-bit I/O mode.
 * @param	Display, cursor underline, and cursor blink settings. See
 * 				LCD display and cursor attributes define in LCD.h file.
 * @retval	None
 ******************************************************************************
 */
void LCD_init(void)
{

	uint8_t CMD[4] = {
			LCD_FUNCTION_SET | LCD_4BIT_INTERFACE | LCD_2LINE_MODE | LCD_5X7DOT_FORMAT,			// Function Set: 	0x28
			LCD_DISPLAY_CURSOR_ON_OFF | LCD_DISPLAY_ON,											// Display ON/OFF: 	0x0C
			LCD_CLEAR_DISPLAY,																	// Clear display: 	0x01
			LCD_CHARACTER_ENTRY_MODE | LCD_INCREMENT | LCD_DISPLAY_SHIFT_OFF					// Entry mode: 		0x06
	};

	RCC_AHB1PeriphClockCmd(LCD_RCC_GPIO_CONTROL, ENABLE);	// GPIO clock for control and data lines
	RCC_AHB1PeriphClockCmd(LCD_RCC_GPIO_DATA_1, ENABLE);
	RCC_AHB1PeriphClockCmd(LCD_RCC_GPIO_DATA_2, ENABLE);
	RCC_AHB1PeriphClockCmd(LCD_RCC_GPIO_DATA_3, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;				// Configure I/O for control lines as output
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_RS;
	GPIO_Init(LCD_GPIO_RS, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_EN;
	GPIO_Init(LCD_GPIO_EN, &GPIO_InitStruct);

#if (USE_RW == 1)
	GPIO_InitStruct.GPIO_Pin = LCD_PIN_RW;
	GPIO_Init(LCD_GPIO_RW, &GPIO_InitStruct);
#endif

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D4;					// Configure I/O for data lines as output
	GPIO_Init(LCD_GPIO_D4, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D5;
	GPIO_Init(LCD_GPIO_D5, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D6;
	GPIO_Init(LCD_GPIO_D6, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D7;
	GPIO_Init(LCD_GPIO_D7, &GPIO_InitStruct);

	GPIO_ResetBits(LCD_GPIO_RS, LCD_PIN_RS);
	GPIO_ResetBits(LCD_GPIO_EN, LCD_PIN_EN);
	GPIO_ResetBits(LCD_GPIO_D4, LCD_PIN_D4);
	GPIO_ResetBits(LCD_GPIO_D5, LCD_PIN_D5);
	GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);
	GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);

	delay_ms(LCD_DELAY_POWER_ON);							// Wait for power on

	GPIO_SetBits(LCD_GPIO_D4, LCD_PIN_D4);					// 8-bit mode
	GPIO_SetBits(LCD_GPIO_D5, LCD_PIN_D5);					// Function set
	GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);				// Initialize 8-bit mode first
	GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);

	LCD_CLK();
	delay_ms(6);

	LCD_CLK();
	delay_ms(6);

	LCD_CLK();
	delay_ms(6);

	GPIO_ResetBits(LCD_GPIO_D4, LCD_PIN_D4);
	GPIO_SetBits(LCD_GPIO_D5, LCD_PIN_D5);					// Function set
	GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);				// Initialize 8-bit mode first
	GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);

	LCD_CLK();

	/* From now the LCD only accepts 4 bit I/O */
	LCD_write_command(CMD[0]);
	LCD_write_command(CMD[1]);
	LCD_write_command(CMD[2]);
	delay_ms(2);
	LCD_write_command(CMD[3]);
	delay_ms(2);
}

/**
 ******************************************************************************
 * @brief	Write a command to the LCD.
 * @param	The LCD instructions set.
 * @retval	None
 ******************************************************************************
 */
void LCD_write_command(uint8_t cmd)
{
#if (USE_RW == 1)
	LCD_wait_busy();
#endif

	LCD_write(cmd, 0);
}

/**
 ******************************************************************************
 * @brief	Write a data byte to the LCD.
 * @param	Data which want to written to the LCD.
 * @retval	None
 ******************************************************************************
 */

void LCD_write_data(uint8_t data)
{
#if (USE_RW == 1)
	LCD_wait_busy();
#endif

	LCD_write(data, 1);
}

/**
 ******************************************************************************
 * @brief	Clear the LCD display and return cursor to home position.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_clrscr(void)
{
	LCD_write_command(LCD_CLEAR_DISPLAY);
	delay_ms(2);
}

/**
 ******************************************************************************
 * @brief	Return cursor to home position.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_home(void)
{
	LCD_write_command(LCD_CURSOR_HOME);
	delay_ms(2);
}

/**
 ******************************************************************************
 * @brief	Set LCD cursor to specific position.
 * @param	LCD column (x)
 * @param	LCD row (y)
 * @retval	None
 ******************************************************************************
 */
void LCD_gotoxy(uint8_t x, uint8_t y)
{
#if LCD_LINES == 1

	LCD_write_command(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_1 + x));

#elif LCD_LINES == 2

	if (y == 0)
		LCD_write_command(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_1 + x));
	else
		LCD_write_command(LCD_SET_DDRAM_ADDRESS | (LCD_START_LINE_2 + x));

#elif LCD_LINES == 4

	if (y == 0)
		LCD_write_command((LCD_START_LINE_1 + x));
	else if (y == 1)
		LCD_write_command((LCD_START_LINE_2 + x));
	else if (y == 2)
		LCD_write_command((LCD_START_LINE_3 + x));
	else if (y == 3)
		LCD_write_command((LCD_START_LINE_4 + x));

#endif
}

/**
 ******************************************************************************
 * @brief	Get LCD cursor/ DDRAM address.
 * @param	None
 * @retval	LCD cursor/ DDRAM address.
 ******************************************************************************
 */
uint8_t LCD_getxy(void)
{
#if (USE_RW == 1)
	return LCD_wait_busy();
#else
	return 0;
#endif
}

/**
 ******************************************************************************
 * @brief	Set LCD entry mode: increment cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_entry_inc(void)
{
	LCD_write_command(LCD_CHARACTER_ENTRY_MODE | LCD_INCREMENT | LCD_DISPLAY_SHIFT_OFF);
}

/**
 ******************************************************************************
 * @brief	Set LCD entry mode: decrement cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_entry_dec(void)
{
	LCD_write_command(LCD_CHARACTER_ENTRY_MODE | LCD_DECREMENT | LCD_DISPLAY_SHIFT_OFF);
}

/**
 ******************************************************************************
 * @brief	Set LCD entry mode: increment cursor and shift character to left.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_entry_inc_shift(void)
{
	LCD_write_command(LCD_CHARACTER_ENTRY_MODE | LCD_INCREMENT | LCD_DISPLAY_SHIFT_ON);
}

/**
 ******************************************************************************
 * @brief	Set LCD entry mode: decrement cursor and shift character to right.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_entry_dec_shift(void)
{
	LCD_write_command(LCD_CHARACTER_ENTRY_MODE | LCD_DECREMENT | LCD_DISPLAY_SHIFT_ON);
}

/**
 ******************************************************************************
 * @brief	Turn on display (can see character(s) on display).
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_display_on(void)
{
	display_cursor_on_off_control |= LCD_DISPLAY_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Turn off display (blank/ can't see character(s) on display).
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_display_off(void)
{
	display_cursor_on_off_control &= ~LCD_DISPLAY_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Turn on underline cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_cursor_on(void)
{
	display_cursor_on_off_control |= LCD_CURSOR_UNDERLINE_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Turn off underline cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_cursor_off(void)
{
	display_cursor_on_off_control &= ~LCD_CURSOR_UNDERLINE_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Turn on blinking cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_blink_on(void)
{
	display_cursor_on_off_control |= LCD_CURSOR_BLINK_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Turn off blinking cursor.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_blink_off(void)
{
	display_cursor_on_off_control &= ~LCD_CURSOR_BLINK_ON;
	LCD_write_command(LCD_DISPLAY_CURSOR_ON_OFF | display_cursor_on_off_control);
}

/**
 ******************************************************************************
 * @brief	Shift the LCD display to the left.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_display_shift_left(void)
{
	LCD_write_command(LCD_DISPLAY_CURSOR_SHIFT | LCD_DISPLAY_SHIFT | LCD_LEFT_SHIFT);
}

/**
 ******************************************************************************
 * @brief	Shift the LCD display to the right.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_display_shift_right(void)
{
	LCD_write_command(LCD_DISPLAY_CURSOR_SHIFT | LCD_DISPLAY_SHIFT | LCD_RIGHT_SHIFT);
}

/**
 ******************************************************************************
 * @brief	Shift the LCD cursor to the left (DDRAM address incremented).
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_cursor_shift_left(void)
{
	LCD_write_command(LCD_DISPLAY_CURSOR_SHIFT | LCD_DISPLAY_CURSOR_SHIFT | LCD_LEFT_SHIFT);
}

/**
 ******************************************************************************
 * @brief	Shift the LCD cursor to the right (DDRAM address decremented).
 * @param	None
 * @retval	None
 ******************************************************************************
 */
void LCD_cursor_shift_right(void)
{
	LCD_write_command(LCD_DISPLAY_CURSOR_SHIFT | LCD_DISPLAY_CURSOR_SHIFT | LCD_RIGHT_SHIFT);
}

/**
 ******************************************************************************
 * @brief	Put a character on the LCD display.
 * @param	Character that want to be displayed.
 * @retval	None
 ******************************************************************************
 */
void LCD_putc(char c)
{
	LCD_write_data(c);
}

/**
 ******************************************************************************
 * @brief	Put string on the LCD display.
 * @param	String that want to be displayed.
 * @retval	None
 ******************************************************************************
 */
void LCD_puts(char *s)
{
	while (*s) {
		LCD_putc(*s++);
	}
}

/**
 ******************************************************************************
 * @brief	Create a custom character on CGRAM location.
 * @param	CGRAM location (0-7).
 * @param	Custom character pattern (8 bytes).
 * @retval	None
 ******************************************************************************
 */
void LCD_create_custom_char(uint8_t location, uint8_t* data_bytes)
{
	int i;

	// We only have 8 locations 0-7 for custom chars
	location &= 0x07;

	// Set CGRAM address
	LCD_write_command(LCD_SET_CGRAM_ADDRESS | (location << 3));

	// Write 8 bytes custom char pattern
	for (i = 0; i < 8; i++)
	{
		LCD_write_data(data_bytes[i]);
	}
}

/**
 ******************************************************************************
 * @brief	Put a custom character on specific LCD display location.
 * @param	LCD column
 * @param	LCD row
 * @param	Custom character location on CGRAM (0-7).
 * @retval	None
 ******************************************************************************
 */
void LCD_put_custom_char(uint8_t x, uint8_t y, uint8_t location)
{
	LCD_gotoxy(x, y);
	LCD_write_data(location);
}

/** Private functions ------------------------------------------------------- */
/**
 ******************************************************************************
 * @brief	Give enable pulse to LCD EN pin.
 * @param	None
 * @retval	None
 ******************************************************************************
 */
static void LCD_CLK(void)
{
	GPIO_SetBits(LCD_GPIO_EN, LCD_PIN_EN);
	delay_us(LCD_DELAY_ENABLE_PULSE);

	GPIO_ResetBits(LCD_GPIO_EN, LCD_PIN_EN);
	delay_us(LCD_DELAY_ENABLE_PULSE);
}

/**
 ******************************************************************************
 * @brief	Write instruction or data to LCD.
 * @param	Instruction/ data that want to sent to LCD.
 * @param	Instruction or data register select. If write instruction, then
 *					RS = 0. Otherwise, RS = 1.
 * @retval	None
 ******************************************************************************
 */
static void LCD_write(uint8_t data, uint8_t rs)
{

#if (USE_RW == 1)
	// Write mode (RW = 0)
	GPIO_ResetBits(LCD_GPIO_RW, LCD_PIN_RW);
#endif

	if (rs)		GPIO_SetBits(LCD_GPIO_RS, LCD_PIN_RS);
	else		GPIO_ResetBits(LCD_GPIO_RS, LCD_PIN_RS);

#if (USE_RW == 1)
	// Configure all data pins as output
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D4;
	GPIO_Init(LCD_GPIO_D4, &GPIO_InitStruct);
	GPIO_ResetBits(LCD_GPIO_D4, LCD_PIN_D4);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D5;
	GPIO_Init(LCD_GPIO_D5, &GPIO_InitStruct);
	GPIO_ResetBits(LCD_GPIO_D5, LCD_PIN_D5);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D6;
	GPIO_Init(LCD_GPIO_D6, &GPIO_InitStruct);
	GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D7;
	GPIO_Init(LCD_GPIO_D7, &GPIO_InitStruct);
	GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);
#endif

	// Envio parte alta
	if (data & 0x80) 	GPIO_SetBits(LCD_GPIO_D7, LCD_PIN_D7);
	else				GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);
	if (data & 0x40) 	GPIO_SetBits(LCD_GPIO_D6, LCD_PIN_D6);
	else				GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);
	if (data & 0x20) 	GPIO_SetBits(LCD_GPIO_D5, LCD_PIN_D5);
	else				GPIO_ResetBits(LCD_GPIO_D5, LCD_PIN_D5);
	if (data & 0x10) 	GPIO_SetBits(LCD_GPIO_D4, LCD_PIN_D4);
	else				GPIO_ResetBits(LCD_GPIO_D4, LCD_PIN_D4);

	LCD_CLK();

	// Envio parte baja
	if (data & 0x08) 	GPIO_SetBits(LCD_GPIO_D7, LCD_PIN_D7);
	else				GPIO_ResetBits(LCD_GPIO_D7, LCD_PIN_D7);
	if (data & 0x04) 	GPIO_SetBits(LCD_GPIO_D6, LCD_PIN_D6);
	else				GPIO_ResetBits(LCD_GPIO_D6, LCD_PIN_D6);
	if (data & 0x02) 	GPIO_SetBits(LCD_GPIO_D5, LCD_PIN_D5);
	else				GPIO_ResetBits(LCD_GPIO_D5, LCD_PIN_D5);
	if (data & 0x01) 	GPIO_SetBits(LCD_GPIO_D4, LCD_PIN_D4);
	else				GPIO_ResetBits(LCD_GPIO_D4, LCD_PIN_D4);

	LCD_CLK();

}

/**
 ******************************************************************************
 * @brief	Read DDRAM address + busy flag or data from LCD.
 * @param	DDRAM address + busy flag or data register select.
 *					If read DDRAM address + busy flag, then RS = 0. Otherwise, RS = 1.
 * @retval	DDRAM address + busy flag or data value.
 ******************************************************************************
 */
#if (USE_RW == 1)
static uint8_t LCD_read(uint8_t rs)
{
	uint8_t data = 0;

	// Write mode (RW = 1)
	GPIO_SetBits(LCD_GPIO_RW, LCD_PIN_RW);

	if (rs)		GPIO_SetBits(LCD_GPIO_RS, LCD_PIN_RS);	// Write data (RS = 1)
	else		GPIO_ResetBits(LCD_GPIO_RS, LCD_PIN_RS);// Write instruction (RS = 0)

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;			// Configure all data pins as output
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;		// Configure all data pins as input

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D4;
	GPIO_Init(LCD_GPIO_D4, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D5;
	GPIO_Init(LCD_GPIO_D5, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D6;
	GPIO_Init(LCD_GPIO_D6, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = LCD_PIN_D7;
	GPIO_Init(LCD_GPIO_D7, &GPIO_InitStruct);

	// EN pin = HIGH
	GPIO_SetBits(LCD_GPIO_EN, LCD_PIN_EN);

	// Pulse length in us
	delay_us(LCD_DELAY_ENABLE_PULSE);

	/* Read high nibble first */
	if (GPIO_ReadInputDataBit(LCD_GPIO_D4, LCD_PIN_D4))	data |= 0x10;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D5, LCD_PIN_D5)) data |= 0x20;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D6, LCD_PIN_D6)) data |= 0x40;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D7, LCD_PIN_D7)) data |= 0x80;

	GPIO_ResetBits(LCD_GPIO_EN, LCD_PIN_EN);
	delay_us(LCD_DELAY_ENABLE_PULSE);					// EN pin LOW delay

	GPIO_SetBits(LCD_GPIO_EN, LCD_PIN_EN);
	delay_us(LCD_DELAY_ENABLE_PULSE);					// Pulse length in us

	/* Read low nibble */
	if (GPIO_ReadInputDataBit(LCD_GPIO_D4, LCD_PIN_D4)) data |= 0x01;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D5, LCD_PIN_D5)) data |= 0x02;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D6, LCD_PIN_D6)) data |= 0x04;
	if (GPIO_ReadInputDataBit(LCD_GPIO_D7, LCD_PIN_D7)) data |= 0x08;

	GPIO_ResetBits(LCD_GPIO_EN, LCD_PIN_EN);

	return data;
}
#endif
/**
 ******************************************************************************
 * @brief	Wait for LCD until finish it's job.
 * @param	None
 * @retval	DDRAM address + busy flag value.
 ******************************************************************************
 */
#if (USE_RW == 1)
static uint8_t LCD_wait_busy()
{
	// Wait until busy flag is cleared
	while (LCD_read(0) & (LCD_BUSY_FLAG));

	// Delay needed for address counter is updated after busy flag is cleared
	delay_us(LCD_DELAY_BUSY_FLAG);

	// Read and return address counter
	return LCD_read(0);
}
#endif


void LCD_WriteString(uint8_t x, uint8_t y, char *Texto){

	uint8_t	i;

	LCD_gotoxy(x, y);

	for(i = 0; i < strlen(Texto); i++){

		LCD_write_data(Texto[i]);
	}
}
/********************************* END OF FILE ********************************/
/******************************************************************************/
