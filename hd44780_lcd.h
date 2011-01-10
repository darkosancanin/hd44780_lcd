/*
HD44780 LCD - 4 Bit Mode
------------------------
Pin Connections:
1. Ground 
2. VCC (+3.3 to +5V)
3. Contrast adjustment (VO)
4. Register Select (RS). RS=0: Command, RS=1: Data (LCD_REGISTER_SELECT_PIN)
5. Read/Write (R/W). R/W=0: Write, R/W=1: Read (GROUND)
6. Clock (Enable). Falling edge triggered (LCD_ENABLE_PIN)
7. Bit 0 (Not used in 4-bit operation)
8. Bit 1 (Not used in 4-bit operation)
9. Bit 2 (Not used in 4-bit operation)
10. Bit 3 (Not used in 4-bit operation)
11. Bit 4 (LCD_DATA_PIN_1)
12. Bit 5 (LCD_DATA_PIN_2)
13. Bit 6 (LCD_DATA_PIN_3)
14. Bit 7 (LCD_DATA_PIN_4)
*/

#define F_CPU 14745600

#define LCD_REGISTER_SELECT_DDR DDRD
#define LCD_REGISTER_SELECT_PORT PORTD
#define LCD_REGISTER_SELECT_PIN PD2

#define LCD_ENABLE_DDR DDRD
#define LCD_ENABLE_PORT PORTD
#define LCD_ENABLE_PIN PD3 

#define LCD_DATA_DDR DDRD
#define LCD_DATA_PORT PORTD
#define LCD_DATA_PIN_1 PD4 
#define LCD_DATA_PIN_2 PD5
#define LCD_DATA_PIN_3 PD6
#define LCD_DATA_PIN_4 PD7

#define LCD_COMMAND_INITIALIZATION_INSTRUCTION 0b11 //0x3
#define LCD_COMMAND_CLEAR_DISPLAY 0b1 //0x1
#define LCD_COMMAND_4_BIT_MODE 0b10 //0x2
#define LCD_COMMAND_RETURN_HOME 0b10 //0x2
#define LCD_COMMAND_TURN_DISPLAY_OFF 0b1000 //0x8
#define LCD_COMMAND_TURN_DISPLAY_ON 0b1100 //0xC
#define LCD_COMMAND_TURN_CURSOR_BLINK_ON 0b1111 //0xF
#define LCD_COMMAND_TURN_CURSOR_BLINK_OFF 0b1100 //0xC
#define LCD_COMMAND_FUNCTION_SET_4_BIT_MODE_AND_2_LINES 0b101000 //0x28
#define LCD_COMMAND_ENTRY_MODE_SET_WITH_INCREMENT_ON 0b110 //0x6
#define LCD_COMMAND_CURSOR_SHIFT_ON_TO_RIGHT 0b10100 //0x14

#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

void lcd_enter_command_mode();
void lcd_enter_data_mode();
void lcd_write_nibble(char data);
void lcd_write_byte(char data);
void lcd_write_string(const char *data);
void lcd_goto_position(uint8_t row, uint8_t column);
void lcd_turn_blinking_cursor_on();
void lcd_turn_blinking_cursor_off();
void lcd_clear_screen();
void lcd_initialize();
