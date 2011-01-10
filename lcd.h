#define F_CPU 14745600

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
