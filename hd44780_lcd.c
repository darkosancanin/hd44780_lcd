#include "hd44780_lcd.h"

void lcd_enter_command_mode(){
  LCD_REGISTER_SELECT_PORT &= ~(1<<LCD_REGISTER_SELECT_PIN);
}

void lcd_enter_data_mode(){
  LCD_REGISTER_SELECT_PORT |= (1<<LCD_REGISTER_SELECT_PIN);
}

void lcd_write_nibble(char data){
  //clear all the data bits
  LCD_DATA_PORT &= ~((1<<LCD_DATA_PIN_1) | (1<<LCD_DATA_PIN_2) | (1<<LCD_DATA_PIN_3) | (1<<LCD_DATA_PIN_4));
  
  if(data & 0b0001) LCD_DATA_PORT |= (1<<LCD_DATA_PIN_1);
  if(data & 0b0010) LCD_DATA_PORT |= (1<<LCD_DATA_PIN_2);
  if(data & 0b0100) LCD_DATA_PORT |= (1<<LCD_DATA_PIN_3);
  if(data & 0b1000) LCD_DATA_PORT |= (1<<LCD_DATA_PIN_4);
 
  //turn on enable pin to high
  LCD_ENABLE_PORT |= (1<<LCD_ENABLE_PIN);
  _delay_us(1);

  //turn on enable pin to low
  LCD_ENABLE_PORT &= ~(1<<LCD_ENABLE_PIN);
  _delay_us(1);  
}

void lcd_write_byte(char data){
  lcd_write_nibble(data >> 4);
  lcd_write_nibble(data);
  _delay_us(100);
}

void lcd_write_string(const char *data){
  lcd_enter_data_mode();
  while(pgm_read_byte(data) != 0x00)
    lcd_write_byte(pgm_read_byte(data++));
}

void lcd_goto_position(uint8_t row, uint8_t column) {
  lcd_enter_command_mode();
  uint8_t row_offset = 0;
  switch(row) {
    case 1: row_offset = 0; break;
    case 2: row_offset = 64; break;
    case 3: row_offset = 20; break;
    case 4: row_offset = 84; break;
  }
  lcd_write_byte(0b10000000 | (row_offset + (column - 1)));
}

void lcd_turn_blinking_cursor_on(){
  lcd_enter_command_mode();
  lcd_write_byte(LCD_COMMAND_TURN_CURSOR_BLINK_ON);
  _delay_ms(50);
}

void lcd_turn_blinking_cursor_off(){
  lcd_enter_command_mode();
  lcd_write_byte(LCD_COMMAND_TURN_CURSOR_BLINK_OFF);
  _delay_ms(50);
}

void lcd_clear_screen(){
  lcd_enter_command_mode();
  lcd_write_byte(LCD_COMMAND_CLEAR_DISPLAY);
  _delay_ms(50);
}

void lcd_initialize(){
  //set the enable pin as output
  LCD_ENABLE_DDR |= (1<<LCD_ENABLE_PIN); 
  
  //set the register select pin as output
  LCD_REGISTER_SELECT_DDR |= (1<<LCD_REGISTER_SELECT_PIN); 
  
  //set the register select pin as output
  LCD_DATA_DDR |= (1<<LCD_DATA_PIN_1) | (1<<LCD_DATA_PIN_2) | (1<<LCD_DATA_PIN_3) | (1<<LCD_DATA_PIN_4); 

  _delay_ms(100);
  
  lcd_enter_command_mode();
  lcd_write_nibble(LCD_COMMAND_INITIALIZATION_INSTRUCTION);
  _delay_ms(5);
  lcd_write_nibble(LCD_COMMAND_INITIALIZATION_INSTRUCTION);
  _delay_ms(5);
  lcd_write_nibble(LCD_COMMAND_INITIALIZATION_INSTRUCTION);
  _delay_ms(5);
  lcd_write_nibble(LCD_COMMAND_4_BIT_MODE);
  lcd_write_byte(LCD_COMMAND_FUNCTION_SET_4_BIT_MODE_AND_2_LINES);
  lcd_write_byte(LCD_COMMAND_TURN_DISPLAY_OFF);
  lcd_write_byte(LCD_COMMAND_CLEAR_DISPLAY);
  _delay_ms(5);
  lcd_write_byte(LCD_COMMAND_ENTRY_MODE_SET_WITH_INCREMENT_ON);
  lcd_write_byte(LCD_COMMAND_CURSOR_SHIFT_ON_TO_RIGHT);
  lcd_write_byte(LCD_COMMAND_TURN_DISPLAY_ON);
  lcd_enter_data_mode();
}
