/*
This test is for a 20x4 LCD.
*/

#include "../hd44780_lcd.h"

int main() {
  lcd_initialize();

  while(1) {
    lcd_clear_screen();
    lcd_goto_position(1, 1);
    lcd_turn_blinking_cursor_on();
    _delay_ms(2000);
    lcd_turn_blinking_cursor_off();

    int i;
    for(i=1;i<=20;i++){
      lcd_goto_position(1, i);
      lcd_write_string(PSTR("*"));
      _delay_ms(100);
    }
    lcd_goto_position(2, 20);
    lcd_write_string(PSTR("*"));
    _delay_ms(100);
    lcd_goto_position(3, 20);
    lcd_write_string(PSTR("*"));
    _delay_ms(100);
    int j;
    for(j=20;j>=1;j--){
      lcd_goto_position(4, j);
      lcd_write_string(PSTR("*"));
      _delay_ms(100);
    }
    lcd_goto_position(3, 1);
    lcd_write_string(PSTR("*"));
    _delay_ms(100);
    lcd_goto_position(2, 1);
    lcd_write_string(PSTR("*"));
    _delay_ms(500);

    lcd_goto_position(2, 1);
    lcd_write_string(PSTR("*     HD44780      *"));
    lcd_goto_position(3, 1);
    lcd_write_string(PSTR("*       LCD        *"));

    _delay_ms(4000);
  }
  
  return 0;
}
















