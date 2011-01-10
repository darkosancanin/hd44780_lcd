GCCFLAGS=-g -Os -Wall -mmcu=atmega168

all: hd44780_lcd.o

hd44780_lcd.o: hd44780_lcd.c
	avr-gcc ${GCCFLAGS} -o hd44780_lcd.o -c hd44780_lcd.c 
