GCCFLAGS=-g -Os -Wall -mmcu=atmega168 

all: lcd.o

lcd.o: lcd.c
	avr-gcc ${GCCFLAGS} -o lcd.o -c lcd.c
