GCCFLAGS=-g -Os -Wall -mmcu=atmega168 
LINKFLAGS=-Wl,-u,vfprintf -lprintf_flt -Wl,-u,vfscanf -lscanf_flt -lm
AVRDUDEFLAGS=-c avr109 -p m168 -b 115200 -P /dev/ttyUSB0
LINKOBJECTS=../hd44780_lcd.o

all:	hd44780_lcd_test-upload

hd44780_lcd_test.hex:	hd44780_lcd_test.c
	make -C ../../hd44780_lcd
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o hd44780_lcd_test.o hd44780_lcd_test.c ${LINKOBJECTS}
	avr-objcopy -j .text -O ihex hd44780_lcd_test.o hd44780_lcd_test.hex
	
hd44780_lcd_test.ass:	hd44780_lcd_test.hex
	avr-objdump -S -d hd44780_lcd_test.o > hd44780_lcd_test.ass
	
hd44780_lcd_test-upload:	hd44780_lcd_test.hex
	avrdude ${AVRDUDEFLAGS} -U flash:w:hd44780_lcd_test.hex:a

build:	hd44780_lcd_test.c
	make -C ../../hd44780_lcd
	avr-gcc ${GCCFLAGS} ${LINKFLAGS} -o hd44780_lcd_test.o hd44780_lcd_test.c ${LINKOBJECTS}
