CC      = arm-none-eabi-gcc
LD      = arm-none-eabi-ld
AR      = arm-none-eabi-ar
AS      = arm-none-eabi-as
CP      = arm-none-eabi-objcopy
OD		= arm-none-eabi-objdump
  
CFLAGS  =  -I./ -c -fno-common -O0 -g -mcpu=cortex-m3 -mthumb 
LFLAGS  = -Tstm32f4.ld -nostartfiles

all: main.elf

clean:
	-rm -f main.lst crt.o main.o main.elf main.hex main.map stm32f10x_rcc.o stm32f10x_gpio.o main.bin main.list

main.elf: main.o stm32f10x_rcc.o stm32f10x_gpio.o stm32f4.ld
	$(LD) $(LFLAGS) -o main.elf main.o stm32f10x_rcc.o stm32f10x_gpio.o

stm32f10x_rcc.o: lib/stm32f10x_rcc.c 
	 $(CC) $(CFLAGS) lib/stm32f10x_rcc.c 
	 
stm32f10x_gpio.o: lib/stm32f10x_gpio.c
	 $(CC) $(CFLAGS) lib/stm32f10x_gpio.c 
	 
main.o: main.c
	 $(CC) $(CFLAGS) main.c
