CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS = arm-none-eabi-as
CP = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
  
CFLAGS = -I./CMSIS/Device/ST/STM32F0xx/Include/ -I./CMSIS/Include/ -I./ -c -fno-common -O0 -g -mcpu=cortex-m0 -mthumb
LFLAGS = -Tstm32f4.ld -nostartfiles

all: main.elf

clean:
	-rm -f main.lst crt.o main.o main.elf main.hex main.map stm32f0xx_rcc.o stm32f0xx_gpio.o main.bin main.list

main.elf: main.o stm32f0xx_rcc.o stm32f0xx_gpio.o stm32f4.ld
	$(LD) $(LFLAGS) -o main.elf main.o stm32f0xx_rcc.o stm32f0xx_gpio.o

stm32f0xx_rcc.o: lib/stm32f0xx_rcc.c 
	 $(CC) $(CFLAGS) lib/stm32f0xx_rcc.c 
	 
stm32f0xx_gpio.o: lib/stm32f0xx_gpio.c
	 $(CC) $(CFLAGS) lib/stm32f0xx_gpio.c 
	 
main.o: main.c
	 $(CC) $(CFLAGS) main.c
