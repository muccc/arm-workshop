CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS = arm-none-eabi-as
CP = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
  
CFLAGS = -I./CMSIS/Include/
CFLAGS += -I././STM32F4xx_HAL_Driver/Inc
CFLAGS += -I./ -c -fno-common -O0 -g -mcpu=cortex-m4 -mthumb
LFLAGS = -Tstm32f4.ld -nostartfiles

all: main.elf

clean:
	-rm -f main.lst crt.o main.o main.elf main.hex main.map stm32f4xx_hal_rcc.o stm32f4xx_hal_gpio.o main.bin main.list

main.elf: main.o stm32f4xx_hal_rcc.o stm32f4xx_hal_gpio.o stm32f4.ld
	$(LD) $(LFLAGS) -o main.elf main.o stm32f4xx_hal_rcc.o stm32f4xx_hal_gpio.o

stm32f4xx_hal_rcc.o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
	 $(CC) $(CFLAGS) STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c 
	 
stm32f4xx_hal_gpio.o: STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	 $(CC) $(CFLAGS) STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
	 
main.o: main.c
	 $(CC) $(CFLAGS) main.c
