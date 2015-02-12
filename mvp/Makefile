CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS = arm-none-eabi-as
CP = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
  
CFLAGS = -I./CMSIS/Include/
CFLAGS += -I./STM32F4xx_HAL_Driver/Inc
CFLAGS += -I./CMSIS/Device/ST/STM32F4xx/Include/
CFLAGS += -I./ -c -fno-common -O0 -g -mcpu=cortex-m4 -mthumb
LFLAGS = -Tstm32f4.ld -nostartfiles

# our code
OBJS = main.o

# startup files
OBJS += handlers.o startup.o

# drivers/lib
OBJS += STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.o
OBJS += STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.o
OBJS += STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.o
OBJS += STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.o
OBJS += CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.o

all: main.elf

clean:
	-rm -f $(OBJS) main.lst main.elf main.hex main.map main.bin main.list

main.elf: $(OBJS) stm32f4.ld
	$(LD) $(LFLAGS) -o main.elf $(OBJS)

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<
