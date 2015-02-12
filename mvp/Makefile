CC = arm-none-eabi-gcc
LD = arm-none-eabi-gcc
AR = arm-none-eabi-ar
AS = arm-none-eabi-as
OBJCOPY = arm-none-eabi-objcopy
OD = arm-none-eabi-objdump
SIZE = arm-none-eabi-size

OCFLAGS = --strip-unneeded

CFLAGS = -I./CMSIS/Include/
CFLAGS += -I./STM32F4xx_HAL_Driver/Inc
CFLAGS += -I./CMSIS/Device/ST/STM32F4xx/Include/
CFLAGS += -I./ -c -fno-common -Os -g -mcpu=cortex-m4 -mthumb -Wall -ffunction-sections -fdata-sections -fno-builtin -Wno-unused-function -ffreestanding
LFLAGS = -Tstm32f4.ld -nostartfiles -Wl,--gc-sections

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

all: main.bin

clean:
	-rm -f $(OBJS) main.lst main.elf main.hex main.map main.bin main.list

main.elf: $(OBJS) stm32f4.ld
	$(LD) $(LFLAGS) -o main.elf $(OBJS)
	-@echo ""
	$(SIZE) main.elf
	-@echo ""

%.o : %.c
	$(CC) $(CFLAGS) -o $@ $<

%.bin: %.elf
	$(OBJCOPY) $(OCFLAGS) -O binary $< $@
