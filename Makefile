CC := arm-none-eabi-gcc
CFLAGS :=  -nostdlib -o out.elf -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Werror
CSOURCES := ./main.c ./CMSIS/source/startup_stm32f411xe.s ./CMSIS/source/system_stm32f4xx.c
CINCLUDE := -I./CMSIS/include
LINK := -T ./STM32F411CEUX_FLASH.ld
OUTPUT := -o out.elf

main:
	$(CC) $(CSOURCES) $(LINK) $(CFLAGS) $(CINCLUDE) $(OUTPUT)

.PHONY: clean

clean: 
	-del *.elf 
	-del *.bin 
	-del *.hex
