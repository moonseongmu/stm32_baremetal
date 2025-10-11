CC := arm-none-eabi-gcc
CFLAGS :=  -nostdlib -o out.elf -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -Werror
CSOURCES := ./main.c ./CMSIS/Device/Source/system_stm32f4xx.c ./CMSIS/Device/Source/startup_stm32f411xe.s
CINCLUDE := -I./CMSIS/Core/Include -I./CMSIS/Core/Include/m-profile -I./CMSIS/Device/Include
LINK := -T ./STM32F411CEUX_FLASH.ld
OUTPUT := -o out.elf

main:
	$(CC) $(CSOURCES) $(LINK) $(CFLAGS) $(CINCLUDE) $(OUTPUT)

.PHONY: clean flash

clean: 
	-del *.elf 
	-del *.bin 
	-del *.hex

flash: 
	pyocd load out.elf