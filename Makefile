CC := arm-none-eabi-gcc
CFLAGS := -o out.elf -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -Wall -Wextra -g -Og -specs=nano.specs
CSOURCES := ./main.c ./syscalls.c ./uart.c ./CMSIS/Device/Source/system_stm32f4xx.c ./CMSIS/Device/Source/startup_stm32f411xe.s
CINCLUDE := -I./CMSIS/Core/Include -I./CMSIS/Core/Include/m-profile -I./CMSIS/Device/Include
LINK := -T ./linkerscript_gcc.ld
CLANGLINK := -T ./linkerscript_clang.ld
OUTPUT := -o out.elf
CLANG := clang
CLANGFLAGS := --target=arm-none-eabi

main:
	$(CC) $(CSOURCES) $(LINK) $(CFLAGS) $(CINCLUDE) $(OUTPUT)

clang:
	$(CLANG) $(CSOURCES) $(CLANGLINK) $(CFLAGS) $(CLANGFLAGS) $(CINCLUDE) $(OUTPUT)

.PHONY: clean flash

clean: 
	-del *.elf 
	-del *.bin 
	-del *.hex

flash: 
	pyocd load out.elf