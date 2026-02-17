#ifndef UART_H
#define UART_H
#include "stm32f4xx.h"

void uart_init(uint32_t baud);
void uart_write_char(char c);
#endif