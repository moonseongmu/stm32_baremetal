#include "uart.h"

void uart_init(uint32_t baud){
    //enable usart1 clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    asm("nop"); //delay 1+ppre1 cycles as per errata
    asm("nop");
    asm("nop");

    //set mode as alternate function for pb6 (usart1 TX)
    GPIOB->MODER |= GPIO_MODER_MODER6_1; 
    //set mode as alternate function for pb7 (usart1 RX)
    GPIOB->MODER |= GPIO_MODER_MODER7_1;
    //set alternatfe function for pb6 to af7
    GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL6_Pos); 
    //set alternatfe function for pb7 to af7
    GPIOB->AFR[0] |= (7 << GPIO_AFRL_AFSEL7_Pos);

    //uart enable
    USART1->CR1 |= USART_CR1_UE;
    //set word length to 9
    USART1->CR1 |= USART_CR1_M;
    //set number of stop bit to 1
    USART1->CR2 &= ~USART_CR2_STOP_Msk;
    //set baud to 11520
    uint16_t mantissa =  SystemCoreClock / (16 * baud);
    uint8_t fraction = (SystemCoreClock / baud) % 16;
    USART1->BRR |= (mantissa << USART_BRR_DIV_Mantissa_Pos);
    USART1->BRR |= (fraction << USART_BRR_DIV_Fraction_Pos);

    //set transmitter enable bit to send idle frame as first transmission
    USART1->CR1 |= USART_CR1_TE;
}

void uart_write_char(char c){
    while(((USART1->SR & USART_SR_TC_Msk) >> USART_SR_TC_Pos) != 1){
        asm("nop");
    }

    USART1->DR = c;
}