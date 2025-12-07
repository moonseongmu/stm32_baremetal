#include <stdint.h>
#include "stm32f4xx.h"
#include "main.h"

uint32_t ticks;
uint32_t button_timestamp;
uint32_t button_expiration = 30; //button blocking time in ms
int button_pressed = 0;

int main(void){
    clock_setup();
    peripheral_init();

    SysTick_Config(100000);
    __enable_irq();

    char letter = 'e';
    while (1){
        while(((USART1->SR & USART_SR_TC_Msk) >> USART_SR_TC_Pos) != 1){
            asm("nop");
        }

        USART1->DR = letter;
    }
    
}

void clock_setup(void){
    //set flash latency to ws3
    FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
    while((FLASH->ACR & FLASH_ACR_LATENCY_3WS) != 0x3){asm("nop");}
    //power interface clock enable 
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    asm("nop"); //delay 1+ppre1 cycles as per errata
    asm("nop");
    asm("nop");

    //set power voltage scaling 1 mode
    PWR->CR |= PWR_CR_VOS_Msk;

    //set pll config registers to 0
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM_Msk;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN_Msk;
    RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP_Msk;

    //HSE as pll clck src
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE; 

    // set pllm as 12
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2; 
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_3; 

    //set plln as 96
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5; 
    RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_6; 

    //set pllp as 2

    //HSE on
    RCC->CR |= RCC_CR_HSEON;
    while(!(RCC->CR & RCC_CR_HSERDY)){asm("nop");}

    //set AHB prescaler to 1

    //set APB1 prescaler to 2
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
    for(int i = 0; i < 16; i++){asm("nop");}

    //set APB2 prescaler to 1

    //PLL on
    RCC->CR |= RCC_CR_PLLON;
    while(!(RCC->CR & RCC_CR_PLLRDY)){asm("nop");}

    //select pll as system clock
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    SystemCoreClockUpdate();
}

void peripheral_init(void){

    //enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; 
    asm("nop"); //delay for 2 cycles as per errata sheet
    asm("nop");

    //enable syscfg clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    asm("nop"); //delay 1+ppre1 cycles as per errata
    asm("nop");
    asm("nop");

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
    USART1->BRR |= (54 << USART_BRR_DIV_Mantissa_Pos);
    USART1->BRR |= (4 << USART_BRR_DIV_Fraction_Pos);

    //set transmitter enable bit to send idle frame as first transmission
    USART1->CR1 |= USART_CR1_TE;
}

void delay_ms(uint32_t millisecs){
    uint32_t start = ticks;
    uint32_t end = start + millisecs;

    if (end < start){
        while (ticks > start){asm("nop");} // wait for ticks to wrap around to zero
    }

    while (ticks < end){asm("nop");}
}

void SysTick_Handler(){
    ticks++;
}

