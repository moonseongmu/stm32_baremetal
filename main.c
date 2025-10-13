#include <stdint.h>
#include "stm32f4xx.h"
#include "main.h"

int main(void)
{
    clock_setup();
    //enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 
    asm("nop"); //delay for 2 cycles as per errata sheet
    asm("nop");
    
    //set mode as output for pc13
    GPIOC->MODER |= GPIO_MODER_MODER13_0; 
    
    while (1)
    {
        //toggle pc13 on
        GPIOC->ODR |= GPIO_ODR_OD13; 
        for(uint32_t i = 0; i<10000000; i++)
        {
            asm("nop");
        }
        //toggle PC13 on
        GPIOC->ODR &= ~GPIO_ODR_OD13; 
        for(uint32_t j = 0; j<10000000; j++)
        {
            asm("nop");
        }
    }
    
}

void clock_setup(void)
{
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

    SystemCoreClockUpdate(); //the and bitwise operation is broken?S
}
