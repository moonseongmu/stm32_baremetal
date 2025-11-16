#include <stdint.h>
#include "stm32f4xx.h"
#include "main.h"

uint32_t ticks;
int button_pressed = 0;

int main(void){
    clock_setup();
    //enable GPIOC clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; 
    asm("nop"); //delay for 2 cycles as per errata sheet
    asm("nop");

    //enable GPIOA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; 
    asm("nop"); //delay for 2 cycles as per errata sheet
    asm("nop");

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    asm("nop"); //delay 1+ppre1 cycles as per errata
    asm("nop");
    asm("nop");

    
    //set mode as output for pc13
    GPIOC->MODER |= GPIO_MODER_MODER13_0; 

    //set mode as input for interrupt for pa0
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk; 

    //set pa0 to pull up
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_0;


    SysTick_Config(100000);
    __enable_irq();

    //select pa0 as exti0 input
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;

    //unmask interrupt line 0 
    EXTI->IMR |= EXTI_IMR_MR0;

    //set falling edge detection on line 0
    EXTI->FTSR |= EXTI_FTSR_TR0;

    //set EXRI0 priotrity to 15(lowest) and exnables the interrupt line
    NVIC_SetPriority(EXTI0_IRQn, 15);
    NVIC_EnableIRQ(EXTI0_IRQn);
    
    while (1){
        if(button_pressed != 0){
            GPIOC->ODR ^= GPIO_ODR_OD13;
            button_pressed = 0;
        }

        /* //toggle pc13 on
        GPIOC->ODR |= GPIO_ODR_OD13; 
        delay_ms(1000);
        //toggle PC13 on
        GPIOC->ODR &= ~GPIO_ODR_OD13; 
        delay_ms(1000); //*/
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

    SystemCoreClockUpdate(); //the and bitwise operation is broken?S
}

void SysTick_Handler(){
    ticks++;
}

void delay_ms(uint32_t millisecs){
    uint32_t start = ticks;
    uint32_t end = start + millisecs;

    if (end < start){
        while (ticks > start){asm("nop");} // wait for ticks to wrap around to zero
    }

    while (ticks < end){asm("nop");}
}

void EXTI0_IRQHandler(void){
    //clear exti0 pending register bit
    EXTI->PR |= EXTI_PR_PR0;

    //set button pressed flag
    button_pressed = 1;

}