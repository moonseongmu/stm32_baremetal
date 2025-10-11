#include <stdint.h>
#include "stm32f4xx.h"

int main(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; //enable GPIOC clock
    asm("nop"); //delay for 2 cycles as per errata sheet
    asm("nop");
    
    GPIOC->MODER |= GPIO_MODER_MODER13_0; //set mode as output for pc13
    
    while (1)
    {
        GPIOC->ODR |= GPIO_ODR_OD13; //toggle pc13 on
        for(uint32_t i = 0; i<1000000; i++)
        {
            asm("nop");
        }
        GPIOC->ODR &= ~GPIO_ODR_OD13; //toggle PC13 on
        for(uint32_t j = 0; j<1000000; j++)
        {
            asm("nop");
        }
    }
    
}