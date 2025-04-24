/*
 * Delay.c
 *
 *  Created on: Apr 3, 2024
 *      Author: kubic
 */


/*
 * Delay.c
 *
 *  Created on: Apr 2, 2024
 *      Author: kubic
 */


#include "Delay.h"
#include "RccConfig.h"

void TIM5Config(void){

	RCC->APB1ENR |= (1<<3);

	TIM5->PSC = 84-1;
	TIM5->ARR = 0xffff;

	//Enable timer
	TIM5->CR1 |= (1<<0);
	while(!(TIM5->SR & (1<<0))); 	//This bit is set by hardware on an update event. It is cleared by software.
									//0: No update occurred.
									//1: Update interrupt pending. This bit is set by hardware when the registers are updated:
}

void Delay_us (uint32_t us){
	TIM5->CNT = 0;
	while(TIM5->CNT < us);
}

void Delay_ms(uint16_t ms){
	for(uint16_t i=0;i<ms;i++)
	Delay_us(1000);
}

void TIM5Enable(void){
	TIM5->CR1 |= (1<<0);
}
void TIM5Disable(void){
	TIM5->CR1 &= ~(1<<0);
}

void TIM5InterruptEnable(){

}
