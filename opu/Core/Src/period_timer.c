

#include "period_timer.h"



void period_timer_config(uint16_t psc, uint16_t arr){

	//Enable clock of TIM4
	RCC->APB1ENR |= 1<<2;
	TIM4->PSC = psc;
	TIM4->ARR = arr-1;
	//! 5. Enable auto reload buffering
	TIM4->CR1 |= TIM_CR1_ARPE;

	//! 6. Only counter overflow generate an interrupt
	TIM4->CR1 |= TIM_CR1_URS;

	//! 7. Transfer the content of the preload registers to buffers
	TIM4->EGR |= TIM_EGR_UG;

	//! 8. Enable the update interrupt
	TIM4->DIER |= TIM_DIER_UIE;
	TIM4->SR &= ~TIM_SR_UIF;    // Clear the interrupt

	//! 9. Configure the NVIC to run a callback function when interrupt occur
		/* Set interrupt priority */
	IRQn_Type IRQn = TIM4_IRQn;
	uint32_t prioritygroup = 0x00U;
	uint32_t PreemptPriority = 1;
	uint32_t SubPriority = 0;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));

	/* Enable interrupt */
	NVIC_EnableIRQ(IRQn);

	//! Optional: Stops the timer when debug is halted
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM4_STOP;
}

void period_timer_enable(uint8_t state){

	if (state == 0)
		TIM4->CR1 &= ~TIM_CR1_CEN;
	else
		TIM4->CR1 |= TIM_CR1_CEN;
}
