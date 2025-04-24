

#include "data_update_watchdog.h"



void update_wd_config(uint16_t psc, uint16_t arr){

	//Enable clock of TIM3
	RCC->APB1ENR |= 1<<1;
	TIM3->PSC = psc;
	TIM3->ARR = arr-1;
	//! 5. Enable auto reload buffering
	TIM3->CR1 |= TIM_CR1_ARPE;

	//! 6. Only counter overflow generate an interrupt
	TIM3->CR1 |= TIM_CR1_URS;

	//! 7. Transfer the content of the preload registers to buffers
	TIM3->EGR |= TIM_EGR_UG;

	//! 8. Enable the update interrupt
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->SR &= ~TIM_SR_UIF;    // Clear the interrupt

	//! 9. Configure the NVIC to run a callback function when interrupt occur
		/* Set interrupt priority */
	IRQn_Type IRQn = TIM3_IRQn;
	uint32_t prioritygroup = 0x00U;
	uint32_t PreemptPriority = 0;
	uint32_t SubPriority = 0;
	prioritygroup = NVIC_GetPriorityGrouping();
	NVIC_SetPriority(IRQn, NVIC_EncodePriority(prioritygroup, PreemptPriority, SubPriority));

	/* Enable interrupt */
	NVIC_EnableIRQ(IRQn);

	//! Optional: Stops the timer when debug is halted
	DBGMCU->APB1FZ |= DBGMCU_APB1_FZ_DBG_TIM3_STOP;
}

void update_wd_enable(uint8_t state){

	if (state == 0)
		TIM3->CR1 &= ~TIM_CR1_CEN;
	else
		TIM3->CR1 |= TIM_CR1_CEN;
}
