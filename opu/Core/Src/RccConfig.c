/*/*
 * RccConfig.c
 *
 *  Created on: Apr 2, 2024
 *      Author: kubic
 */

#include "RccConfig.h"


void SysClockConfig(void){

#define PLL_M 8
#define PLL_N 84
#define PLL_P 0 // to odpovídá PLLP = 2 (bit17:16 v PLLCFGR)

	//Main Clock
	// Enable HSE
	RCC->CR  |= RCC_CR_HSEON;
	while((RCC->CR & RCC_CR_HSERDY));

	// set the power enable clock and voltage regulator
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	PWR->CR |= PWR_CR_VOS;

	//Flash related
	FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN | FLASH_ACR_LATENCY_5WS;

	//Prescalers
	//AHB PRes
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//APB1_PRes
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	//APB2
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	//PLL (RCC_PLLCFGR register)
	RCC->PLLCFGR = (PLL_M << 0) | (PLL_N << 6) | (PLL_P << 16) | (RCC_PLLCFGR_PLLSRC_HSI);

	//Enable PLL
	RCC->CR |= RCC_CR_PLLON;
	while(!(RCC->CR & RCC_CR_PLLRDY));

	//Select clock source
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

//sysclk 84 MHz, HCLK 84 MHz, PCLK1 42 MHz, APB1 TIM a APB2 per+tim 84 MHz

}
