#include "UART.h"

void USART1Config(void){
    // 1) Enable peripheral clocks
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;   // USART1 on APB2
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // GPIOB

    // 2) Configure PB6=TX, PB7=RX to AF7
    GPIOB->MODER   &= ~((3<<12)|(3<<14));   // clear MODE6[13:12], MODE7[15:14]
    GPIOB->MODER   |=  (2<<12)|(2<<14);     // set AF mode
    GPIOB->OSPEEDR |=  (3<<12)|(3<<14);     // very high speed
    GPIOB->AFR[0]  &= ~((0xF<<24)|(0xF<<28));
    GPIOB->AFR[0]  |=  (7<<24)|(7<<28);     // AF7 for USART1

    // 3) Ensure USART1 is disabled before config
    USART1->CR1 = 0;

    // 4) Compute and set BRR (oversampling by 16)
    //    USARTDIV = UART_CLOCK / BAUD_RATE
    //    Mantissa = floor(USARTDIV)
    //    Fraction = round((USARTDIV - Mantissa) * 16)
    {
        uint32_t clk = UART_CLOCK;
        uint32_t baud = BAUD_RATE;
        uint32_t mant = clk / baud;
        uint32_t frac = ((clk % baud) * 16 + baud/2) / baud;  // rounding
        USART1->BRR = (5 << 4) | (11<<0);
    }

    // 5) Configure CR1: enable TE, RE, UE, RXNE interrupt
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_UE | USART_CR1_RXNEIE;
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_SendChar(uint8_t c){
	while(!(USART1->SR & USART_SR_TXE)) {}
    USART1->DR = c;
    // wait for TC (transmission complete)
    while(!(USART1->SR & USART_SR_TC));
}

uint8_t USART1_GetChar(void){
    // wait for RXNE
    while(!(USART1->SR & USART_SR_RXNE));
    return (uint8_t)USART1->DR;
}

void USART1_SendFloat(float f) {
    union {
        float    ff;
        uint8_t  b[4];
    } u;
    u.ff = f;
    /* Send in little‑endian order: LSB first */
    for (int i = 0; i < 4; i++) {
    	USART1_SendChar(u.b[i]);
    }
}
