#include "RccConfig.h"
#include "Delay.h"
#include <stdint.h>

#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))
#define readBit(reg, bit) ((reg) & (1<<bit))

#define BAUD_RATE 921600
#define UART_CLOCK 84000000


void USART1Config(void);

void USART1_SendChar(uint8_t c);

void USART1_SendString(char *string);

void USART1_SendFloat(float f);

uint8_t USART1_GetChar(void);
