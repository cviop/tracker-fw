#include "stm32f4xx.h"
#include <string.h>
#include <stdint.h>

#define BAUD                    921600U
#define LED0_PIN                (1U << 4)  // PA4
#define LED1_PIN                (1U << 5)  // PA5
#define LED2_PIN                (1U << 6)  // PA6
#define LED3_PIN                (1U << 7)  // PA7
#define SW_PINS                 ((1U<<0)|(1U<<1)|(1U<<2))  // PC0, PC1, PC2
#define TC_address				0xA8

// USART2 parser states
typedef enum {
    WAIT_FOR_HEADER = 0,
    WAIT_FOR_MSG_TYPE,
    RECEIVE_DATA
} usart2_state_t;

// Flags to trigger actions
volatile uint8_t gps_data_to_TC           = 0;  // type 1
volatile uint8_t OPU_angles_to_TC         = 0;  // type 2
volatile uint8_t OPU_raw_data_to_TC       = 0;  // type 3
volatile uint8_t gyro_cal_to_OPU          = 0;  // type 10 ready
volatile uint8_t mag_cal_ready_to_OPU     = 0;  // type 11 ready
volatile uint8_t AOU_send_angles		  = 0;  // type 12
volatile uint8_t AOU_stop_motors		  = 0;
volatile uint8_t AOU_start_motors		  = 0;
volatile uint8_t AOU_clear_errors		  = 0;
volatile uint8_t read_and_send_switches   = 0;  // type 8

// Data buffers
uint8_t buf6[100];
uint8_t buf1[12];
uint8_t buf36[36];
uint8_t buf12[12];
uint8_t buf48[48];

// Function prototypes
void SystemClock_Config(void);
static uint32_t getPCLK(uint8_t bus);
void init_USART1(void);
void init_USART2(void);
void init_USART6(void);
void UARTs_Config(void);
void EXTI_Config(void);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void USART2_IRQHandler(void);
void USART1_SendByte(uint8_t byte);
void USART2_SendByte(uint8_t byte);

void USART1_SendArray(uint8_t *out_array, uint32_t numBytes);
void USART2_SendArray(uint8_t *out_array, uint32_t numBytes);

void USART1_SendByteToAddr(uint8_t out_addr, uint8_t out_byte);
void USART1_SendDataToAddr(uint8_t out_addr, uint8_t out_msg_type, uint8_t *out_array, uint32_t numBytes);

int main(void) {
    SystemClock_Config();

    // Initialize LEDs PA4-PA7 as outputs, off
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~((3U<<(4*2))|(3U<<(5*2))|(3U<<(6*2))|(3U<<(7*2)));
    GPIOA->MODER |= ((1U<<(4*2))|(1U<<(5*2))|(1U<<(6*2))|(1U<<(7*2)));
    GPIOA->ODR &= ~(LED0_PIN|LED1_PIN|LED2_PIN|LED3_PIN);

    UARTs_Config();
    EXTI_Config();

    while (1) {
        /*if (gps_data_to_TC) {
            uint8_t hdr6[8] = {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19};
            for (int i = 0; i < 8; i++) {
                while (!(USART6->SR & USART_SR_TXE)) {}
                USART6->DR = hdr6[i];
            }
            uint32_t cnt = 0, timeout = 8400000U;
            while (cnt < 100 && timeout--) {
                if (USART6->SR & USART_SR_RXNE) {
                    buf6[cnt++] = USART6->DR;
                }
            }
            if (cnt == 100) {
                for (uint32_t i = 0; i < 100; i++) {
                    while (!(USART2->SR & USART_SR_TXE)) {}
                    USART2->DR = buf6[i];
                }
            }
            gps_data_to_TC = 0;
        }*/

    	if (gps_data_to_TC) {
    	    /* send UBX header out on USART6 */
    	    uint8_t hdr6[8] = {0xB5,0x62,0x01,0x07,0x00,0x00,0x08,0x19};
    	    for (int i = 0; i < 8; i++) {
    	        while (!(USART6->SR & USART_SR_TXE)) {}
    	        USART6->DR = hdr6[i];
    	    }

    	    /* read exactly 100 bytes back from USART6 into buf6 */
    	    uint32_t cnt = 0, timeout = 8400000U;
    	    while (cnt < 100 && timeout--) {
    	        if (USART6->SR & USART_SR_RXNE) {
    	            buf6[cnt++] = (uint8_t)USART6->DR;
    	        }
    	    }

    	    if (cnt == 100) {
    	        /* compute UBX Fletcher checksum over buf6[2]..buf6[97] */
    	        uint8_t ck_a = 0, ck_b = 0;
    	        for (int i = 2; i < 100 - 2; i++) {
    	            ck_a = (uint8_t)(ck_a + buf6[i]);
    	            ck_b = (uint8_t)(ck_b + ck_a);
    	        }

    	        /* compare to checksum bytes buf6[98], buf6[99] */
    	        if (ck_a == buf6[98] && ck_b == buf6[99]) {
    	            /* CRC OK: forward original packet */
    	            for (int i = 0; i < 100; i++) {
    	                while (!(USART2->SR & USART_SR_TXE)) {}
    	                USART2->DR = buf6[i];
    	            }
    	        }
    	        else {
    	            /* CRC failed: zero out buf6 and send 100 zeros */
    	            memset(buf6, 0, 100);
    	            for (int i = 0; i < 100; i++) {
    	                while (!(USART2->SR & USART_SR_TXE)) {}
    	                USART2->DR = buf6[i];
    	            }
    	        }
    	    }

    	    gps_data_to_TC = 0;
    	}


        // OPU angles over USART1
		if (OPU_angles_to_TC) {
			uint8_t cmd2[2] = {0xA9,1};
			// send command
			for (int i = 0; i < 2; i++) {
				while (!(USART1->SR & USART_SR_TXE)) {}
				USART1->DR = cmd2[i];
			}
			// wait for header 0xA8,0x01
			uint8_t prev = 0, curr;
			uint32_t timeout1 = 8400000U;
			while (timeout1--) {
				if (USART1->SR & USART_SR_RXNE) {
					curr = USART1->DR;
					if (prev == TC_address && curr == 0x01) break;
					prev = curr;
				}
			}
			// if header found, read up to 12 bytes, else zero-fill
			uint32_t cnt2 = 0;
			if (timeout1) {
				while (cnt2 < 12 && timeout1--) {
					if (USART1->SR & USART_SR_RXNE) buf1[cnt2++] = USART1->DR;
				}
			}
			// zero-fill remainder
			for (uint32_t i = cnt2; i < 12; i++) buf1[i] = 0;
			// forward via USART2
			for (uint32_t i = 0; i < 12; i++) {
				while (!(USART2->SR & USART_SR_TXE)) {}
				USART2->DR = buf1[i];
			}
			OPU_angles_to_TC = 0;
		}

        if (OPU_raw_data_to_TC) {
            /*uint8_t cmd[2] = {0xA9, 2};
            while (!(USART1->SR & USART_SR_TXE)) {}
            USART1->DR = cmd[0];
            while (!(USART1->SR & USART_SR_TXE)) {}
            USART1->DR = cmd[1];*/
            USART1_SendByteToAddr(0xA9, 2);

            // wait for header 0xA8,0x01
			uint8_t prev = 0, curr;
			uint32_t timeout1 = 8400000U;
			while (timeout1--) {
				if (USART1->SR & USART_SR_RXNE) {
					curr = USART1->DR;
					if (prev == TC_address && curr == 0x01) break;
					prev = curr;
				}
			}
            for (uint32_t i = 0; i < 36; i++) {
                while (!(USART1->SR & USART_SR_RXNE)) {}
                buf36[i] = USART1->DR;
            }

            USART2_SendArray(buf36, 36);
            OPU_raw_data_to_TC = 0;
        }


        if (gyro_cal_to_OPU) {
            gyro_cal_to_OPU = 0;
            USART1_SendDataToAddr(0xA9, 3, buf12, 12);
        }

        if (mag_cal_ready_to_OPU) {
        	mag_cal_ready_to_OPU = 0;
        	USART1_SendDataToAddr(0xA9, 4, buf48, 48);
        }

        if (AOU_send_angles) {
        	AOU_send_angles = 0;
        	USART1_SendDataToAddr(0xAA, 1, buf12, 12);
        }
        if (AOU_stop_motors) {
        	AOU_stop_motors = 0;
        	USART1_SendDataToAddr(0xAA, 4, buf12, 12);
        }
        if (AOU_start_motors) {
        	AOU_start_motors = 0;
        	USART1_SendDataToAddr(0xAA, 5, buf12, 12);
        }
        if (AOU_clear_errors) {
        	AOU_clear_errors = 0;
        	USART1_SendDataToAddr(0xAA, 6, buf12, 12);
        }
        if (read_and_send_switches){
        	read_and_send_switches = 0;
        	uint8_t sw0 = !(GPIOC->IDR & (1U<<0));
        	uint8_t sw1 = !(GPIOC->IDR & (1U<<1));
        	uint8_t sw2 = !(GPIOC->IDR & (1U<<2));
        	USART2_SendByte(sw0);
        	USART2_SendByte(sw1);
			USART2_SendByte(sw2);

        }
    }
}

// EXTI handlers
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1U<<0)) {
        EXTI->PR = (1U<<0);
        if (!(GPIOC->IDR & (1U<<0))) GPIOA->ODR |= LED0_PIN;
        else GPIOA->ODR &= ~LED0_PIN;
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U<<1)) {
        EXTI->PR = (1U<<1);
        if (!(GPIOC->IDR & (1U<<1))) GPIOA->ODR |= LED1_PIN;
        else GPIOA->ODR &= ~LED1_PIN;
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & (1U<<2)) {
        EXTI->PR = (1U<<2);
        if (!(GPIOC->IDR & (1U<<2))) GPIOA->ODR |= LED2_PIN;
        else GPIOA->ODR &= ~LED2_PIN;
    }
}

// USART2 IRQ: parse and trigger flags or reset
void USART2_IRQHandler(void) {
    static usart2_state_t state = WAIT_FOR_HEADER;
    static uint8_t msg_type;
    static uint32_t count;

    if (USART2->SR & USART_SR_RXNE) {
        uint8_t b = USART2->DR;
        switch (state) {
            case WAIT_FOR_HEADER:
                if (b == TC_address) state = WAIT_FOR_MSG_TYPE;
                break;
            case WAIT_FOR_MSG_TYPE:
                msg_type = b;
                if (msg_type == 0x12) {
                    // reset immediately
                    NVIC_SystemReset();
                } else if (msg_type == 10 || msg_type == 11 || msg_type == 12 ||
                			msg_type == 13 || msg_type == 14 ||  msg_type == 15) {
                    count = 0;
                    state = RECEIVE_DATA;
                } else {
                    switch (msg_type) {
                        case 1:  gps_data_to_TC         = 1; break;
                        case 2:  OPU_angles_to_TC       = 1; break;
                        case 3:  OPU_raw_data_to_TC     = 1; break;
                        case 4: //reset OPU
                        	while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0xA9;
							while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0x12;
                        	break;
                        case 5:
                        	while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0xA9;
							while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0x11;
							break;
                        case 6: break;
                        case 7: break;
                        case 8: // read switches
							read_and_send_switches = 1;
                        	break;
                        case 9: // reset AOU, TODO neni implementovano
                        	while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0xAA;
							while (!(USART1->SR & USART_SR_TXE)) {}
							USART1->DR = 0x12;
							break;
                        case 255:
                        	USART2_SendByte(TC_address);
                        	break;

                        default: break;
                    }
                    state = WAIT_FOR_HEADER;
                }
                break;
            case RECEIVE_DATA:
            	switch(msg_type){
            	case 10: //receive gyro cal data
            		buf12[count++] = b;
					if (count >= 12) {
						gyro_cal_to_OPU = 1;
						state = WAIT_FOR_HEADER;
					}
					break;

            	case 11: //receive mag cal data
            		buf48[count++] = b;
					if (count >= 48) {
						mag_cal_ready_to_OPU = 1;
						state = WAIT_FOR_HEADER;
					}
					break;
            	case 12: //receive data to orient tracker
            		buf12[count++] = b;
					if (count >= 12) {
						AOU_send_angles = 1;
						state = WAIT_FOR_HEADER;
					}
            		break;
            	case 13: //receive which motors to turn off
            		buf12[count++] = b;
					if (count >= 12) {
						AOU_stop_motors = 1;
						state = WAIT_FOR_HEADER;
					}
            		break;
            	case 14: //receive which motors to turn on
            		buf12[count++] = b;
					if (count >= 12) {
						AOU_start_motors = 1;
						state = WAIT_FOR_HEADER;
					}
            		break;
            	case 15: //receive which motors to turn on
					buf12[count++] = b;
					if (count >= 12) {
						AOU_clear_errors = 1;
						state = WAIT_FOR_HEADER;
					}
					break;
            	}

            	/*
                if (msg_type == 10) {
                    buf12[count++] = b;
                    if (count >= 12) {
                        gyro_cal_to_OPU = 1;
                        state = WAIT_FOR_HEADER;
                    }
                } else {
                    buf48[count++] = b;
                    if (count >= 48) {
                        mag_cal_ready_to_OPU = 1;
                        state = WAIT_FOR_HEADER;
                    }
                }
                */
                break;
        }
    }
}



// Function definitions
void SystemClock_Config(void) {
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    FLASH->ACR = FLASH_ACR_LATENCY_2WS |
                 FLASH_ACR_PRFTEN    |
                 FLASH_ACR_ICEN      |
                 FLASH_ACR_DCEN;

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;  // APB1 = 42 MHz

    RCC->PLLCFGR = (16U << RCC_PLLCFGR_PLLM_Pos) |
                   (168U << RCC_PLLCFGR_PLLN_Pos)|
                   (0U   << RCC_PLLCFGR_PLLP_Pos)|   // PLLP = 2
                   (4U   << RCC_PLLCFGR_PLLQ_Pos);    // USBOTG

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

static uint32_t getPCLK(uint8_t bus) {
    const uint32_t sysclk = 84000000U;
    uint32_t presc = (bus == 1)
        ? ((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos)
        : ((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos);
    return (presc < 4) ? sysclk : (sysclk >> (presc - 3));
}

void init_USART1(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    USART1->BRR = (5U << 4) | (11U << 0);
    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void init_USART2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    USART2->BRR = (2U << 4) | (14U << 0);
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
}

void init_USART6(void) {
    RCC->APB2ENR |= RCC_APB2ENR_USART6EN;
    USART6->BRR = (5U << 4) | (11U << 0);
    USART6->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

void UARTs_Config(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

    // USART1: PB6 = TX, PB7 = RX (AF7)
    GPIOB->MODER &= ~((3U<<(6*2)) | (3U<<(7*2)));
    GPIOB->MODER |=  ((2U<<(6*2)) | (2U<<(7*2)));
    GPIOB->AFR[0] &= ~((0xFU<<(6*4)) | (0xFU<<(7*4)));
    GPIOB->AFR[0] |=  ((7U<<(6*4)) | (7U<<(7*4)));

    // USART2: PA2 = TX, PA3 = RX (AF7)
    GPIOA->MODER &= ~((3U<<(2*2)) | (3U<<(3*2)));
    GPIOA->MODER |=  ((2U<<(2*2)) | (2U<<(3*2)));
    GPIOA->AFR[0] &= ~((0xFU<<(2*4)) | (0xFU<<(3*4)));
    GPIOA->AFR[0] |=  ((7U<<(2*4)) | (7U<<(3*4)));

    // USART6: PC6 = TX, PC7 = RX (AF8)
    GPIOC->MODER &= ~((3U<<(6*2)) | (3U<<(7*2)));
    GPIOC->MODER |=  ((2U<<(6*2)) | (2U<<(7*2)));
    GPIOC->AFR[0] &= ~((0xFU<<(6*4)) | (0xFU<<(7*4)));
    GPIOC->AFR[0] |=  ((8U<<(6*4)) | (8U<<(7*4)));

    init_USART1();
    init_USART2();
    init_USART6();

    NVIC_EnableIRQ(USART2_IRQn);
}

void USART1_SendByte(uint8_t byte){
	while (!(USART1->SR & USART_SR_TXE)) {}
	USART1->DR = byte;
}


void USART2_SendByte(uint8_t byte){
	while (!(USART2->SR & USART_SR_TXE)) {}
	USART2->DR = byte;
}


void USART1_SendArray(uint8_t *out_array, uint32_t numBytes){
	for (uint32_t i = 0; i < numBytes; i++) {
		while (!(USART1->SR & USART_SR_TXE)) {}
		USART1->DR = out_array[i];
	}
}
void USART2_SendArray(uint8_t *out_array, uint32_t numBytes){
	for (uint32_t i = 0; i < numBytes; i++) {
		while (!(USART2->SR & USART_SR_TXE)) {}
		USART2->DR = out_array[i];
	}
}

void USART1_SendDataToAddr(uint8_t out_addr, uint8_t out_msg_type, uint8_t *out_array, uint32_t numBytes){
	USART1_SendByte(out_addr);
	USART1_SendByte(out_msg_type);
	USART1_SendArray(out_array, numBytes);
}

void USART1_SendByteToAddr(uint8_t out_addr, uint8_t out_byte){
	USART1_SendByte(out_addr);
	USART1_SendByte(out_byte);
}



void EXTI_Config(void) {
    // PC0-2 inputs with pull-up
    GPIOC->MODER &= ~((3U<<0)|(3U<<2)|(3U<<4));
    GPIOC->PUPDR &= ~((3U<<0)|(3U<<2)|(3U<<4));
    GPIOC->PUPDR |= ((1U<<0)|(1U<<2)|(1U<<4));

    // Map EXTI0-2 to PC0-2
    SYSCFG->EXTICR[0] &= ~0x00000FFF;
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PC
                      | SYSCFG_EXTICR1_EXTI1_PC
                      | SYSCFG_EXTICR1_EXTI2_PC;

    // Enable both falling and rising edges
    EXTI->IMR   |= SW_PINS;
    EXTI->FTSR |= SW_PINS;
    EXTI->RTSR |= SW_PINS;

    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_EnableIRQ(EXTI2_IRQn);
}
