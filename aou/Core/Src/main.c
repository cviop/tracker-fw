#include "stm32f4xx.h"
#include <stdint.h>

/* Definitions for message format and timeout */
#define MESSAGE_LENGTH      14         // 1 header byte + 1 msg type + 3 floats (4 bytes each)
#define HEADER_BYTE         0xAA       // 10101010 in binary
#define MESSAGE_TIMEOUT_MS  10         // Timeout in milliseconds (adjust as needed)

#define BAUD_RATE 115200
#define UART_CLOCK 42000000

#define PI 3.1415
#define DegToRad 0.01745329

/* Global timing variables */
volatile uint32_t ms_ticks = 0;
volatile uint32_t loop_run = 0;         // Incremented by SysTick every 1 ms

/* LED timers:
   led_timer is used for the LED on PB4 (for valid message reception),
   pb7_led_timer is used for the LED on PB7 (for discarded messages).
*/
volatile uint32_t led_timer = 0;
volatile uint32_t pb7_led_timer = 0;

/* Reception variables */
volatile uint8_t rx_buffer[MESSAGE_LENGTH];
volatile uint8_t msg_type = 0;
volatile uint8_t rx_index = 0;
volatile uint8_t data_ready = 0;
volatile uint32_t last_rx_tick = 0;



/* Received angles */
volatile float yaw = 0, pitch = 0, roll = 0;

/* Function prototypes */
void SystemClock_Config(void);
void GPIO_Config(void);
void UART1_Config(void);
void UART2_Config(void);
void send_motor_angle_command(uint8_t motor_id, float command);
void send_motor_stop_command(uint8_t motor_id);
void send_motor_start_command(uint8_t motor_id);
void send_motor_err_clear_command(uint8_t motor_id);

/* SysTick Interrupt Handler:
   - Increments the millisecond counter.
   - Signals the main loop using loop_run.
   - Manages the two LED timers.
*/
void SysTick_Handler(void)
{
    ms_ticks++;
    loop_run=1;

    /* Manage LED on PB4 */
    if (led_timer > 0)
    {
        led_timer--;
        if (led_timer == 0)
        {
            /* Turn LED PB4 off by resetting bit (16 + 4) in BSRR */
            GPIOB->BSRR = (1 << (16 + 4));
        }
    }

    /* Manage LED on PB7 for discarded message */
    if (pb7_led_timer > 0)
    {
        pb7_led_timer--;
        if (pb7_led_timer == 0)
        {
            /* Turn LED PB7 off by resetting bit (16 + 7) in BSRR */
            GPIOB->BSRR = (1 << (16 + 7));
        }
    }
}

/* USART1 IRQ Handler:
   - Waits for the header (HEADER_BYTE).
   - Once header is received, subsequent bytes are stored.
   - Updates last_rx_tick for each byte.
   - When the full 13-byte message is received, it decodes yaw, pitch, and roll.
*/
void USART1_IRQHandler(void)
{
    if (USART1->SR & USART_SR_RXNE)
    {
        uint8_t byte = USART1->DR & 0xFF;

        if (rx_index == 0)
        {
            if (byte == HEADER_BYTE)
            {
                rx_buffer[0] = byte;
                rx_index = 1;
                last_rx_tick = ms_ticks;
            }
            else{
            	GPIOB->BSRR = (1 << 7);  // Turn LED PB7 on
            	pb7_led_timer = 500;
            }
            // If not the header, ignore the byte.
        }
        else
        {
            /* Store subsequent bytes and update the timeout tick */
            rx_buffer[rx_index++] = byte;
            last_rx_tick = ms_ticks;

            if (rx_index >= MESSAGE_LENGTH)
            {

				/* Indicate that a full message is ready */
				data_ready = 1;
				rx_index = 0;

				/* Turn on LED on PB4 for 500 ms (valid message indicator) */
				GPIOB->BSRR = (1 << 4);
				led_timer = 500;

            }
        }
    }
}

/* Function to send a 5-byte packet on USART2:
   - First byte: motor ID header.
   - Next 4 bytes: float command.
*/
void send_motor_angle_command(uint8_t motor_id, float command)
{
    uint8_t msg[6];
    msg[0] = motor_id;
    msg[1] = 1; //msg type for angle set
    union { float f; uint8_t b[4]; } conv;
    conv.f = command;
    msg[2] = conv.b[0];
    msg[3] = conv.b[1];
    msg[4] = conv.b[2];
    msg[5] = conv.b[3];

    for (int i = 0; i < 6; i++)
    {
        while (!(USART2->SR & USART_SR_TXE));  // Wait for TX register to be empty
        USART2->DR = msg[i];
    }
}

void send_motor_stop_command(uint8_t motor_id){
	uint8_t msg[6];
	msg[0] = motor_id;
	msg[1] = 4; //msg type for angle set
	for (int i = 0; i < 6; i++)
	{
		while (!(USART2->SR & USART_SR_TXE));  // Wait for TX register to be empty
		USART2->DR = msg[i];
	}
}

void send_motor_start_command(uint8_t motor_id){
	uint8_t msg[6];
	msg[0] = motor_id;
	msg[1] = 5; //msg type for angle set
	for (int i = 0; i < 6; i++)
	{
		while (!(USART2->SR & USART_SR_TXE));  // Wait for TX register to be empty
		USART2->DR = msg[i];
	}
}

void send_motor_err_clear_command(uint8_t motor_id){
	uint8_t msg[6];
	msg[0] = motor_id;
	msg[1] = 6; //msg type for angle set
	for (int i = 0; i < 6; i++)
	{
		while (!(USART2->SR & USART_SR_TXE));  // Wait for TX register to be empty
		USART2->DR = msg[i];
	}
}

/* Main function */
int main(void)
{
    /* Configure system clock, GPIOs, and UARTs */
    SystemClock_Config();
    GPIO_Config();
    UART1_Config();
    UART2_Config();

    /* Configure SysTick to generate an interrupt every 1 ms */
    SysTick_Config(SystemCoreClock / 1000);

    while (1)
    {
        if (loop_run)
        {
            loop_run=0;  // Consume the 1 ms tick

            /* Check for partial message timeout.
               If we have started receiving a message but no new byte arrived within
               MESSAGE_TIMEOUT_MS ms then discard the partial message.
            */
            if ((rx_index != 0) && ((ms_ticks - last_rx_tick) > MESSAGE_TIMEOUT_MS))
            {
                rx_index = 0;  // Discard the partial message

                /* Blink LED on PB7 for 500 ms to indicate message discard */
                GPIOB->BSRR = (1 << 7);  // Turn LED PB7 on
                pb7_led_timer = 500;
            }

            /* When a complete message is received, process and send motor commands.
               Here, the commands are computed by direct multiplication:
               - Motor 1: yaw * 25
               - Motor 2: (pitch * 25) + (roll * 75)
               - Motor 3: (-pitch * 25) + (roll * 75)
            */
            if (data_ready)
            {
            	msg_type = rx_buffer[1];

				switch(msg_type){
				case 1: //angles setpoint
					/* Complete message received. Decode angles from bytes 1..12 */
					union { uint8_t b[4]; float f; } conv;

					/* Yaw: bytes 1 to 4 */
					conv.b[0] = rx_buffer[2];
					conv.b[1] = rx_buffer[3];
					conv.b[2] = rx_buffer[4];
					conv.b[3] = rx_buffer[5];
					yaw = conv.f;

					/* Pitch: bytes 5 to 8 */
					conv.b[0] = rx_buffer[6];
					conv.b[1] = rx_buffer[7];
					conv.b[2] = rx_buffer[8];
					conv.b[3] = rx_buffer[9];
					pitch = conv.f;

					/* Roll: bytes 9 to 12 */
					conv.b[0] = rx_buffer[10];
					conv.b[1] = rx_buffer[11];
					conv.b[2] = rx_buffer[12];
					conv.b[3] = rx_buffer[13];
					roll = conv.f;

					/*
					float motor1 = yaw * 25.0f;
					float motor2 = (pitch * 25.0f) + (roll * 75.0f);
					float motor3 = (-pitch * 25.0f) + (roll * 75.0f);
					*/

					float motor1 = yaw;
					float motor2 = pitch;
					float motor3 = roll;

					send_motor_angle_command(1, motor1*DegToRad);
					send_motor_angle_command(2, motor2*DegToRad);
					send_motor_angle_command(3, motor3*DegToRad);
					break;
				case 4: //motor stop = 1[HEADER|msg_type|1/0|x|x|x|1/0|x|x|x|1/0|x|x|x|]
					uint8_t turn_off_motor_1 = rx_buffer[2];
					uint8_t turn_off_motor_2 = rx_buffer[6];
					uint8_t turn_off_motor_3 = rx_buffer[10];

					if(turn_off_motor_1)
						send_motor_stop_command(1);
					if(turn_off_motor_2)
						send_motor_stop_command(2);
					if(turn_off_motor_3)
						send_motor_stop_command(3);
					break;
				case 5:
					uint8_t turn_on_motor_1 = rx_buffer[2];
					uint8_t turn_on_motor_2 = rx_buffer[6];
					uint8_t turn_on_motor_3 = rx_buffer[10];

					if(turn_on_motor_1)
						send_motor_start_command(1);
					if(turn_on_motor_2)
						send_motor_start_command(2);
					if(turn_on_motor_3)
						send_motor_start_command(3);
					break;
				case 6:
					uint8_t clear_err_motor_1 = rx_buffer[2];
					uint8_t clear_err_motor_2 = rx_buffer[6];
					uint8_t clear_err_motor_3 = rx_buffer[10];

					if(clear_err_motor_1)
						send_motor_err_clear_command(1);
					if(clear_err_motor_2)
						send_motor_err_clear_command(2);
					if(clear_err_motor_3)
						send_motor_err_clear_command(3);
					break;
				}



                data_ready = 0;  // Clear the flag once processed
            }
        }
    }

    return 0;
}

/* GPIO configuration:
   - Enables clocks for GPIOA (for USART1/2) and GPIOB (for LEDs).
   - Configures PA9/PA10 for USART1 (AF7).
   - Configures PA2/PA3 for USART2 (AF7).
   - Configures PB4 as an output for the valid-message LED.
   - Configures PB7 as an output for the discard-indicator LED.
*/
void GPIO_Config(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    /* Configure PA9 and PA10 for USART1 (Alternate Function 7) */
    GPIOA->MODER &= ~((3UL << (9 * 2)) | (3UL << (10 * 2)));
    GPIOA->MODER |= ((2UL << (9 * 2)) | (2UL << (10 * 2)));
    GPIOA->AFR[1] &= ~((0xF << ((9 - 8) * 4)) | (0xF << ((10 - 8) * 4)));
    GPIOA->AFR[1] |= ((7 << ((9 - 8) * 4)) | (7 << ((10 - 8) * 4)));

    /* Configure PA2 and PA3 for USART2 (Alternate Function 7) */
    GPIOA->MODER &= ~((3UL << (2 * 2)) | (3UL << (3 * 2)));
    GPIOA->MODER |= ((2UL << (2 * 2)) | (2UL << (3 * 2)));
    GPIOA->AFR[0] &= ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    GPIOA->AFR[0] |= ((7 << (2 * 4)) | (7 << (3 * 4)));

	GPIOA->OSPEEDR |= (3<<4) | (3<<6); //3 ~ 0b11


    /* Configure PB4 as a general-purpose output (for valid message LED) */
    GPIOB->MODER &= ~(3UL << (4 * 2));
    GPIOB->MODER |= (1UL << (4 * 2));

    /* Configure PB7 as a general-purpose output (for discard-indicator LED) */
    GPIOB->MODER &= ~(3UL << (7 * 2));
    GPIOB->MODER |= (1UL << (7 * 2));
}

/* USART1 configuration:
   - Enables clock for USART1.
   - Sets baud rate (~115200 baud at 84 MHz).
   - Enables the receiver and RXNE interrupt.
   - Enables the NVIC for USART1.
*/
void UART1_Config(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    //USART1->BRR = 730;  // Approximate value for 115200 baud at 84 MHz
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
    USART1->BRR = (5U << 4) | (11U << 0);
    NVIC_EnableIRQ(USART1_IRQn);
}

/* USART2 configuration:
   - Enables clock for USART2.
   - Sets baud rate (~115200 baud).
   - Enables the transmitter.
*/
void UART2_Config(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    //uint16_t brr = (UART_CLOCK + (BAUD_RATE/2)) / BAUD_RATE;
    //USART2->BRR = (uint32_t)brr;  // Approximate value for 115200 baud
    USART2->BRR = (13<<0) | (23<<4);   // Baud rate of 115200, PCLK1 at 42MHz
	USART2->CR1 |= (1<<2); //RE=1 recieve enable
	USART2->CR1 |= (1<<3); //TE=1
    USART2->CR1 |= (1<<13); //USART2 enable
}

/* System Clock Configuration using HSI and PLL:
   - Enables HSI and waits until it is ready.
   - Configures the PLL (PLLM = 16, PLLN = 336, PLLP = 4, PLLQ = 7) for an 84 MHz clock.
   - Sets Flash latency and switches the system clock to the PLL.
   - Configures AHB/APB prescalers.
*/
void SystemClock_Config(void)
{
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY));

    /* Disable PLL for configuration */
    RCC->CR &= ~RCC_CR_PLLON;

    RCC->PLLCFGR = (16)              |  // PLLM = 16
                   (336 << 6)        |  // PLLN = 336
                   (1 << 16)         |  // PLLP = 4 (01 in bits 17:16)
                   (7 << 24);           // PLLQ = 7

    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));

    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_2WS;

    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);

    /* Set AHB = 1, APB1 = /2, APB2 = 1 */
    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
}
