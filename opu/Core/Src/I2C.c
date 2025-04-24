
#include "I2C.h"
#include "RccConfig.h"


void I2C1_Config(void){

#define T_rise 0 //us
#define T_high 1 //us
#define f_pclk1 42 //MHz

	RCC->APB1ENR |= (1<<21); //I2C clock begin

	RCC->AHB1ENR |= (1<<1);

	GPIOB->MODER |= (1<<17);
	GPIOB->MODER |= (1<<19);

	GPIOB->OTYPER |= (1<<8); //Piny musí být OD
	GPIOB->OTYPER |= (1<<9);

	GPIOB->OSPEEDR |= (3<<16);
	GPIOB->OSPEEDR |= (3<<18);

	GPIOB->PUPDR &= ~(1<<16);
	GPIOB->PUPDR &= ~(1<<17);
	GPIOB->PUPDR &= ~(1<<18);
	GPIOB->PUPDR &= ~(1<<19);

	GPIOB->AFR[1] |= (4<<0);
	GPIOB->AFR[1] |= (4<<4);

	//disable I2C
	I2C1->CR1 &= ~(1<<0); //disable I2C


	//Reset I2C
	I2C1->CR1 |= (1<<15);
	//Write 0 to take it out of reset
	I2C1->CR1 &= ~(1<<15);

	I2C1->CR2 |= (f_pclk1<<0);

	//CCR hodnota
	uint16_t ccr_value = (T_rise+T_high)*f_pclk1;
	I2C1->CCR |= (ccr_value<<0);

	//Trise calc
	uint8_t trise = T_rise*f_pclk1 +1;
	I2C1->TRISE |= (trise);

	//Enable I2C
	I2C1->CR1 |= (1<<0); //Enable I2C

}

void I2C1_Start(void){
	I2C1->CR1 |= (1<<10); //Enable the ACK
	I2C1->CR1 |= (1<<8); //Generate start
	while(!(I2C1->SR1 & (1<<0))); //Wait for start bit to set
}

void I2C1_Write(uint8_t data){
	while(!(I2C1->SR1 & (1<<7))); //Wait for TXEnable is set
	I2C1->DR = data;
	while(!(I2C1->SR1 & (1<<2))); //Wait for BTF (byte transfer finished)
}

void I2C1_Address(uint8_t Address){
	I2C1->DR = Address;
	while(!(I2C1->SR1 & (1<<1)));
	uint8_t temp = I2C1->SR1 | I2C1->SR2; //read SR1 and SR2 to clear the ADDR bit
}

void I2C1_Stop(void){
	I2C1->CR1 |= (1<<9);
}

void I2C1_WriteMulti(uint8_t *data, uint8_t size){
	while(!(I2C1->SR1 & (1<<7))); //Wait for TXE bit to set
	while(size){
		while(!(I2C1->SR1 & (1<<7))); //Wait for TXEnable to set
		I2C1->DR = (volatile uint32_t)*data++;
		size--;
	}
	while(!(I2C1->SR1 & (1<<2))); //Wait for BTF to set
}

void I2C1_Read(uint8_t Address, uint8_t *buffer, uint8_t size){

	int remaining = size;
	if(size == 1){
		//Write the slave address
		I2C1->DR = Address;
		while(!(I2C1->SR1 & (1<<1))); //Wait for addr bit to set

		//Clear the ACK
		I2C1->CR1 &= ~(1<<10);
		uint8_t temp = I2C1->SR1 | I2C1->SR2; //Read SR1 and SR2 to clear the addr bit
		I2C1->CR1 |= (1<<9); //Stop the I2C

		//Wait for RXbuffer Not empty bit to set
		while(!(I2C1->SR1 & (1<<6)));

		//Read the data from the DATA register
		buffer[size-remaining] = I2C1->DR;
	}
	else{
		I2C1->DR = Address;
		while(!(I2C1->SR1 & (1<<1))); //Wait for addr bit to set
		uint8_t temp = I2C1->SR1 | I2C1->SR2; //Read SR1 and SR2 to clear the addr bit

		while(remaining > 2){
			//Wait for RXbuffer Not empty bit to set
			while(!(I2C1->SR1 & (1<<6)));

			buffer[size-remaining] = I2C1->DR; //Copy data to bufffffer

			I2C1->CR1 |= 1<<10; //Set ack bit to acknowledge the data received

			remaining--;
		}
		//Read the second last byte
		while(!(I2C1->SR1 & (1<<6))); //Wait for RXNE bit to set, bit is set when input buffer is not empty
		buffer[size-remaining] = I2C1->DR;

		I2C1->CR1 &= ~(1<<10); //Clear the ACK bit

		I2C1->CR1 |= (1<<9); //Stop the I2C

		remaining--;

		//Read the LAST byte
		while(!(I2C1->SR1 & (1<<6)));
		buffer[size-remaining] = I2C1->DR;
	}

}

