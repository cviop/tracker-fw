
#include <stdint.h>


void I2C1_Config(void);
void I2C1_Start();
void I2C1_Write(uint8_t data);
void I2C1_Address(uint8_t Address);
void I2C1_Stop(void);
void I2C1_WriteMulti(uint8_t *data, uint8_t size);
void I2C1_Read(uint8_t Address, uint8_t *buffer, uint8_t size);
