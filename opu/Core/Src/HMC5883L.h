#include <stdint.h>

#define HMC5883L_ADDR 0x3C

#define HMC5883L_CFG_A 0x00
#define HMC5883L_CFG_B 0x01
#define HMC5883L_MODER 0x02

#define HMC5883L_OUT_X_MSB 0x03
#define HMC5883L_OUT_X_LSB 0x04

#define HMC5883L_OUT_Z_MSB 0x05
#define HMC5883L_OUT_Z_LSB 0x06

#define HMC5883L_OUT_Y_MSB 0x07
#define HMC5883L_OUT_Y_LSB 0x08

#define HMC5883L_SR 0x09
#define HMC5883L_IDRA 0x10
#define HMC5883L_IDRB 0x11
#define HMC5883L_IDRC 0x12



void HMC5883L_Write(uint8_t Address, uint8_t Reg, uint8_t Data);
void HMC5883L_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size);
void HMC5883L_Init (void);
void HMC5883L_Read_Mag(float *mag_data);
void HMC5883L_Read_Mag_RAW(uint8_t *mag_data_raw);
