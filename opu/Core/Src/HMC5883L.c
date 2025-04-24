#include "I2C.h"
#include "HMC5883L.h"
#include "UART.h"
#include <string.h>

//uint8_t check;
int16_t Mag_X_RAW = 0;
int16_t Mag_Y_RAW = 0;
int16_t Mag_Z_RAW = 0;


void HMC5883L_Write(uint8_t Address, uint8_t Reg, uint8_t Data){
	I2C1_Start();
	I2C1_Address(Address);
	I2C1_Write(Reg);
	I2C1_Write(Data);
	I2C1_Stop();
}

void HMC5883L_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size){
	I2C1_Start();
	I2C1_Address(Address);
	I2C1_Write(Reg);
	I2C1_Start();
	I2C1_Read(Address+0x01, buffer, size);
	I2C1_Stop();
}

void HMC5883L_Init (void)
{
	uint8_t check;
	//uint8_t Data;

	// check device ID WHO_AM_I

	HMC5883L_Read (HMC5883L_ADDR,0x00, &check, 1);
	if(check==16){
		HMC5883L_Write(HMC5883L_ADDR, HMC5883L_CFG_A, 0b01011000);
		HMC5883L_Write(HMC5883L_ADDR, HMC5883L_CFG_B, 0b00000000);
		HMC5883L_Write(HMC5883L_ADDR, HMC5883L_MODER, 0b00000000);
	}


}

void HMC5883L_Read_Mag(float *mag_data){
	uint8_t Rx_data[6];

		// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HMC5883L_Read (HMC5883L_ADDR, HMC5883L_OUT_X_MSB, Rx_data, 6);



	Mag_X_RAW = -(int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Mag_Z_RAW = -(int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Mag_Y_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);



	mag_data[0] = Mag_X_RAW/1.370;
	mag_data[1] = Mag_Y_RAW/1.370;
	mag_data[2] = Mag_Z_RAW/1.370;
}

void HMC5883L_Read_Mag_RAW(uint8_t *mag_data_raw){
	HMC5883L_Read (HMC5883L_ADDR, HMC5883L_OUT_X_MSB, mag_data_raw, 6);
}
