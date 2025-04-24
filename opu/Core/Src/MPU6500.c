/*
 * MPU6500.c
 *
 * Optimized for reduced I2C transactions and faster computation
 * Created on: Apr 22, 2025
 * Author: kubic (optimized by ChatGPT)
 */

#include "I2C.h"
#include "MPU6500.h"

// Sensitivity constants
#define ACCEL_SCALE (1.0f/16384.0f)
#define GYRO_SCALE  (1.0f/131.0f)
#define TEMP_SCALE  (1.0f/333.87f)
#define TEMP_OFFSET (21.0f)

// Combine repeated I2C bursts into one read: 14 bytes from ACCEL_XOUT_H to TEMP_OUT_L
#include "I2C.h"
#include "MPU6500.h"

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

int16_t Gyro_X_RAW = 0;
int16_t Gyro_Y_RAW = 0;
int16_t Gyro_Z_RAW = 0;

int16_t Temperature_RAW = 0;

float Ax, Ay, Az, Gx, Gy, Gz, Temperature;

uint8_t check;


void MPU_Write(uint8_t Address, uint8_t Reg, uint8_t Data){
	I2C1_Start();
	I2C1_Address(Address);
	I2C1_Write(Reg);
	I2C1_Write(Data);
	I2C1_Stop();
}

void MPU_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size){
	I2C1_Start();
	I2C1_Address(Address);
	I2C1_Write(Reg);
	I2C1_Start();
	I2C1_Read(Address+0x01, buffer, size);
	I2C1_Stop();
}

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	// check device ID WHO_AM_I

	MPU_Read (MPU6050_ADDR,WHO_AM_I_REG, &check, 1);

	if (check == 112)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		Data = 0;
		MPU_Write (MPU6050_ADDR, PWR_MGMT_1_REG, Data);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		MPU_Write(MPU6050_ADDR, SMPLRT_DIV_REG, Data);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ? 2g
		Data = 0x00;
		MPU_Write(MPU6050_ADDR, ACCEL_CONFIG_REG, Data);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ? 500 ?/s
		Data = 0b00001000;
		MPU_Write(MPU6050_ADDR, GYRO_CONFIG_REG, Data);
	}

}

/*
void MPU6050_Init(void) {
    uint8_t who = 0;
    MPU_Read(MPU6050_ADDR, WHO_AM_I_REG, &who, 1);
    if (who == 0x68) {
        // Wake up and configure
        static const uint8_t cfg[] = {
            PWR_MGMT_1_REG, 0x00,      // Wakeup
            SMPLRT_DIV_REG, 0x07,      // 1kHz / (1+7)
            ACCEL_CONFIG_REG, 0x00,    // ±2g
            GYRO_CONFIG_REG,  0x00     // ±500°/s
        };
        // Write all in one burst
        for (int i = 0; i < sizeof(cfg); i += 2) {
            MPU_Write(MPU6050_ADDR, cfg[i], cfg[i+1]);
        }
    }
}
*/
void MPU6050_Read_Accel (float *accel_data)
{

	uint8_t Rx_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, Rx_data, 6);

	Accel_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Accel_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Accel_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	accel_data[0] = -Accel_X_RAW/1670.13251783894;
	accel_data[1] = Accel_Y_RAW/1670.13251783894;
	accel_data[2] = -Accel_Z_RAW/1670.13251783894;
}

void MPU6050_Read_Accel_RAW (uint8_t *accel_data_raw){
	MPU_Read (MPU6050_ADDR, ACCEL_XOUT_H_REG, accel_data_raw, 6);

}

void MPU6050_Read_Gyro (float *gyro_data)
{

	uint8_t Rx_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	MPU_Read (MPU6050_ADDR, GYRO_XOUT_H_REG, Rx_data, 6);

	Gyro_X_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);
	Gyro_Y_RAW = (int16_t)(Rx_data[2] << 8 | Rx_data [3]);
	Gyro_Z_RAW = (int16_t)(Rx_data[4] << 8 | Rx_data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	gyro_data[0] = -Gyro_X_RAW/3752.87355810689;
	gyro_data[1] = Gyro_Y_RAW/3752.87355810689;
	gyro_data[2] = Gyro_Z_RAW/3752.87355810689;
}
void MPU6050_Read_Gyro_RAW (uint8_t *gyro_data_raw)
{
	MPU_Read (MPU6050_ADDR, GYRO_XOUT_H_REG, gyro_data_raw, 6);
}


void MPU6050_Read_Temperature (float *temp_data)
{

	uint8_t Rx_data[2];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	MPU_Read (MPU6050_ADDR, TEMP_OUT_H_REG, Rx_data, 2);

	Temperature_RAW = (int16_t)(Rx_data[0] << 8 | Rx_data [1]);


	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	temp_data[0] = (Temperature_RAW/333.87)+21;
}

void MPU6050_Read_Temperature_RAW (uint8_t *temp_data_raw)
{

	MPU_Read (MPU6050_ADDR, TEMP_OUT_H_REG, temp_data_raw, 2);
}

static inline void MPU6050_Read_AllRaw(uint8_t *buf) {
    I2C1_Start();
    I2C1_Address(MPU6050_ADDR);
    I2C1_Write(ACCEL_XOUT_H_REG);
    I2C1_Start();
    I2C1_Read(MPU6050_ADDR | 0x01, buf, 14);
    I2C1_Stop();
}

// Read and scale accel, gyro, temp in one transaction
void MPU6050_Read_All(float *accel, float *gyro) {
    uint8_t buf[14];
    MPU6050_Read_AllRaw(buf);

    // Parse raw data
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gz = (int16_t)((buf[12] << 8) | buf[13]);
    int16_t tr = (int16_t)((buf[6] << 8) | buf[7]);

    // Scale
    accel[0] = ax * ACCEL_SCALE;
    accel[1] = ay * ACCEL_SCALE;
    accel[2] = az * ACCEL_SCALE;

    gyro[0]  = gx * GYRO_SCALE;
    gyro[1]  = gy * GYRO_SCALE;
    gyro[2]  = gz * GYRO_SCALE;

    //temp[0]  = (tr * TEMP_SCALE) + TEMP_OFFSET;
}
