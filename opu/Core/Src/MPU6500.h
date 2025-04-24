


#define MPU6050_ADDR 0xD2


#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


void MPU_Write(uint8_t Address, uint8_t Reg, uint8_t Data);
void MPU_Read(uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size);
void MPU6050_Init (void);
void MPU6050_Read_Accel (float *accel_data);
void MPU6050_Read_Gyro (float *gyro_data);
void MPU6050_Read_Temperature (float *temp_data);

void MPU6050_Read_Accel_RAW (uint8_t *accel_data_raw);
void MPU6050_Read_Gyro_RAW(uint8_t *gyro_data_raw);
void MPU6050_Read_Temperature_RAW(uint8_t *temp_data_raw);
