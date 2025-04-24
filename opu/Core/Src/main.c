


#include "RccConfig.h"
#include "Delay.h"
#include "I2C.h"
#include "MPU6500.h"
#include "HMC5883L.h"
#include "UART.h"
#include "period_timer.h"
#include "data_update_watchdog.h"


#include <stdio.h>
#include <math.h>

#define togglebit(reg, bit) ((reg) ^= (1U << (bit)))
#define PI 3.1415



/* Functions declaration */

void GPIOConfig(void);
int sgn(float num);

void parse_gyro_calib_data(float *gyro_callib_500_dps, int16_t *gyro_calib_mantisa, int8_t *gyro_calib_exponent);
void parse_mag_calib_data(float *A, float *b, int16_t *calib_mantisa, int8_t *calib_exponent);
float IntToFloat(uint8_t *bit_array);
static inline void FloatToIntBytes(float f_in, uint8_t *bit_array);
//float A[9] = {0.953695115491089,-0.00214575513728498,0.00926270131810991,  -0.00214575513728498,0.94909992276292,-0.0241767265993721,  0.00926270131810991,-0.0241767265993721,1.10549725510669};
//float b[3] = {89.9317441172301,         -96.2968409545642,          91.8197342359188};
float A[9] = {1,0,0, 0,1,0, 0,0,1};
float b[3] = {0,0,0};
float gyro_callib_500_dps[3] = {0,0,0};
float gyro_scaling_500_dps[3] = {1,1,1};

/* Interrupt handlers */

volatile uint8_t CF_update_run = 0;
volatile uint8_t run_complete = 1;
volatile uint8_t sensor_read_lag = 0;
volatile uint8_t reset_i2c = 0;
volatile uint8_t byte_count = 0;
volatile uint8_t status_reg = 0b00000000;
//b7 -
//b6 -
//b5 -
//b4 -
//b3 -
//b2 - 1 =
//b1 - 1 = using external mag calib values
//b0 - 1 = using external gyro calib values
volatile uint8_t data_output_halt = 1;

void TIM4_IRQHandler ( void )
{
	if ( TIM4->SR & TIM_SR_UIF )
	{
		TIM4->SR &= ~TIM_SR_UIF;
		//GPIOA->ODR |= 1<<5;
		CF_update_run = 1;
		//GPIOA->ODR &= ~(1<<5);
	}
}

void TIM3_IRQHandler ( void ) // watchdog
{
	if ( TIM3->SR & TIM_SR_UIF )

	{
		TIM3->SR &= ~TIM_SR_UIF;

		if(!run_complete & !data_output_halt){

			//NVIC_SystemReset();
		}
		run_complete = 0;
	}
}

volatile uint8_t uart_data_ready = 0;

volatile uint8_t msg_type = 0;
volatile uint8_t USART1_rx_byte = 0;
volatile uint8_t getting_msg_type = 1;
volatile uint8_t getting_gyro_calib_data = 0;
volatile uint8_t getting_mag_calib_data = 0;
volatile uint8_t getting_gyro_scaling_calib_data = 0;

volatile uint8_t msg_len = 0;

int16_t gyro_calib_mantisa[3] = {0,0,0};
int8_t gyro_calib_exponent[3] = {0,0,0};

int16_t gyro_scaling_calib_mantisa[3] = {0,0,0};
int8_t gyro_scaling_calib_exponent[3] = {0,0,0};

int16_t mag_calib_mantisa[12] = {0,0,0, 0,0,0, 0,0,0}; //0-8 -> A, 9-12 -> b
int8_t mag_calib_exponent[12] = {0,0,0, 0,0,0, 0,0,0}; //0-8 -> A, 9-12 -> b


volatile uint8_t external_gyro_calib = 0; // set to parse calib values in main()
volatile uint8_t external_gyro_scaling_calib = 0; // set to parse calib values in main()
volatile uint8_t external_mag_calib = 0; // set to parse calib values in main()
volatile uint8_t send_raw_data = 0;
volatile uint8_t send_status = 0;

uint8_t rx_buffer[48] = {0};


volatile uint8_t reset_calib_values = 0;
volatile uint8_t send_angles_req = 0;

void USART1_IRQHandler(void) {
    if (!(USART1->SR & USART_SR_RXNE))
        return;

    uint8_t data = (uint8_t)(USART1->DR & 0xFF);

    enum {
        WAIT_HEADER,
        GOT_TYPE,
        RECV_GYRO_CAL,
        RECV_MAG_CAL,
        RECV_GYRO_SCALE
    };
    static uint8_t state = WAIT_HEADER;
    //static uint8_t buf[48];
    static uint8_t count;
    static uint8_t msg_type;

    switch (state) {
        case WAIT_HEADER:
            if (data == 0xA9) {
                state = GOT_TYPE;
                count = 0;
            }
            break;

        case GOT_TYPE:
            msg_type = data;
            switch (msg_type) {
                case 0x01:
                    send_angles_req   = 1;
                    send_raw_data     = 0;
                    state             = WAIT_HEADER;
                    break;
                case 0x02:
                    send_raw_data     = 1;
                    state             = WAIT_HEADER;
                    break;
                case 0x03:
                    state = RECV_GYRO_CAL;
                    break;
                case 0x04:
                    state = RECV_MAG_CAL;
                    break;
                case 0x05:
                    send_status       = 1;
                    state             = WAIT_HEADER;
                    break;
                case 0x06:
                    data_output_halt  = 1;
                    send_raw_data     = 0;
                    GPIOA->ODR      &= ~(1<<5);
                    state             = WAIT_HEADER;
                    break;
                case 0x07:
                    data_output_halt  = 0;
                    state             = WAIT_HEADER;
                    break;
                case 0x10:
                    state = RECV_GYRO_SCALE;
                    break;
                case 0x11:
                    reset_calib_values = 1;
                    state             = WAIT_HEADER;
                    break;
                case 0x12:
                    NVIC_SystemReset();
                    /* no fall‑through */
                    break;
                default:
                    state = WAIT_HEADER;
                    break;
            }
            break;

        case RECV_GYRO_CAL:
        	rx_buffer[count++] = data;
            if (count == 12) {

                uart_data_ready         = 1;
                external_gyro_calib     = 1;
                state                   = WAIT_HEADER;
            }
            break;

        case RECV_MAG_CAL:
        	rx_buffer[count++] = data;
            if (count == 48) {

                status_reg           |= (1<<1);
                uart_data_ready       = 1;
                external_mag_calib    = 1;
                state                 = WAIT_HEADER;
            }
            break;

        case RECV_GYRO_SCALE:
        	rx_buffer[count++] = data;
            if (count == 12) {

                uart_data_ready              = 1;
                external_gyro_scaling_calib  = 1;
                state                        = WAIT_HEADER;
            }
            break;
    }
}




int main(void){
	uint8_t period_ms = 2;
	float T = period_ms/1000.0;

	float accel_alpha = 0.1;
	float gyro_alpha = 0.05;
	float cf_alpha = 0.01;

	float mag_data[3];
	float mag_data_callibrated[3];
	float mag_data_rearranged[3];
	float accel_data[3];
	float gyro_data[3];
	float temp_data[1];

	float mag_data_filt [3] = {0,0,0};
	float accel_data_filt [3] = {0,0,0};
	float gyro_data_filt [3] = {0,0,0};

	float phi_acc_est = 0;
	float phi_acc_est_old = 0;

	float theta_acc_est = 0;
	float theta_acc_est_old = 0;

	float psi_mag_est = 0;
	float psi_mag_est_old = 0;

	float phi_dot_est = 0;
	float theta_dot_est = 0;
	float psi_dot_est = 0;

	float rotmat[9] = {1,0,0,0,0,0,0,0,0};

	float phi_est = 0;
	float theta_est = 0;
	float psi_est = 0;

	float mag_unit[3] = {0,0,0};
	float mag_unit_stab[3] = {0,0,0};

	float rotmat_x[9] = {1,0,0,0,0,0,0,0,0};
	float rotmat_y[9] = {0,0,0,0,1,0,0,0,0};
	float rotmat_x_reversed[9] = {1,0,0,0,0,0,0,0,0};
	float east_horizon[3] = {0,0,0};
	float east_horizon_norm[3] = {0,0,0};

	float rot_x_r_mag_unit[3] = {0,0,0};


	uint8_t first_run = 1;

	uint8_t mag_data_raw[6];
	uint8_t accel_data_raw[6];
	uint8_t gyro_data_raw[6];
	uint8_t temp_data_raw[2];



	int16_t phi;
	int16_t theta;
	int16_t psi;

	SysClockConfig();
	TIM5Config();
	I2C1_Config();
	GPIOConfig();
	USART1Config();




	period_timer_config(12000, 7*period_ms);
	update_wd_config(12000, 7*100*period_ms);


	period_timer_enable(1);
	update_wd_enable(1);

	MPU6050_Init ();
	HMC5883L_Init();


	while (1)
	{
		if(external_gyro_calib){
			for(uint8_t i = 0; i<3; i++)
				gyro_callib_500_dps[i] = IntToFloat(&rx_buffer[4*i]);
			status_reg |= 1<<0;
			external_gyro_calib = 0;
		}
		if(external_gyro_scaling_calib){
			for(uint8_t i = 0; i<3; i++)
				gyro_scaling_500_dps[i] = IntToFloat(&rx_buffer[4*i]);
			status_reg |= 1<<2;
			external_gyro_scaling_calib = 0;
		}
		if(external_mag_calib){
			for (uint8_t i = 0; i < 9; i++)
				A[i] = IntToFloat(&rx_buffer[4*i]);
			for (uint8_t i = 0; i < 3; i++)
				b[i] = IntToFloat(&rx_buffer[4*(i+9)]);  // now 'b' is your magnetometer array again
			status_reg           |= (1<<1);
			external_mag_calib = 0;
		}
		if(send_status){
			send_status = 0;
			Delay_ms(2);
			USART1_SendChar(status_reg);
		}
		if(reset_calib_values){
			reset_calib_values = 0;
			A[0] = 1; A[1] = 0; A[2] = 0;
			A[3] = 0; A[4] = 1; A[5] = 0;
			A[6] = 0; A[7] = 0; A[8] = 1;
			b[0] = 0; b[1] = 0; b[2] = 0;
			gyro_callib_500_dps[0] = 0; gyro_callib_500_dps[1] = 0; gyro_callib_500_dps[2] = 0;
			gyro_scaling_500_dps[0] = 1; gyro_scaling_500_dps[1] = 1; gyro_scaling_500_dps[2] = 1;
			status_reg &= ~(1<<0);
			status_reg &= ~(1<<1);
		}

		if(CF_update_run){
			CF_update_run = 0;

			HMC5883L_Read_Mag(mag_data);

			MPU6050_Read_Accel(accel_data);
			MPU6050_Read_Gyro(gyro_data);

			//MPU6050_Read_All(accel_data, gyro_data);

			mag_data_callibrated[0] = (mag_data[0]-b[0])*A[0] + (mag_data[1]-b[1])*A[1] + (mag_data[2]-b[2])*A[2];
			mag_data_callibrated[1] = (mag_data[0]-b[0])*A[3] + (mag_data[1]-b[1])*A[4] + (mag_data[2]-b[2])*A[5];
			mag_data_callibrated[2] = (mag_data[0]-b[0])*A[6] + (mag_data[1]-b[1])*A[7] + (mag_data[2]-b[2])*A[8];

			mag_data_rearranged[0] = -mag_data_callibrated[1];
			mag_data_rearranged[1] = -mag_data_callibrated[0];
			mag_data_rearranged[2] = -mag_data_callibrated[2];

			float mag_length = sqrt(pow(mag_data_rearranged[0],2)+pow(mag_data_rearranged[1],2)+pow(mag_data_rearranged[2],2));

			for(uint8_t index = 0; index<3; index++){
				gyro_data[index] = (gyro_data[index] - gyro_callib_500_dps[index])*gyro_scaling_500_dps[index];
			}
			/* FILTERING */
			for(uint8_t index = 0; index<3; index++){
				accel_data_filt[index] = accel_alpha * accel_data_filt[index] + (1-accel_alpha) * accel_data[index];
				gyro_data_filt[index] = gyro_alpha * gyro_data_filt[index] + (1-gyro_alpha) * gyro_data[index];
			}

			/*ANGLES EST*/
			phi_acc_est_old = phi_acc_est;
			theta_acc_est_old = theta_acc_est;

			phi_acc_est = atan2(accel_data_filt[1],accel_data_filt[2]);
			theta_acc_est = atan2(-accel_data_filt[0],sqrt(pow(accel_data_filt[1], 2)+pow(accel_data_filt[2], 2)));

			rotmat[1] = sin(phi_acc_est)*tan(theta_acc_est); rotmat[2] = cos(phi_acc_est)*tan(theta_acc_est);
			rotmat[4] = cos(phi_acc_est); 					rotmat[5] = -sin(phi_acc_est);
			rotmat[7] = sin(phi_acc_est)*cos(theta_acc_est); rotmat[8] = cos(phi_acc_est)*cos(theta_acc_est);

			phi_dot_est = 	gyro_data_filt[0] + rotmat[1]*gyro_data_filt[1] + rotmat[2]*gyro_data_filt[2];
			theta_dot_est = rotmat[4]*gyro_data_filt[1] + rotmat[5]*gyro_data_filt[2];
			psi_dot_est = 	rotmat[7]*gyro_data_filt[1] + rotmat[8]*gyro_data_filt[2];

			if(first_run){
				phi_est = phi_acc_est;
				theta_est = theta_acc_est;
			}

			/*COMPLEMENTARY FILTER*/
			phi_est = phi_acc_est*cf_alpha + (1-cf_alpha)*(phi_est + T*phi_dot_est);
			theta_est = theta_acc_est*cf_alpha + (1-cf_alpha)*(theta_est+T*theta_dot_est);


			//Mag
			for(uint8_t iter = 0; iter<3; iter++){
				mag_unit[iter] = mag_data_rearranged[iter]/mag_length;
			}

			//rotmat_x[4] = cos(phi_est); 	rotmat_x[5] = -sin(phi_est);
			//rotmat_x[7] = sin(phi_est); 	rotmat_x[8] = cos(phi_est);

			rotmat_y[0] = cos(theta_est); rotmat_y[2] = sin(theta_est);
			rotmat_y[6] = -sin(theta_est);rotmat_y[8] = cos(theta_est);


			rotmat_x_reversed[4] = cos(phi_est); 	rotmat_x_reversed[5] = sin(phi_est);
			rotmat_x_reversed[7] = -sin(phi_est); 	rotmat_x_reversed[8] = cos(phi_est);

			// mag_stabilised = rotmat_y*rotmat_x_reversed*mag_unit'
			//rot_x_r_mag_unit = rotmat_x_reversed*mag_unit'

			rot_x_r_mag_unit[0] = rotmat_x_reversed[0]*mag_unit[0];
			rot_x_r_mag_unit[1] = rotmat_x_reversed[4]*mag_unit[1]+ rotmat_x_reversed[5]*mag_unit[2];
			rot_x_r_mag_unit[2] = rotmat_x_reversed[7]*mag_unit[1]+ rotmat_x_reversed[8]*mag_unit[2];

			mag_unit_stab[0] = rotmat_y[0]*rot_x_r_mag_unit[0]+ rotmat_y[2]*rot_x_r_mag_unit[2];
			mag_unit_stab[1] = rot_x_r_mag_unit[1];
			mag_unit_stab[2] = rotmat_y[6]*rot_x_r_mag_unit[0]+ rotmat_y[8]*rot_x_r_mag_unit[2];

			//Cross product: east_horizon = cross([0 0 -1], mag_unit);
			/*	cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
				cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
				cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
			*/
			east_horizon[0] = mag_unit_stab[1];
			east_horizon[1] = -mag_unit_stab[0];

			//norm
			for(uint8_t iter = 0; iter<3; iter++){
				east_horizon_norm[iter] = east_horizon[iter]/(sqrt(pow(east_horizon[0],2)+pow(east_horizon[1],2)+pow(east_horizon[2],2)));
			}
			psi_mag_est_old = psi_mag_est;

			//heading angle calc
			//int8_t sign = (east_horizon_norm[0] > 0) - (east_horizon_norm[0] < 0);
			//psi_mag_est = -sign*acos(east_horizon_norm[1]);

			psi_mag_est = atan2(east_horizon_norm[1], east_horizon_norm[0]);

			if(first_run){
				psi_est = psi_mag_est;
			}

			// Complementary filter
			int8_t sign_psi_mag_est = (psi_mag_est > 0) - (psi_mag_est < 0);
			int8_t sign_psi_mag_est_old = (psi_mag_est_old > 0) - (psi_mag_est_old < 0);

			if((psi_mag_est_old > PI/2) && ((sign_psi_mag_est_old == 1) && (sign_psi_mag_est ==-1))){ // transition from high to low
				psi_est = psi_est-2*PI;
			}
			if((psi_mag_est_old < -PI/2) && ((sign_psi_mag_est_old == -1) && (sign_psi_mag_est == 1))){ // transition from low to high
				psi_est = psi_est+2*PI;
			}

			psi_est = psi_mag_est*cf_alpha + (1-cf_alpha)*(psi_est + T*psi_dot_est);
		}


		// Data transmit
		if(send_angles_req){
			uint8_t phi_bytes[4] = {0};
			uint8_t theta_bytes[4] = {0};
			uint8_t psi_bytes[4] = {0};

			FloatToIntBytes(phi_est,phi_bytes);
			FloatToIntBytes(theta_est,theta_bytes);
			FloatToIntBytes(psi_est,psi_bytes);

			USART1_SendChar(0xA8);
			USART1_SendChar(0x01);
			USART1_SendFloat(phi_est);
			USART1_SendFloat(theta_est);
			USART1_SendFloat(psi_est);
			send_angles_req = 0;
		}


		run_complete = 1;
		first_run = 0;
		//GPIOA->ODR &= ~(1<<5);

		if(send_raw_data){
			send_raw_data = 0;


			//USART1_SendChar(0);


			USART1_SendChar(0xA8);
			USART1_SendChar(0x01);
			for(uint8_t i = 0; i<3;i++)
				USART1_SendFloat(accel_data[i]);
			for(uint8_t i = 0; i<3;i++)
				USART1_SendFloat(gyro_data[i]);
			for(uint8_t i = 0; i<3;i++)
				USART1_SendFloat(mag_data_callibrated[i]);

			run_complete = 1;
		}

	}
}

void SendFloat(float data_in){

}

float IntToFloat(uint8_t *bit_array){
	union { uint8_t b[4]; float f_out; } conv;
	conv.b[0] = bit_array[0];
	conv.b[1] = bit_array[1];
	conv.b[2] = bit_array[2];
	conv.b[3] = bit_array[3];
	return conv.f_out;

}

static inline void FloatToIntBytes(float f_in, uint8_t *bit_array) {
    union {
        float    f;
        uint8_t  b[4];
    } conv;

    conv.f = f_in;
    bit_array[0] = conv.b[0];
    bit_array[1] = conv.b[1];
    bit_array[2] = conv.b[2];
    bit_array[3] = conv.b[3];
}

void GPIOConfig(void){
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<10);
	GPIOA->OTYPER &= (0<<5);
	GPIOA->OSPEEDR |= (1<<11);
	GPIOA->PUPDR &= ~((1<<10) | (1<<11));

}


void parse_gyro_calib_data(float *gyro_callib_500_dps, int16_t *gyro_calib_mantisa, int8_t *gyro_calib_exponent){
	for(uint8_t idx = 0; idx<3; idx++){
		gyro_callib_500_dps[idx] = (float)gyro_calib_mantisa[idx]* (float)(pow(10,(float)gyro_calib_exponent[idx]));
	}
}

void parse_mag_calib_data(float *A, float *b, int16_t *mag_calib_mantisa, int8_t *mag_calib_exponent){
	for(uint8_t idx = 0; idx<9; idx++){
		A[idx] = (float)mag_calib_mantisa[idx] * (float)(pow(10,(float)mag_calib_exponent[idx]));
	}
	for(uint8_t idx = 0; idx<3; idx++){
		b[idx] = (float)mag_calib_mantisa[idx+9] * (float)(pow(10,(float)mag_calib_exponent[idx+9]));
	}
}

int sgn(float num){
	if (num > 0) return 1;
	if (num < 0) return -1;
	return 0;
}

