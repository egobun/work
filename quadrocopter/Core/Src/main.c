/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250_reg.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
DMA_HandleTypeDef hdma_uart7_rx;

/* USER CODE BEGIN PV */
//variables for initializing the sensor and recording its data
int16_t destination_a[3] ={0,};
int16_t destination_g[3] ={0,};
float destination[3] ={0,};
float destination2[3] = {0,};
int16_t destination_m[3] ={0,};

//array for sending data to Serial Plotter by UART
char buf[50];

uint8_t buff[6] = {0,};
uint8_t sum = 0;
float kp1 = 0.011;
float kd1 = 0.0085;
float kp2 = 0.01;
float kd2 = 0.0085;
float kp3 = 0.006;
float kd3 = 0.001;
uint16_t pwm1 = 1499; 
uint16_t pwm2 = 1499; 
uint16_t pwm3 = 1499; 
uint16_t pwm4 = 1499; 
uint16_t pwm_normal = 1499; 

//initial accelerometer readings
int16_t proximate_normal_a_1 = -3250;
int16_t proximate_normal_a_2 = -550;
//initial gyroscope readings
int16_t proximate_normal_g_1 = 170;
int16_t proximate_normal_g_2 = 540;
int16_t proximate_normal_g_3 = -250;


uint8_t mass[10];
int16_t result[5];
int16_t result_filter[4];

float myfloat = 0;
int flag = 0;
int counter = 0;

int time = 0;
int16_t dataz = 0;
int16_t yaw = 0;
uint16_t board_data[4] = {0,};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_UART7_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int max(int a, int b){
  return (a > b) ? a : b;
}
 
int min(int a, int b){
  return (a > b) ? b : a;
}

double Med(int16_t *Arr, int n){
    int i,j;
    double z;
    for (i=0; i<n-1;i++)
        for (j=i+1;j<n;j++)
            if (Arr[i] > Arr[j])
            {
                z=Arr[i];
                Arr[i]=Arr[j];
                Arr[j]=z;
            }
    if ((n % 2) == 0)
        return 0.5*(Arr[n/2]+Arr[n/2-1]);
    else
        return Arr[n/2];
}

void MPU_init (void) {
 
   uint8_t R;
   uint8_t c;
 
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, PWR_MGMT_1, 1, 0x00, 1, 100);   // Clear sleep mode bit (6), enable all sensors
   HAL_Delay(100);
   R = 0x01;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, PWR_MGMT_1, 1, &R, 1, 100);   // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
 
   // Configure Gyro and Accelerometer
   // Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
   // DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
   // Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
   R = 0x03;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, CONFIG, 1, &R, 1, 100);
 
   // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
   R = 0x04;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, SMPLRT_DIV, 1, &R, 1, 100);   // Use a 200 Hz rate; the same rate set in CONFIG above
 
   // Set gyroscope full scale range
   // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, GYRO_CONFIG, 1, &c, 6, 100);    // get current GYRO_CONFIG register value
   // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | 0 << 3; // Set full scale range for the gyro - 0=+250dps
   // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, GYRO_CONFIG, 1, &c, 1, 100);    // Write new GYRO_CONFIG value to register
 
  // Set accelerometer full-scale range configuration
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, ACCEL_CONFIG, 1, &c, 6, 100);   // get current ACCEL_CONFIG register value
   // c = c & ~0xE0; // Clear self-test bits [7:5]
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | 0 << 3; // Set full scale range for the accelerometer  - 0=2g
  HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, ACCEL_CONFIG, 1, &c, 1, 100);   // Write new ACCEL_CONFIG register value
 
   // Set accelerometer sample rate configuration
   // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
   // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_CONFIG_2, 1, &c, 6, 100);   // get current ACCEL_CONFIG2 register value
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0]) 
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_ACCEL_CONFIG_2, 1, &c, 1, 100);   // Write new ACCEL_CONFIG2 register value
 
   // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
   // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
 
   // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
   R=0x22;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_INT_PIN_CFG, 1, &R, 1, 100);
  R=0x01;
   HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS_W, MPU9250_INT_ENABLE, 1, &R, 1, 100);   // Enable data ready (bit 0) interrupt
 
 
}

void MAGN_init (float * destination) {
 
   uint8_t R;
   uint8_t rawData[3];  // x/y/z gyro calibration data stored here
 
   // First extract the factory calibration for each magnetometer axis
  R=0x00;
   HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Power down magnetometer 
  HAL_Delay(10);
   R=0x0F;
   HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Enter Fuse ROM access mode
  HAL_Delay(10);
   HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_W, AK8963_ASAX, 1, rawData, 3, 100);  // Read the x-, y-, and z-axis calibration values
  destination[0] =  (float)(rawData[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128)/256.0f + 1.0f; 
  destination[2] =  (float)(rawData[2] - 128)/256.0f + 1.0f;
   R=0x00;
   HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Power down magnetometer 
  HAL_Delay(10);
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
   R = 0 << 4 | 0x06;
   HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_CNTL, 1, &R, 1, 100);    // Set magnetometer data resolution and sample ODR
  HAL_Delay(10);
 
   R=0x40;
  HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS_W, AK8963_ASTC, 1, &R, 1, 100); // set self-test  
}

void MPU_get_accel (int16_t * destination) {
 
 
  uint8_t rawData[6];
  HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_ACCEL_XOUT_H, 1, rawData, 6, 100);
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}

 
void MPU_get_gyro (int16_t * destination) {
 
   uint8_t rawData[6];  // x/y/z gyro register data stored here
   HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS_R, MPU9250_GYRO_XOUT_H, 1, rawData, 6, 100);    // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ; 
  destination[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
}
 
 
void MPU_get_magn (int16_t * destination) {
   uint8_t rawData[7];  // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
   uint8_t c;
   HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_ST1, 1, &c, 1, 100);
  if(c >= 0x01) { // wait for magnetometer data ready bit to be set
      HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS_R, AK8963_XOUT_L, 1, rawData, 7, 100);  // Read the six raw data and ST2 registers sequentially into data array
      c = rawData[6]; // End data read by reading ST2 register
    if(!(c & 0x08)) { // Check if magnetic sensor overflow set, if not then report data
         destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a signed 16-bit value
         destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) ;  // Data stored as little Endian
         destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) ;
				 
   }
  }
 
}

//The first parameter of the function is an array (size=3) of values from the three axes of the gyroscope (accelerometer, magnetometer), the description of the remaining parameters is indicated in "USER CODE BEGIN 1"
void MPU_filter(int16_t* destination_ins , int16_t** arr, int16_t * destination_filter,int16_t * destination_filter2,int16_t **arr2, int* N,int* counter_N, int* counter_M){
	for(int x = 0;x < 3;x++){
		if(*counter_N < *N+1){
			arr[x][(*counter_N)-1] = destination_ins[x];
		}	
		if(*counter_N == *N+1){
			int sum = 0;
			for(int j = 0;j < *N;j++){
				sum += arr[x][j];
			}
			destination_filter[x] = sum / *N;
			arr2[x][*counter_M - (*N+1)] = destination_filter[x];
			if(x == 2){
				*counter_M = 0;
			}
			
		}
		if(*counter_N > *N+1){
			for(int j = 0;j < *N - 1;j++){
				arr[x][j] = arr[x][j+1];
			}
			arr[x][*N-1] = destination_ins[x];
			int sum = 0;
			for(int j = 0;j < *N;j++){
				sum += arr[x][j];
			}
			destination_filter[x] = sum / *N;
			arr2[x][*counter_M] = destination_filter[x];
			
			if(*counter_M == 2){
				destination_filter2[x] = Med(arr2[x],3);
			}
			if(*counter_M == 2 && x == 2){
				*counter_M = -1;
			}
		}
	}
	*counter_N += 1;
	*counter_M += 1;
	
}


float trace_float(uint8_t * array){
	union help{
		uint8_t mass[4];
		float pix;
	}p;
 
	for(int i = 0; i<4;i++){
		p.mass[i] = array[i];
	}
	return p.pix;
}
int16_t* trace_int16_t(uint8_t * array, int16_t* result){
	union help{
		uint8_t mass8_t[10];
		int16_t mass16_t[5];
	}p;
 
	for(int i = 0; i<10;i++){
		p.mass8_t[i] = array[i];
	}
	for(int i = 0; i < 5; i++){
		result[i] = p.mass16_t[i];
	}
	return result;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//if(buff[0] == 0x7E || buff[0] == 0x7F || buff[0] == 0x80 || buff[0] == 0x81 || buff[0] == 0x82 || buff[0] == 0x83 || buff[0] == 0x84 || buff[0] == 0x85 || buff[0] == 0x86|| buff[0] == 0x87|| buff[0] == 0x88 || buff[0] == 0x89)
		//	sum = buff[0];
	
	if(mass[7] == 0 && mass[8] == 0 && mass[9] == 0){
		flag = 1;
		//myfloat = trace_float(mass);
	}
	else{
		flag = 2;
		//trace_int16_t(mass,result);
	}
	//HAL_UART_Receive_IT(&huart7, (uint8_t*)buff, 6);
	HAL_UART_Receive_DMA(&huart7,(uint8_t*)mass,10);
	//HAL_UART_Receive_DMA(&huart7,(uint8_t*)mass,1);
	
 
}
void send_k_float(uint8_t* sum, uint8_t* buff, float* kp, float* kd){	
	if(buff[0] == 0x7E ){
		*kp = trace_float(&buff[1]);
	}
	if(buff[0] == 0x7F ){
		*kd = trace_float(&buff[1]);
	}
}

void emergency_shutdown(uint16_t* pwm1, uint16_t* pwm2, uint16_t* pwm3, uint16_t* pwm4){
	if(*pwm1 > 1600 *2){
			*pwm1 = 1600*2;
		}
		if(*pwm1 < 800*2){
			*pwm1 = 800*2;
		}
		if(*pwm2 > 1600 *2){
			*pwm2 = 1600*2;
		}
		if(*pwm2 < 800*2){
			*pwm2 = 800*2;
		}
		if(*pwm3 > 1600 *2){
			*pwm3 = 1600*2;
		}
		if(*pwm3 < 800*2){
			*pwm3 = 800*2;
		}
		if(*pwm4 > 1600 *2){
			*pwm4 = 1600*2;
		}
		if(*pwm4 < 800*2){
			*pwm4 = 800*2;
		}
}

void package_came (float* kp1, float* kd1, float* kp2, float* kd2, float* kp3, float* kd3, uint16_t* pwm1, uint16_t* pwm2, uint16_t* pwm3, uint16_t* pwm4, uint16_t* pwm_normal, uint8_t* buff){
		if(flag == 1){
			if(mass[0] == 0x7E){
				*kp1 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x7F){
				*kd1 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x80){
				*pwm_normal = trace_float(&buff[1]);
			}
			if(mass[0] == 0x81){
				*pwm1 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x82){
				*pwm2 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x83){
				*pwm3 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x84){
				*pwm4 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x85){
				*kp2 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x86){
				*kd2 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x87){
				*kp3 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x88){
				*kd3 = trace_float(&buff[1]);
			}
			if(mass[0] == 0x89){
				*pwm1 = trace_float(&buff[1]);
				*pwm2 = trace_float(&buff[1]);
				*pwm3 = trace_float(&buff[1]);
				*pwm4 = trace_float(&buff[1]);
			}
		}
}

void package_came_2(float* kp1, float* kd1, float* kp2, float* kd2, float* kp3, float* kd3, uint16_t* pwm1, uint16_t* pwm2, uint16_t* pwm3, uint16_t* pwm4, uint16_t* pwm_normal, uint8_t* buff){
		if(flag == 1){
			if(mass[0] == 0x7E){
				*kp1 = myfloat;
			}
			if(mass[0] == 0x7F){
				*kd1 = myfloat;
			}
			if(mass[0] == 0x80){
				*pwm_normal = myfloat;
			}
			if(mass[0] == 0x81){
				*pwm1 = myfloat;
			}
			if(mass[0] == 0x82){
				*pwm2 = myfloat;
			}
			if(mass[0] == 0x83){
				*pwm3 = myfloat;
			}
			if(mass[0] == 0x84){
				*pwm4 = myfloat;
			}
			if(mass[0] == 0x85){
				*kp2 = myfloat;
			}
			if(mass[0] == 0x86){
				*kd2 = myfloat;
			}
			if(mass[0] == 0x87){
				*kp3 = myfloat;
			}
			if(mass[0] == 0x88){
				*kd3 = myfloat;
			}
			if(mass[0] == 0x89){
				*pwm_normal = myfloat;
				/*
				*pwm1 = myfloat;
				*pwm2 = myfloat;
				*pwm3 = myfloat;
				*pwm4 = myfloat;
				*/
			}
		}
}
void package_came_board(int flag){
	if(flag == 1){
		counter++;
		myfloat = trace_float(mass+1);
		//HAL_Delay(100);
	}
	if(flag == 2){
		
		trace_int16_t(mass,result);
		//HAL_Delay(100);
	}
	//flag = 0;
}
void board(int16_t* result, int16_t* result_filter){
	result_filter[0] = (-result[4] + 2050) / 20;
	result_filter[1] = (-result[3] + 2050) / 20;
	result_filter[2] = (-result[1] + 2050) * 4;
	result_filter[3] = (-result[2] + 2050) * 4;
}
uint16_t linear_transformations_board(uint16_t pwm){
	uint16_t pwm_pro = 4089/956 * pwm - 3755879/956;  
	return pwm_pro;
}
void frsky_board(uint16_t* board_data, int16_t* result_filter){
		board_data[0] = linear_transformations_board(TIM8->CCR1);
		board_data[1] = linear_transformations_board(TIM3->CCR2);
		board_data[2] = linear_transformations_board(TIM3->CCR3 - TIM3->CCR2);
		board_data[3] = linear_transformations_board(TIM3->CCR4 - TIM3->CCR3);
		result_filter[0] = (-board_data[0] + 2050) / 20;
		result_filter[1] = (-board_data[1]  + 2050) / 20;
		result_filter[2] = (-board_data[2]  + 2050) * 4;
		result_filter[3] = (-board_data[3]  + 2050) * 4;
	
}
void yaw_control(int* time,int16_t* dataz,int16_t* filter_1_data_g){
(*dataz) +=  filter_1_data_g[2]/1000;
(*time)++;
if(*time == 5)
		
if (*time == 10)
	(*time) = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//variables for the function"MPU_filter for the accelerometer"
	int sample_size = 20;//N, the size of the sample for which the average value is taken
	int counter_N_filter_1_a = 1;//counter_N, counter for the first filter
	int counter_M_filter_2_a = 1;//counter_M, counter for the second filter
	int16_t filter_1_data_a[3] ={0,};//destination_filter_1, data filtered using an average value
	int16_t filter_2_data_a[3] ={0,};//destination_filter_2, data filtered using the mean and median filter
	int16_t filter_matrix_1_a[3][20] = {{0}}; //arr, a matrix storing a buffer of averaged values
	int16_t *startRows_1_a[3] = { filter_matrix_1_a[0], filter_matrix_1_a[1], filter_matrix_1_a[2] };//auxiliary array for passing "filter_matrix_1" to the function
	int16_t filter_matrix_2_a[3][3] = {{0}}; //arr2, a matrix storing a buffer of averaged values
	int16_t *startRows_2_a[3] = { filter_matrix_2_a[0], filter_matrix_2_a[1], filter_matrix_2_a[2] };//auxiliary array for passing "filter_matrix_2" to the function
	
	
	//variables for the function"MPU_filter for the gyroscope"
	int counter_N_filter_1_g = 1;//counter_N, counter for the first filter
	int counter_M_filter_2_g = 1;//counter_M, counter for the second filter
	int16_t filter_1_data_g[3] ={0,};//destination_filter_1, data filtered using an average value
	int16_t filter_2_data_g[3] ={0,};//destination_filter_2, data filtered using the mean and median filter
	int16_t filter_matrix_1_g[3][20] = {{0}}; //arr, a matrix storing a buffer of averaged values
	int16_t *startRows_1_g[3] = { filter_matrix_1_g[0], filter_matrix_1_g[1], filter_matrix_1_g[2] };//auxiliary array for passing "filter_matrix_1" to the function
	int16_t filter_matrix_2_g[3][3] = {{0}}; //arr2, a matrix storing a buffer of averaged values
	int16_t *startRows_2_g[3] = { filter_matrix_2_g[0], filter_matrix_2_g[1], filter_matrix_2_g[2] };//auxiliary array for passing "filter_matrix_2" to the function
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_UART7_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart7, (uint8_t*)buff, 6);//late while you get signal for 6 bite and write him in "textbuff"
  HAL_UART_Receive_DMA(&huart7, (uint8_t*)mass, 10);
	//HAL_UART_Receive_DMA(&huart7, (uint8_t*)mass2, 1);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
	//HAL_UART_Receive_IT(&huart4, (uint8_t*)buff, 6);//late while you get signal for 6 bite and write him in "textbuff"
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  while (1)
  {
		MPU_get_gyro(destination_g);
		MPU_get_accel(destination_a);
		MPU_get_magn(destination_m);
		MPU_filter((int16_t*)destination_a,startRows_1_a,(int16_t*)filter_1_data_a,(int16_t*)filter_2_data_a,startRows_2_a,&sample_size,&counter_N_filter_1_a,&counter_M_filter_2_a);
		MPU_filter((int16_t*)destination_g,startRows_1_g,(int16_t*)filter_1_data_g,(int16_t*)filter_2_data_g,startRows_2_g,&sample_size,&counter_N_filter_1_g,&counter_M_filter_2_g);
		//yaw_control(&time, &dataz, filter_1_data_g);
		dataz +=  filter_1_data_g[2]/1000;
		yaw = kp3 * dataz +  kd3 * (filter_1_data_g[2] - proximate_normal_g_3);
		
		
		send_k_float(&sum,buff,&kp1,&kd1);
		
		
		pwm1 = result_filter[0]-5+yaw -15- result_filter[1] + pwm_normal + kp1 * ((proximate_normal_a_1 - result_filter[2])-filter_1_data_a[0]) + kd1 * ((-filter_1_data_g[1] + 2 * (proximate_normal_g_1)) + (proximate_normal_g_1)) + kp2 * ((proximate_normal_a_2 + result_filter[3]) - (- filter_1_data_a[1] + 2 * (proximate_normal_a_2))) + kd2 * ((-filter_1_data_g[0] + 2 * (proximate_normal_g_2)) + (proximate_normal_g_2)) ;
		pwm2 = result_filter[0]-yaw /*+ 15*/ + result_filter[1] + pwm_normal + kp1 * ((proximate_normal_a_1 - result_filter[2])-filter_1_data_a[0]) + kd1 * ((-filter_1_data_g[1] + 2 * (proximate_normal_g_1)) + (proximate_normal_g_1)) + kp2 * ((proximate_normal_a_2 - result_filter[3])-filter_1_data_a[1]) + kd2 * (filter_1_data_g[0] - (proximate_normal_g_2));
		pwm3 = result_filter[0]-5+yaw +10- result_filter[1] + pwm_normal + kp1 * ((proximate_normal_a_1 + result_filter[2]) - (- filter_1_data_a[0] + 2 * proximate_normal_a_1)) + kd1 * (filter_1_data_g[1] - (proximate_normal_g_1)) + kp2 * ((proximate_normal_a_2 - result_filter[3])-filter_1_data_a[1]) + kd2 * (filter_1_data_g[0] - (proximate_normal_g_2));
		pwm4 = result_filter[0]-yaw -15+ result_filter[1] + pwm_normal + kp1 * ((proximate_normal_a_1 + result_filter[2]) - (- filter_1_data_a[0] + 2 * proximate_normal_a_1)) + kd1 * (filter_1_data_g[1] - (proximate_normal_g_1)) + kp2 * ((proximate_normal_a_2 + result_filter[3])- (- filter_1_data_a[1] + 2 * proximate_normal_a_2)) + kd2 * ((-filter_1_data_g[0] + 2 * (proximate_normal_g_2)) + (proximate_normal_g_2));
		
		
		//package_came_board
		package_came_board(flag);
		//package came
		package_came_2(&kp1, &kd1, &kp2, &kd2,&kp3, &kd3, &pwm1, &pwm2, &pwm3, &pwm4, &pwm_normal, buff);
		
		
		
		//frsky_board(board_data, result_filter);
		board(result,result_filter);
		/*
		
		if(flag ==-1){
			HAL_UART_Receive_DMA(&huart7,(uint8_t*)mass,10);
		}
		*/
		//emergency shutdown
		emergency_shutdown(&pwm1,&pwm2,&pwm3,&pwm4);
		
		TIM1->CCR1 = pwm1;
		TIM2->CCR1 = pwm2;
		TIM3->CCR1 = pwm3;
	  TIM4->CCR1 = pwm4;
		
		//sending data via UART for plotting in serial plotter
		sprintf(buf, "$%d %d %d %d;", board_data[0], board_data[1], board_data[2], board_data[3]);
		HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 39999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 39999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 39999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 39999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1499;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 0xFFFFFFFF;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 16;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 115200;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
