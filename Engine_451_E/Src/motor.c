#include "motor.h"
#include "main.h"

#include <string.h>
#include <math.h>
#include <stdbool.h>

#define BETWEEN(x, var, result, old_var) (var >= x) ? result : old_var;
extern TIM_HandleTypeDef htim5; 
extern TIM_HandleTypeDef htim8; 
extern TIM_HandleTypeDef htim13; 
extern TIM_HandleTypeDef htim14;
extern uint8_t c; 
float deltaRPM = 0;
 
#define CCR_SIZE 35 
 
uint32_t ccr1[CCR_SIZE]; 
#define CCR_MAX (500000 / 7) 
uint32_t last_ccr1 = CCR_MAX; 
int last_ccr1_index = 0; 
 
#define DEFAULT_ARR 65000 
#define MIN_ARR 140

uint32_t newARR = DEFAULT_ARR; 

uint32_t openLoopStartTime = 0; 
 
#define OPEN_LOOP_TIME 1000
#define ARR_SIZE 64 
float arr_data[ARR_SIZE]; 
int last_arr_index = 0; 
 
 
uint8_t new_power = 30; 
uint8_t new_direction = 0; 

//For speed 
uint16_t speed_impulse = 0; 
float speed = 0.0f; 
//direction 
uint8_t direction = 1; 
uint8_t direction_user = 1;
uint8_t PWM_min = 0; 
uint8_t novel = 10; 

#define CCR_SIZE 35 
 
uint32_t ccr1[CCR_SIZE]; 
 
 

 
 
 
#define ARR_SIZE 64 
float arr_data[ARR_SIZE]; 
 
 
float targetRate = 200; 
 
//For speed 
 
//direction 
 
uint8_t i = 0, state = 2; 
#define PWM_SIZE 40 
uint8_t PWM_variable_10[PWM_SIZE]; 
uint8_t PWM_variable_20[PWM_SIZE]; 
uint8_t PWM_variable_30[PWM_SIZE]; 
uint8_t PWM_variable_40[PWM_SIZE]; 
uint8_t PWM_variable_50[PWM_SIZE]; 
uint8_t PWM_variable_60[PWM_SIZE]; 
uint8_t PWM_variable_70[PWM_SIZE]; 
uint8_t PWM_variable_80[PWM_SIZE]; 
uint8_t PWM_variable_90[PWM_SIZE]; 
uint8_t PWM_variable_100[PWM_SIZE];
/*
float PWM_SIN_Variable[PWM_SIZE] = 
{
			0, //0.006156, 
			0.024472, 
			0.054497, 
			0.095492, 
			0.146447, 
			0.206107, 
			0.273005, 
			0.345492, 
			0.421783, 
			0.500000, 
			0.578217, 
			0.654508, 
			0.726995, 
			0.793893, 
			0.853553, 
			0.904508, 
			0.945503, 
			0.975528, 
			0.993844, 
			1.000000
};
*/
float PWM_SIN_Variable[40] = 
{
	0.047581927,
	0.095056066,
	0.142314872,
	0.189251289,
	0.23575899,
	0.281732622,
	0.327068038,
	0.37166254,
	0.415415106,
	0.458226622,
	0.500000108,
	0.540640932,
	0.580057029,
	0.618159111,
	0.654860862,
	0.690079142,
	0.723734171,
	0.755749708,
	0.786053228,
	0.814576083,
	0.841253661,
	0.866025528,
	0.888835568,
	0.909632108,
	0.928368038,
	0.945000915,
	0.95949306,
	0.971811643,
	0.981928759,
	0.98982149,	
	0.995471956,
	0.998867356,
	1,
	0.998867356,
	0.995471956,
	0.98982149,
	0.981928759,
	0.971811643,
	0.95949306,
	0.945000915
};
uint8_t* PWM_ARR = PWM_variable_20;
void InitPower(uint8_t* pwm_arr, uint8_t power)
{
	for (i = 0; i < PWM_SIZE; i++) 
	{
		pwm_arr[i] = PWM_SIN_Variable[i] * power;
			/*
		if (i <= 9)
			pwm_arr[i] = PWM_SIN_Variable[i * 2] * power + 1;
		else
			pwm_arr[i] = PWM_SIN_Variable[18 - (i - 10) * 2] * power + 1;
           */
	}
}
void motor_init()
{
	memset(ccr1, 0xff, sizeof(ccr1));
	
	for (int i = 0; i < ARR_SIZE; i++)
		arr_data[i] = DEFAULT_ARR;
	
	InitPower(PWM_variable_10, 10);
	InitPower(PWM_variable_20, 20);
	InitPower(PWM_variable_30, 30);
	InitPower(PWM_variable_40, 40);
	InitPower(PWM_variable_50, 50);
	InitPower(PWM_variable_60, 50);
	InitPower(PWM_variable_70, 60);
	InitPower(PWM_variable_80, 70);
	InitPower(PWM_variable_90, 70);
	InitPower(PWM_variable_100, 70);
}


void Enable(uint8_t new_state)
{
	if (new_state)
		HAL_TIM_Base_Start_IT(&htim14);
	else
		HAL_TIM_Base_Stop_IT(&htim14);
}

bool open_loop = false;

void StartOpenLoop()
{
	open_loop = true;
	openLoopStartTime = HAL_GetTick();
}

float targetARRf = 0;
float actualARRf = 0;
float step = 0;

uint16_t ARR = 700;
uint32_t H1t = 0;
uint32_t H2t = 0;
uint32_t H3t = 0;
bool checkBreak = true;
float rate = 0;
float mean_rate = 0;
float mean_arr = 0;

float ARR_GetCut(float new_arr) //??????????? ????????? ARR
{
	if (new_arr > DEFAULT_ARR)
		return DEFAULT_ARR;
	if (new_arr < MIN_ARR)
		return MIN_ARR;
	return new_arr;
}

float ARR_GetStep(float input_arr)
{
	float step = 0;
	
	if (input_arr < MIN_ARR)
		input_arr = MIN_ARR;
	
	step = pow((input_arr - MIN_ARR) / (7000.0f - MIN_ARR), 1.5) * 1300 + 8;

	return step;
}

void SetSpeed(uint16_t rpm) 
{ 

  
 uint32_t acc = 0; 
 __disable_irq(); 
 for (int i = 0; i < CCR_SIZE; i++) 
  acc += ccr1[i]; 
 __enable_irq(); 
 rate = 60.0f / ((float)acc * 1e-7 / CCR_SIZE) / 7; 
  
 float k = 1e7 * 60 / (6 * 20 * 7); 
 targetARRf = ARR_GetCut(k / rpm); 
 actualARRf = ARR_GetCut(k / rate); 
 
 arr_data[last_arr_index] = actualARRf; 
 last_arr_index = (last_arr_index + 1) % ARR_SIZE; 
 float facc = 0; 
 for (int i = 0; i < ARR_SIZE; i++) 
  facc += arr_data[i]; 
 mean_arr = facc / ARR_SIZE; 
 
 mean_rate = k / mean_arr; 
 
 step = ARR_GetStep(mean_arr); 
 deltaRPM = -step * rate * rate / k; 
 if(mean_arr > 7000){
		step *= 3; 
 }
// if (fabs(mean_arr - targetARRf) < 2 * step)  
// { 
//  __disable_irq(); 
//  newARR = (uint16_t)targetARRf; 
//  __enable_irq(); 
// } 
// else  
 { 
  __disable_irq(); 
  if (targetARRf < mean_arr) // ?????? 
   newARR = (uint16_t)ARR_GetCut(fmax(mean_arr - step*0.5, targetARRf)); 
  else      // ?????????? 
   newARR = (uint16_t)ARR_GetCut(fmin(mean_arr + step*0.5, targetARRf)); 
   
  if (newARR > 1300) 
   PWM_ARR = PWM_variable_10; 
  if (newARR > 570 && newARR <= 1300) 
   PWM_ARR = PWM_variable_20; 
  if (newARR > 350 && newARR <= 570) 
   PWM_ARR = PWM_variable_30; 
  if (newARR > 270 && newARR <= 350) 
   PWM_ARR = PWM_variable_40; 
  if (newARR > 210 && newARR <= 270) 
   PWM_ARR = PWM_variable_50; 
  if (newARR > 180 && newARR <= 210) 
   PWM_ARR = PWM_variable_60;   
  if (newARR > 160 && newARR <= 180) 
   PWM_ARR = PWM_variable_70; 
  if (newARR > 140 && newARR <= 160) 
   PWM_ARR = PWM_variable_80; 
  if (newARR > 120 && newARR <= 140) 
   PWM_ARR = PWM_variable_90; 
  if (newARR <= 120) 
   PWM_ARR = PWM_variable_100; 
   
  /* 
  if (novel == 10) 
   PWM_ARR = PWM_variable_10; 
  if (novel == 20) 
   PWM_ARR = PWM_variable_20; 
  else if (novel == 30) 
   PWM_ARR = PWM_variable_30; 
  else if (novel == 40) 
   PWM_ARR = PWM_variable_40; 
  else if (novel == 50) 
   PWM_ARR = PWM_variable_50; 
  else if (novel == 60) 
   PWM_ARR = PWM_variable_60; 
  else if (novel == 70) 
   PWM_ARR = PWM_variable_70; 
  else if (novel == 80) 
   PWM_ARR = PWM_variable_80; 
  else if (novel == 90) 
   PWM_ARR = PWM_variable_90;   
  else if(novel == 100) 
   PWM_ARR = PWM_variable_100; 
   */ 
  __enable_irq(); 
 } 
}
	
void SetDirection(uint8_t new_direction)
{
	direction = new_direction;
}

float GetSpeed(void)
{
	return mean_rate * direction;
}



uint8_t GetDirection(void)
{
	return direction;
}



void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	/* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	TIM14->ARR = newARR;
	if (i == 20)
	{
		i = 0;
		if (direction)
		{
			if (state == 5)
				state = 0;
			else
				state++;
		}
		else
		{
			if (state == 0)
				state = 5;
			else
				state--;
		}
	}
	if(direction == 0){
		if (!i)
			
			//TIM8->CCER &= ~0x555;  			//Disable all
		TIM8->CCER |= 0x555;     		
		switch (state)
		{
		case 0:
			//TIM8->CCR3 = PWM_ARR[i];
			TIM8->CCR3 = PWM_ARR[19 - i];
			TIM8->CCR2 = PWM_ARR[39 - i];
			TIM8->CCR1 = PWM_min;
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH2
		
			break;
			
		case 1:
			TIM8->CCR3 = PWM_ARR[i + 20];
			//TIM8->CCR2 = PWM_ARR[19-i];
			TIM8->CCR2 = PWM_ARR[i];
			TIM8->CCR1 = PWM_min;
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH1
			
			break;
			
		case 2:
			TIM8->CCR3 = PWM_ARR[39 - i];
			TIM8->CCR2 = PWM_min;
			//TIM8->CCR1 = PWM_ARR[i];
			TIM8->CCR1 = PWM_ARR[19 - i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH2 CH1
			
			break;
			
		case 3:
			//TIM8->CCR3 = PWM_ARR[19-i];
			TIM8->CCR3 = PWM_ARR[i];
			TIM8->CCR2 = PWM_min;
			TIM8->CCR1 = PWM_ARR[i + 20];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH2
			
			break;
			
		case 4:
			TIM8->CCR3 = PWM_min;
			//TIM8->CCR2 = PWM_ARR[i];
			TIM8->CCR2 = PWM_ARR[19 - i];
			TIM8->CCR1 = PWM_ARR[39 - i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH1
			
			break;
			
		case 5:
			TIM8->CCR3 = PWM_min;
			TIM8->CCR2 = PWM_ARR[i + 20];
			//TIM8->CCR1 = PWM_ARR[19-i];
			TIM8->CCR1 = PWM_ARR[i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH2 CH1
			
			break;
		}
		
		i++;
	}
	
	if(direction == 1){
		if (!i)
			
			//TIM8->CCER &= ~0x555;  			//Disable all
		TIM8->CCER |= 0x555;     		
		switch (state)
		{
		case 0:
			//TIM8->CCR3 = PWM_ARR[i];
			TIM8->CCR3 = PWM_ARR[i];
			TIM8->CCR2 = PWM_ARR[39 - i];
			TIM8->CCR1 = PWM_min;
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH2
		
			break;
			
		case 1:
			TIM8->CCR3 = PWM_ARR[i + 20];
			TIM8->CCR2 = PWM_ARR[19-i];
			//TIM8->CCR2 = PWM_ARR[i];
			TIM8->CCR1 = PWM_min;
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH1
			
			break;
			
		case 2:
			TIM8->CCR3 = PWM_ARR[39 - i];
			TIM8->CCR2 = PWM_min;
			TIM8->CCR1 = PWM_ARR[i];
			//TIM8->CCR1 = PWM_ARR[19 - i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH2 CH1
			
			break;
			
		case 3:
			TIM8->CCR3 = PWM_ARR[19-i];
			//TIM8->CCR3 = PWM_ARR[i];
			TIM8->CCR2 = PWM_min;
			TIM8->CCR1 = PWM_ARR[i + 20];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH2
			
			break;
			
		case 4:
			TIM8->CCR3 = PWM_min;
			TIM8->CCR2 = PWM_ARR[i];
			//TIM8->CCR2 = PWM_ARR[19 - i];
			TIM8->CCR1 = PWM_ARR[39 - i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH3 CH1
			
			break;
			
		case 5:
			TIM8->CCR3 = PWM_min;
			TIM8->CCR2 = PWM_ARR[i + 20];
			TIM8->CCR1 = PWM_ARR[19-i];
			//TIM8->CCR1 = PWM_ARR[i];
			
			if (!i)
				TIM8->CCER |= 0x555; //Enable CH2 CH1
			
			break;
		}
		
		i++;
	}
	HAL_TIM_IRQHandler(&htim8);
	HAL_TIM_IRQHandler(&htim14);
}


void TIM5_IRQHandler(void)
{
	checkBreak = true;
		
	H1t = TIM5->CCR1;
	H2t = TIM5->CCR2;
	H3t = TIM5->CCR3;
	
	uint32_t new_ccr1 = TIM5->CCR1;
		
	ccr1[last_ccr1_index] = new_ccr1;
	last_ccr1_index = (last_ccr1_index + 1) % CCR_SIZE;

	HAL_TIM_IRQHandler(&htim5);
}