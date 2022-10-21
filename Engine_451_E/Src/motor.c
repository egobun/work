#include "motor.h"
#include "main.h"

#include <string.h>
#include <math.h>

#define BETWEEN(x, var, result, old_var) (var >= x) ? result : old_var;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern uint8_t c;


#define CCR_SIZE 35

uint32_t ccr1[CCR_SIZE];
#define CCR_MAX (500000 / 7)
uint32_t last_ccr1 = CCR_MAX;
int last_ccr1_index = 0;

#define DEFAULT_ARR 7000

uint32_t newARR = DEFAULT_ARR;
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
uint8_t PWM_min = 0;
uint8_t novel = 10;
#define PWM_SIZE 20
#define CCR_SIZE 35

uint32_t ccr1[CCR_SIZE];


#define DEFAULT_ARR 7000



#define ARR_SIZE 64
float arr_data[ARR_SIZE];


float targetRate = 200;

//For speed

//direction
float step_way[2] = {0,};
uint8_t counter_step = 0;
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
float PWM_SIN_Variable[20] = 
{
			0.006156, 
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
/*
float PWM_SIN_Variable[40] = 
	{
		0.039259808,
		0.078459079,
		0.117537373,
		0.156434433,
		0.195090282,
		0.233445316,
		0.271440395,
		0.309016932,
		0.346116988,
		0.382683357,
		0.418659656,
		0.453990412,
		0.488621149,
		0.522498467,
		0.555570131,
		0.587785147,
		0.61909384,
		0.649447937,
		0.6788006320,
		0.707106666,
		0.734322393,
		0.760405849,
		0.785316815,
		0.809016879,
		0.831469499,
		0.852640053,
		0.872495899,
		0.89100642,
		0.908143075,
		0.923879439,
		0.938191248,
		0.951056436,
		0.962455163,
		0.972369856,
		0.980785225,
		0.987688295,
		0.993068421,
		0.996917309,
		0.999229024,
		1

	};
	*/
	/*
	float PWM_SIN_Variable[40] = 
	{
		0.039259808,
		0.078459079,
		0.117537373,
		0.156434433,
		0.195090282,
		0.233445316,
		0.271440395,
		0.309016932,
		0.346116988,
		0.382683357,
		0.418659656,
		0.453990412,
		0.488621149,
		0.522498467,
		0.555570131,
		0.587785147,
		0.61909384,
		0.649447937,
		0.6788006320,
		0.707106666,
		0.734322393,
		0.760405849,
		0.785316815,
		0.809016879,
		0.831469499,
		0.852640053,
		0.872495899,
		0.89100642,
		0.908143075,
		0.923879439,
		0.938191248,
		0.951056436,
		1,
		0.999229024,
		0.996917309,
		0.993068421,
		0.987688295,
		0.980785225,
		0.972369856,
		0.962455163

	};
	*/
	/*
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
		0.995471956,
		0.981928759,
		0.95949306,
		0.928368038,
		0.888835568,
		0.841253661,
		0.786053228
	};
/*

float PWM_SIN_Variable[20] = 
{
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1,
			1
};
*/
uint8_t* PWM_ARR = PWM_variable_20;
void InitPower(uint8_t* pwm_arr, uint8_t power)
{
	for (i = 0; i < 40; i++) {
		pwm_arr[i] = PWM_SIN_Variable[i] * power;
		/*
		if (i <= 9)
			pwm_arr[i] = PWM_SIN_Variable[i * 2] * power + 1;
		else
			pwm_arr[i] = PWM_SIN_Variable[18 - (i - 10) * 2] * power + 1;
		*/
	}
	
//	new_power = power;
//	uint8_t i = 0;
//	for (; i < 20; i++)
//	{
//		PWM_variable[i] = (uint8_t)(PWM_SIN_Variable[i]*(float)power);
//	}
}
uint8_t InitPower2(uint8_t* pwm_arr, uint8_t power)
{
	for (i = 0; i < 40; i++) {
		pwm_arr[i] = PWM_SIN_Variable[i] * power;
		/*
		if (i <= 9)
			pwm_arr[i] = PWM_SIN_Variable[i * 2] * power + 1;
		else
			pwm_arr[i] = PWM_SIN_Variable[18 - (i - 10) * 2] * power + 1;
		*/
	}
	return *pwm_arr;
//	new_power = power;
//	uint8_t i = 0;
//	for (; i < 20; i++)
//	{
//		PWM_variable[i] = (uint8_t)(PWM_SIN_Variable[i]*(float)power);
//	}
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
	InitPower(PWM_variable_60, 60);
	InitPower(PWM_variable_70, 70);
	InitPower(PWM_variable_80, 80);
	InitPower(PWM_variable_90, 90);
	InitPower(PWM_variable_100, 100);
}

void Enable(uint8_t new_state)
{
	if (new_state)
		HAL_TIM_Base_Start_IT(&htim14);
	else
		HAL_TIM_Base_Stop_IT(&htim14);
}

void StartOpenLoop()
{
	TIM14->ARR = DEFAULT_ARR;

}

float targetARRf = 0;
float actualARRf = 0;
float step = 0;
float deltaRPM = 0;
float speed_hall_1 = 0, speed_hall_2 = 0, speed_hall_3 = 0;

float middle_speed = 0;
uint16_t ARR = 700;
uint32_t H1t = 0;
uint32_t H2t = 0;
uint32_t H3t = 0;
uint32_t H1t_prev = 0, H2t_prev = 0, H3t_prev = 0;
float rate = 0;
float mean_rate = 0;
float mean_arr = 0;

void SetSpeed(uint16_t rpm)
{
	__disable_irq();
	H1t_prev = H1t;
	H2t_prev = H2t;
	H3t_prev = H3t;
	__enable_irq();
	
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
	
//	if (fabs(mean_arr - targetARRf) < 2 * step) 
//	{
//		__disable_irq();
//		newARR = (uint16_t)targetARRf;
//		__enable_irq();
//	}
//	else 
	{
		__disable_irq();
		/*
		if(newARR < 1000 && fabs(step_way[1] - step_way[0]) > 3)
			step = step * 0.5;
		*/
		if(newARR < 160)
			step = step * 0.5;
		
		if (targetARRf < mean_arr) // ??????
			newARR = (uint16_t)ARR_GetCut(fmax(mean_arr - step, targetARRf));
		else						// ??????????
			newARR = (uint16_t)ARR_GetCut(fmin(mean_arr + step, targetARRf));
		
		if (newARR > 1300)
			PWM_ARR = PWM_variable_10;
		if (newARR > 570 && newARR <= 1300)
			PWM_ARR = PWM_variable_20;
		if (newARR > 350 && newARR <= 570)
			PWM_ARR = PWM_variable_30;
		if (newARR > 270 && newARR <= 350)
			PWM_ARR = PWM_variable_40;
		if (newARR > 210 && newARR <= 270)
			PWM_ARR = PWM_variable_60;
		if (newARR > 180 && newARR <= 210)
			PWM_ARR = PWM_variable_70;		
		if (newARR > 160 && newARR <= 180)
			PWM_ARR = PWM_variable_80;
		if (newARR > 140 && newARR <= 160)
			PWM_ARR = PWM_variable_90;
		if (newARR > 120 && newARR <= 140)
			PWM_ARR = PWM_variable_100;
		if (newARR <= 120)
			PWM_ARR = PWM_variable_100;
			
			/*
			novel = 70084/newARR;
			InitPower(PWM_variable_10,novel);
			PWM_ARR = PWM_variable_10;
		*/
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


float ARR_GetCut(float new_arr) //??????????? ????????? ARR
{
	if (new_arr > 7000)
		return 7000;
	if (new_arr < 50)
		return 50;
	return new_arr;
}

int16_t offset = 0;

float ARR_GetStep(float input_arr)
{
	float step = 0;
	
	if (input_arr < 140)
		input_arr = 140;
	
	step = pow((input_arr - 140.0f) / (7000.0f - 140.0f), 1.5) * 1300 + 8 + offset;
	
	step_way[counter_step] = step;
	counter_step++;
	if(counter_step == 2){
		counter_step = 0;
	}
	
//	
//	if (input_arr < 400)
//		step = 5;
//	
//	if (input_arr < 370)
//		step = 4;

	return step;
}
	
void SetDirection(uint8_t new_direction)
{
	direction = new_direction;
}

float GetSpeed(void)
{
	return mean_rate;
	//return speed_hall;
}
uint8_t middle_speed_index = 0;
#define MIDDLE_SPEED_LEVEL 35
float middle_speed_array[MIDDLE_SPEED_LEVEL] = { 0 };
float MiddleSpeedHall(float mid)
{
	float middle_sum = 0;
	middle_speed_array[middle_speed_index++] = mid;
	if (middle_speed_index == MIDDLE_SPEED_LEVEL)
		middle_speed_index = 0;
	for (uint8_t j = 0; j < MIDDLE_SPEED_LEVEL; j++)
		middle_sum += middle_speed_array[j];
	return middle_sum / MIDDLE_SPEED_LEVEL;
}

float HallFilter(uint8_t hal_num)
{
	float speed_hall = 0;
	uint16_t hall_value = 0;
	switch (hal_num)
	{
	case 1:
		hall_value = TIM5->CCR1;
		speed_hall = 35e5 / (float)hall_value - 2.0f;
		return speed_hall;
	case 2:
		speed_hall = 23e5 / (float)TIM5->CCR2 - 2.0f;
		return speed_hall;
	case 3:
		speed_hall = 12e5 / (float)TIM5->CCR3 - 2.0f;
		return speed_hall;
	default:
		
		break;
	}
	return speed_hall;
}
	
	
uint8_t median_filter_index = 0;
uint16_t median_filter_data[3] = { 0 };
uint16_t median_filter(uint16_t var)
{
	median_filter_data[median_filter_index++] = var;
	if (median_filter_index == 3)
		median_filter_index = 0;
	
	if ((median_filter_data[0] > median_filter_data[1]) && (median_filter_data[0] < median_filter_data[2]))
		return median_filter_data[0];
	if ((median_filter_data[0] > median_filter_data[2]) && (median_filter_data[0] < median_filter_data[1]))
		return median_filter_data[0];
	if ((median_filter_data[1] > median_filter_data[0]) && (median_filter_data[1] < median_filter_data[2]))
		return median_filter_data[1];
	if ((median_filter_data[1] > median_filter_data[2]) && (median_filter_data[1] < median_filter_data[0]))
		return median_filter_data[1];
	if ((median_filter_data[2] > median_filter_data[0]) && (median_filter_data[2] < median_filter_data[1]))
		return median_filter_data[2];
	if ((median_filter_data[2] > median_filter_data[1]) && (median_filter_data[2] < median_filter_data[0]))
		return median_filter_data[2];
}
	


/*
void SetPower(uint8_t power)
{
	new_power = power;
	uint8_t i = 0;
	for (; i < 20; i++)
	{
		PWM_variable[i] = (uint8_t)(PWM_SIN_Variable[i]*(float)power);
	}
}
*/
uint8_t GetDirection(void)
{
	return direction;
}

void TIM8_UP_TIM13_IRQHandler(void)
{
	speed = ((float)speed_impulse) * 60.0f / 7.0f * 2 / 3;
	speed_impulse = 0;
	
	HAL_TIM_IRQHandler(&htim8);
	HAL_TIM_IRQHandler(&htim13);
}

uint8_t NEW_STATE = 2;
uint8_t nst1 = 2, nst2 = 1, nst3 = 0, nst4 = 5, nst5 = 4, nst6 = 3;

uint32_t trig_h1		= 14166;
uint32_t trig_h2		= 1000;
uint8_t closed_loop = 0;
uint16_t tim5_cnt = 0;

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	/* USER CODE BEGIN TIM8_TRG_COM_TIM14_IRQn 0 */
	
	TIM14->ARR = newARR;
	/*
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
	
	if (!i)
		TIM8->CCER &= ~0x555;  			//Disable all
	
	switch(state)
	{
	case 0:
		TIM8->CCR3 = PWM_ARR[i];
		TIM8->CCR2 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x550;     	//Enable CH3 CH2
		break;
		
	case 1:
		TIM8->CCR3 = PWM_ARR[i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x505;      	//Enable CH3 CH1
		
		break;
		
	case 2:
		TIM8->CCR2 = PWM_ARR[i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x055;       	//Enable CH2 CH1
		
		break;
		
	case 3:
		TIM8->CCR2 = PWM_ARR[i];
		TIM8->CCR3 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x550;      	//Enable CH3 CH2
		
		break;
		
	case 4:
		TIM8->CCR1 = PWM_ARR[i];
		TIM8->CCR3 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x505;      	//Enable CH3 CH1
		
		break;
		
	case 5:
		TIM8->CCR1 = PWM_ARR[i];
		TIM8->CCR2 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x055;       	//Enable CH2 CH1
		
		break;
	}
	*/
	
	//two vesion
	/*
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
	
	if (!i)
		TIM8->CCER &= ~0x555;  			//Disable all
	
	switch(state)
	{
	case 0:
		TIM8->CCR3 = PWM_ARR[i];
		TIM8->CCR2 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x550;     	//Enable CH3 CH2
		break;
		
	case 1:
		TIM8->CCR3 = PWM_ARR[19-i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x505;      	//Enable CH3 CH1
		
		break;
		
	case 2:
		TIM8->CCR2 = PWM_ARR[i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x055;       	//Enable CH2 CH1
		
		break;
		
	case 3:
		TIM8->CCR2 = PWM_ARR[19-i];
		TIM8->CCR3 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x550;      	//Enable CH3 CH2
		
		break;
		
	case 4:
		TIM8->CCR1 = PWM_ARR[i];
		TIM8->CCR3 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x505;      	//Enable CH3 CH1
		
		break;
		
	case 5:
		TIM8->CCR1 = PWM_ARR[19-i];
		TIM8->CCR2 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x055;       	//Enable CH2 CH1
		
		break;
	}
	*/
	
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
	
	if (!i)
		
		//TIM8->CCER &= ~0x555;  			//Disable all
		TIM8->CCER |= 0x555;     		
	switch(state)
	{
	case 0:
		//TIM8->CCR3 = PWM_ARR[i];
		TIM8->CCR3 = PWM_ARR[19-i];
		TIM8->CCR2 = PWM_ARR[39-i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x555;     	//Enable CH3 CH2
	
		break;
		
	case 1:
		TIM8->CCR3 = PWM_ARR[i+20];
		//TIM8->CCR2 = PWM_ARR[19-i];
		TIM8->CCR2 = PWM_ARR[i];
		TIM8->CCR1 = PWM_min;
		
		if (!i)
			TIM8->CCER |= 0x555;      	//Enable CH3 CH1
		
		break;
		
	case 2:
		TIM8->CCR3 = PWM_ARR[39-i];
		TIM8->CCR2 = PWM_min;
		//TIM8->CCR1 = PWM_ARR[i];
		TIM8->CCR1 = PWM_ARR[19-i];
		
		if (!i)
			TIM8->CCER |= 0x555;       	//Enable CH2 CH1
		
		break;
		
	case 3:
		//TIM8->CCR3 = PWM_ARR[19-i];
		TIM8->CCR3 = PWM_ARR[i];
		TIM8->CCR2 = PWM_min;
		TIM8->CCR1 = PWM_ARR[i+20];
		
		if (!i)
			TIM8->CCER |= 0x555;      	//Enable CH3 CH2
		
		break;
		
	case 4:
		TIM8->CCR3 = PWM_min;
		//TIM8->CCR2 = PWM_ARR[i];
		TIM8->CCR2 = PWM_ARR[19-i];
		TIM8->CCR1 = PWM_ARR[39-i];
		
		if (!i)
			TIM8->CCER |= 0x555;      	//Enable CH3 CH1
		
		break;
		
	case 5:
		TIM8->CCR3 = PWM_min;
		TIM8->CCR2 = PWM_ARR[i+20];
		//TIM8->CCR1 = PWM_ARR[19-i];
		TIM8->CCR1 = PWM_ARR[i];
		
		if (!i)
			TIM8->CCER |= 0x555;       	//Enable CH2 CH1
		
		break;
	}
	
	i++;
	
	HAL_TIM_IRQHandler(&htim8);
	HAL_TIM_IRQHandler(&htim14);
}


void TIM5_IRQHandler(void)
{
	c++;
	if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC1)) {
		H1t = TIM5->CCR1;
		H2t = TIM5->CCR2;
		H3t = TIM5->CCR3;
	
		uint32_t new_ccr1 = TIM5->CCR1;
	//	if (new_ccr1 > last_ccr1 / 1.5)
		{
			ccr1[last_ccr1_index] = new_ccr1;
			last_ccr1_index = (last_ccr1_index + 1) % CCR_SIZE;
			last_ccr1 = new_ccr1;
		
	//		SetSpeed(1000);
		}
		if (last_ccr1 > CCR_MAX)
			last_ccr1 = CCR_MAX;
	}

	HAL_TIM_IRQHandler(&htim5);
}


/*
void motor_init()
{
	memset(ccr1, 0xff, sizeof(ccr1));
	for (int i = 0; i < ARR_SIZE; i++)
		arr_data[i] = DEFAULT_ARR;
}
*/