#include <stdint.h>

extern uint8_t direction; 
extern uint8_t direction_user;
extern float mean_rate;
extern float mean_arr;

void motor_init(void);
void Enable(uint8_t new_state);
void StartOpenLoop(void);
void SetSpeed(uint16_t rpm);
void SetDirection(uint8_t new_direction);
float GetSpeed(void);
void SetPower(uint8_t power);
uint8_t GetDirection(void);
float ARR_GetCut(float new_arr);
float ARR_GetStep(float input_arr);
float MiddleSpeedHall(float mid);
uint16_t median_filter(uint16_t var);
float HallFilter(uint8_t hal_num);
