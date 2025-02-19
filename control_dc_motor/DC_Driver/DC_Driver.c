#include "DC_Driver.h"
#include "stdio.h"

uint8_t previous_state;
int count_value;
float velocity_from_encoder, velocity_filter;
float position_from_encoder;
int32_t count_position_value = 0;
int32_t count_veloctity_value = 0;
int32_t xung = 0;
void pid_init(pid_specifications *pid,float i_Kp,float i_Ki,float i_Kd,float i_alpha,float i_delta_t,float i_u_max,float i_u_min, uint8_t i_desired_speed) { 
    _Kp = i_Kp;
    _Ki = i_Ki;
    _Kd = i_Kd;
    _alpha = i_alpha;
    _delta_t = i_delta_t;
    _Kb = 1 / i_delta_t;
    _u_max = i_u_max;
    _u_min = i_u_min;
		_desired_speed = i_desired_speed;
}




void EXTI9_5_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
unsigned char State0;
	State0 = (State0<<1) | HAL_GPIO_ReadPin(CH_A_GPIO_Port, CH_A_Pin);
	State0 = (State0<<1) | HAL_GPIO_ReadPin(CH_B_GPIO_Port, CH_B_Pin);
	State0 = State0&0x03; 
	switch (State0) {
		case 0:
			if(previous_state==1){ count_value++;}
			else { count_value--;}
		break;
		case 1:
			if(previous_state==3){ count_value++;}
			else { count_value--;}
		break;
		case 2:
			if(previous_state==0){ count_value++;}
			else { count_value--;}
		break;
		case 3:
			if(previous_state==2){ count_value++;}
			else { count_value--;}
		break;
		}
	previous_state = State0;
	count_veloctity_value++;
	if (count_value>=2472) {
		count_value = 0;
		count_position_value++;
	}
	else if	(count_value<=-2472) {
		count_value = 0;
		count_position_value--;
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(CH_B_Pin);
  /* USER CODE BEGIN EXTI4_IRQn 1 */
}

/**
* @brief This function handles EXTI line3 interrupt.
*/
void EXTI4_IRQHandler(void)	// doc encoder	
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// CHANNEL A
unsigned char State1;
	State1 = (State1<<1) | HAL_GPIO_ReadPin(CH_A_GPIO_Port, CH_A_Pin);
	State1 = (State1<<1) | HAL_GPIO_ReadPin(CH_B_GPIO_Port, CH_B_Pin);
	State1 = State1&0x03;
	switch (State1) {
		case 0:
			if(previous_state==1){ count_value++;}
			else { count_value--;}
		break;
		case 1:
			if(previous_state==3){ count_value++;}
			else { count_value--;}
		break;
		case 2:
			if(previous_state==0){ count_value++;}
			else { count_value--;}
		break;
		case 3:
			if(previous_state==2){ count_value++;}
			else { count_value--;}
		break;
		}
	previous_state = State1;
	count_veloctity_value++;
	if (count_value>=2472) {
		count_value = 0;
		count_position_value++;
	}
	else if	(count_value<=-2472) {
		count_value = 0;
		count_position_value--;
	}
  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(CH_A_Pin);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}



void caculate_speed(pid_specifications *pid)
{

	int temp = count_veloctity_value;
	count_veloctity_value = 0;
	velocity_from_encoder = temp * 500 / 103;
	pid->real_velocity = velocity_from_encoder;
}

void caculate_position(pid_specifications *pid) {
	pid->real_position = count_position_value * 0.8;
}

int caculate_pid_of_velocity(pid_specifications *pid) {
		
		_e_reset = ZERO_VALUE;
		/* Sampling Error */
		ek = (abs(_desired_speed) - abs(velocity_from_encoder));
	
		/* Caculate P term */
		kuP = _Kp * ek;
	
		/* Anti-windup */
		_ek_windup = _Ki * ek + _Kb * _e_reset;
	
		/* Caculate I term */
		kuI = _Ki_1 + _ek_windup * _delta_t;
	
		/* Caculate D term*/
		kuD = _Kd * ((ek - ek_1) / _delta_t);
		
		/* Low pass filter */
		fuD = ((1 - _alpha) * fuD_1) + (_alpha * kuD);
	
		/* Get current value to sign previous value variable */
		_Ki_1 = kuI;
		
		/* Summing up the values of the I term and P term determines the sign for the variable 'u' */
		_u = kuP + kuI + fuD;
		
    if (_u > 100) {
        _u_sat = 100;
    } else if (_u < 0) {
        _u_sat = 0;
    } else {
        _u_sat = _u;
    }
		_e_reset = _u - _u_sat;
		
		return _u_sat;
}



void control_motor_follow_vel_and_pos(pid_specifications *pid, float velocity) {
		_desired_speed = velocity;
		caculate_speed(pid);
		int pwm =  caculate_pid_of_velocity(pid);
	
		if(_desired_speed > 0){
		HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(pwm));
		}
		else if (_desired_speed < 0){
			HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, abs(pwm));
		}
		else __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ZERO_VALUE);
		
}
