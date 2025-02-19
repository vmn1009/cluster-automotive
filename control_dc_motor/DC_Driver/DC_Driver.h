/**
* @file: DC_Driver.h 
* @author: Minh Nhat
* @date: 30/01/2024
* @version: v1.0
* @description: Write a library to control DC Motor with PID algorithm
*/

#ifndef DC_DRIVER_H_
#define DC_DRIVER_H_

/* Include some necessary libraries */
#include "main.h"
#include "stdlib.h"
#include "stdint.h"
#include "string.h"
#include "stdbool.h"

/* Extern some timer, uart variables */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart6;



/* Define macros for easier access to PID struct members */
#define ek                          pid->e
#define ek_1                        pid->previous_e
#define _Kp                         pid->Kp
#define _Kd                         pid->Kd
#define _Ki                         pid->Ki
#define _Kb                         pid->Kb
#define _delta_t                    pid->delta_t
#define _Ki_1                       pid->Ki_previous
#define _alpha                      pid->alpha
#define _u_sat                      pid->u_sat
#define _u                          pid->u
#define _u_max                      pid->u_max
#define _u_min                      pid->u_min
#define kuP                         pid->p_term
#define kuI                         pid->i_term
#define kuD                         pid->d_term
#define fuD                         pid->d_term_filter
#define fuD_1												pid->d_term_filter_previpous
#define _e_reset                    pid->e_reset
#define _ek_windup                  pid->ek_windup
#define _desired_speed              pid->desired_speed
#define _desired_position           pid->desired_position 
#define _direction									pid->direction


/* Define macros for number */
									
#define ZERO_VALUE									0
#define ONE_VALUE										1
#define TWO_VALUE										2
#define THREE_VALUE									3
#define PI_VALUE										3.14
#define PULSE_PER_AROUND						2472
#define P2IR_VALUE									PI_VALUE / PULSE_PER_AROUND

#define NUMBER_OF_CHARACTERS				20
#define COUNTER_CLOCK_WISE					1
#define CLOCK_WISE									0


#define STATUS_RESET								0
#define STATUS_SET									1
#define SAMPLE_TIME									0.05
#define X4_ENCODER_MODE							4
#define ONE_MINUTE									60
#define PULSES_PER_REVOLUTION				616

#define FIRST_CASE									0
#define SECOND_CASE									1
#define THIRD_CASE									2
#define FOURTH_CASE									3


/* Create a struct to store specifications of PID to calculate speed from encoder signal */
typedef struct {
    float e;                 				// Current error
    float previous_e;        				// Previous error
						
    float Kp;                				// Proportional gain
    float Ki;                				// Integral gain
    float Ki_previous;       				// Previous integral gain
    float Kd;                				// Derivative gain
    float Kb;                				// Back calculation gain
    float delta_t;           				// Time step
    float alpha;             				// Filter coefficient
						
    float p_term;            				// Proportional term
    float i_term;            				// Integral term
    float d_term;            				// Derivative term
    float d_term_filter;     				// Filtered derivative term
    float d_term_filter_previpous;  // Previous filtered derivative term
    
    float u_sat;             				// Saturated control signal
    float u;                 				// Control signal
    float u_max;             				// Maximum control signal
    float u_min;             				// Minimum control signal
    float real_velocity;     				// Velocity value
    float real_position;
		float e_reset;           				// Error reset threshold
    float ek_windup;         				// Integral windup threshold
						
    float desired_speed;   				// Desired motor speed
    float desired_position;   		// Desired motor position
		bool	direction;								// Direction of rotation of the motor
} pid_specifications; 



/**
* @name: caculate_speed
* @param: *pid - PID specifications structure
* @description: Calculate the speed using the PID algorithm based on encoder signal
*/
void caculate_speed(pid_specifications *pid);

/**
* @name: caculate_pid_of_velocity
* @param: *pid - PID specifications structure
* @description: Calculate PID control for velocity
*/
int caculate_pid_of_velocity(pid_specifications *pid);

/**
* @name: caculate_pid_of_position
* @param: *pid - PID specifications structure
* @description: Calculate PID control for position
*/
void caculate_pid_of_position(pid_specifications *pid);

/**
* @name: control_motor_follow_pwm
* @param: pwm - Pulse Width Modulation value for motor control
* @description: Control the DC motor using the provided PWM value
*/
void control_motor_follow_pwm(pid_specifications *pid, int32_t i_pwm);

/**
* @name: pid_init
* @param: *pid, Kp, Ki, Kd, alpha, delta, delta_t, u_max, u_min, desired_speed
* @description: Initialize PID parameters in the PID specifications struct
*/
void pid_init(pid_specifications *pid, float i_Kp, float i_Ki, float i_Kd, float i_alpha, float i_delta_t, float i_u_max, float i_u_min, uint8_t i_desired_speed);

/**
* @name: control_motor_follow_vel_and_pos
* @param *pid - PID spefications structure
* @param velocity - input value follow rpm unit
* @param position - input value follow mm unit
* @description: Control the DC motor follow velocity and position input value
*/
void control_motor_follow_vel_and_pos(pid_specifications *pid, float velocity);

#endif
