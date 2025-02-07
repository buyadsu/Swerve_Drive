/*
 * swerve_module.c
 *
 *  Created on: Feb 3, 2025
 *      Author: root
 */


#include "swerve_module.h"

// Private helper functions
static float steering_pid_update(SteeringMotor* motor, float target, float current);
static void set_steering_pwm(SteeringMotor* motor, int16_t pwm);
static void constrain_pulse_width(uint16_t* pulse, uint16_t min, uint16_t max);

void SM_Init(SwerveModule* module) {
    // Initialize steering motor PWM
    HAL_TIM_PWM_Start(module->steering.pwm_tim, module->steering.pwm_channel);

    // Initialize encoder timer
    HAL_TIM_Encoder_Start(module->steering.encoder_tim, TIM_CHANNEL_ALL);

    // Initialize driving motor PWM
    HAL_TIM_PWM_Start(module->driving.pwm_tim, module->driving.pwm_channel);
//    __HAL_TIM_SET_COMPARE(module->driving.pwm_tim, module->driving.pwm_channel, module->driving.arming_pulse);
    // Reset PID parameters
    module->steering.prev_error = 0.0f;
    module->steering.integral = 0.0f;
}

void SM_UpdateSteering(SwerveModule* module, float target_angle) {
    float current = SM_GetCurrentAngle(module);
    float pid_output = steering_pid_update(&module->steering, target_angle, current);
    set_steering_pwm(&module->steering, (int16_t)pid_output);
}

void SM_UpdateDriving(SwerveModule* module, float speed) {
	uint16_t target_speed = module->driving.min_pulse + (uint16_t)((module->driving.max_pulse - module->driving.min_pulse) * fabsf(speed));
    constrain_pulse_width(&target_speed, module->driving.min_pulse, module->driving.max_pulse);

    // For debugging, you might print the pulse:
     printf("Driving speed: %d\n", target_speed);

     printf("Driving ARR: %lu\n", __HAL_TIM_GET_AUTORELOAD(module->driving.pwm_tim));
     printf("Driving PSC: %lu, ARR: %lu\n", module->driving.pwm_tim->Instance->PSC, module->driving.pwm_tim->Instance->ARR);

    __HAL_TIM_SET_COMPARE(module->driving.pwm_tim, module->driving.pwm_channel, target_speed);
}

float SM_GetCurrentAngle(SwerveModule* module) {
    int32_t counts = module->steering.encoder_tim->Instance->CNT;

    printf("Encoder Counts: %ld\n", counts);

    return (counts * 360.0f) / module->counts_per_degree;
}

bool SM_SteeringAtTarget(SwerveModule* module, float target_angle, float tolerance) {
    float current = SM_GetCurrentAngle(module);
    return fabsf(current - target_angle) <= tolerance;
}

void SM_CalibrateESC(DrivingMotor* motor) {
	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, motor->max_pulse);
    printf("Driving Calibrate PSC: %lu, ARR: %lu\n", motor->pwm_tim->Instance->PSC, motor->pwm_tim->Instance->ARR);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR1);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR2);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR3);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR4);
	HAL_Delay(7000);
	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, motor->min_pulse);
    printf("Driving Calibrate PSC: %lu, ARR: %lu\n", motor->pwm_tim->Instance->PSC, motor->pwm_tim->Instance->ARR);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR1);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR2);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR3);
    printf("Driving Calibrate PCC: %lu\n", motor->pwm_tim->Instance->CCR4);
	HAL_Delay(8000);
	__HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, motor->arming_pulse);
}

// Private function implementations
static float steering_pid_update(SteeringMotor* motor, float target, float current) {
    float error = target - current;

    // Integral term with clamping
    motor->integral += error * motor->dt;
    motor->integral = fmaxf(fminf(motor->integral, motor->integral_limit), -motor->integral_limit);

    // Derivative term
    float derivative = (error - motor->prev_error) / motor->dt;
    motor->prev_error = error;

    return (motor->Kp * error) + (motor->Ki * motor->integral) + (motor->Kd * derivative);
}

static void set_steering_pwm(SteeringMotor* motor, int16_t pwm) {
    bool direction = (pwm <= 0);
    pwm = fminf(fabsf(pwm), __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim));

    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, direction);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, pwm);
//    printf("Steering ARR: %lu\n", __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim));
    printf("Steering PSC: %lu, ARR: %lu\n", motor->pwm_tim->Instance->PSC, motor->pwm_tim->Instance->ARR);
}

static void constrain_pulse_width(uint16_t* pulse, uint16_t min, uint16_t max) {
    if(*pulse < min) *pulse = min;
    if(*pulse > max) *pulse = max;
}
