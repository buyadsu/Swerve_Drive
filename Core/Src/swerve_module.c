/*
 * swerve_module.c
 *
 *  Created on: Feb 3, 2025
 *      Author: root
 */


#include "swerve_module.h"
#include "odrive_uart.h"
#include <string.h>
#include <stdio.h>

// Debug print
//#define DEBUG_PRINT

// Private variables
static float prev_angles[4] = {0};  // RF, LF, RB, LB

// Private helper functions
static float normalize_angle_error(float error);
static float steering_pid_update(SteeringMotor* motor, float target, float current);
static int16_t clamp_pid_output(float pid_output, int16_t min_pwm, int16_t max_pwm);
static void set_steering_pwm(SteeringMotor* motor, int16_t pwm);
static void constrain_pulse_width(uint16_t* pulse, uint16_t min, uint16_t max);

// Private function prototypes
static void optimize_angle(float* angle, float* speed, int module_idx);
static void update_steering_pid(SwerveModule* module, float target_angle);
static float get_current_angle(SwerveModule* module);

void SM_Init(SwerveModule* module) {
    // Initialize steering motor PWM
    HAL_TIM_PWM_Start(module->steering.pwm_tim, module->steering.pwm_channel);

    // Initialize encoder timer
    HAL_TIM_Encoder_Start(module->steering.encoder_tim, TIM_CHANNEL_ALL);

    // Initialize ODrive
    ODrive_ClearErrors(module->driving.uart);
    ODrive_SetState(module->driving.uart, ODRIVE_STATE_IDLE);

    // Reset PID parameters
    module->steering.prev_error = 0.0f;
    module->steering.integral = 0.0f;
}

static int16_t clamp_pid_output(float pid_output, int16_t min_pwm, int16_t max_pwm) {
    if (pid_output < min_pwm) return min_pwm;
    if (pid_output > max_pwm) return max_pwm;
    return (int16_t)pid_output;
}

void SM_SetSteeringAngle(SwerveModule* module, float angle)
{
    optimize_angle(&angle, NULL, module->driving.axis);
    update_steering_pid(module, angle);
}

void SM_SetDrivingSpeed(SwerveModule* module, float speed)
{
    // Convert speed to velocity (turns/s)
    float velocity = speed * module->driving.max_velocity;
    
    // Send velocity command to ODrive
    ODrive_SendVelocity(module->driving.uart, velocity);
}

void SM_CalibrateESC(SwerveModule* module)
{
    // Clear any errors
    ODrive_ClearErrors(module->driving.uart);
    
    // Set to calibration state
    ODrive_SetState(module->driving.uart, ODRIVE_STATE_CALIBRATION);
    
    // Wait for calibration to complete (you might want to add proper error checking here)
    HAL_Delay(1000);
    
    // Set to closed loop control
    ODrive_SetState(module->driving.uart, ODRIVE_STATE_CLOSED_LOOP_CONTROL);
}

void SM_Update(SwerveModule* module)
{
    // Update steering PID
    float current_angle = get_current_angle(module);
    update_steering_pid(module, current_angle);
}

static void optimize_angle(float* angle, float* speed, int module_idx)
{
    const float original_angle = *angle;
    if (speed) *speed = fabsf(*speed);  // Maintain positive speed

    // Normalize to 0-360 first
    *angle = fmodf(*angle + 360.0f, 360.0f);

    // Calculate shortest path considering rotation requirements
    float angle_diff = *angle - prev_angles[module_idx];
    if (angle_diff > 180.0f) {
        *angle -= 360.0f;
    } else if (angle_diff < -180.0f) {
        *angle += 360.0f;
    }

    // Update previous angle
    prev_angles[module_idx] = *angle;

    // Speed deadzone handling
    if (speed && *speed < 0.1f) {
        *angle = 0.0f;
        *speed = 0.0f;
        prev_angles[module_idx] = 0.0f;
    }
}

static void update_steering_pid(SwerveModule* module, float target_angle)
{
    float current_angle = get_current_angle(module);
    float error = target_angle - current_angle;
    
    // Calculate PID terms
    float p_term = module->steering.Kp * error;
    float i_term = module->steering.Ki * error * module->steering.dt;
    float d_term = module->steering.Kd * error / module->steering.dt;
    
    // Combine terms and limit output
    float output = p_term + i_term + d_term;
    output = fmaxf(fminf(output, module->steering.max_pwm), -module->steering.max_pwm);
    
    // Set direction and PWM
    HAL_GPIO_WritePin(module->steering.dir_gpio_port, module->steering.dir_gpio_pin, 
                      output >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    
    __HAL_TIM_SET_COMPARE(module->steering.pwm_tim, module->steering.pwm_channel, 
                         (uint32_t)fabsf(output));
}

static float get_current_angle(SwerveModule* module)
{
    int32_t counts = (int32_t)__HAL_TIM_GET_COUNTER(module->steering.encoder_tim);
    return counts * module->counts_per_degree;
}

float SM_GetCurrentAngle(SwerveModule* module) {
    int32_t counts = (int16_t)(module->steering.encoder_tim->Instance->CNT);

	#ifdef DEBUG_PRINT
		printf("Encoder Counts: %ld\n", counts);
	#endif

    // Convert the encoder counts to an angle.
    // (Assumes module->counts_per_degree is set appropriately.)
    return counts * module->counts_per_degree * 360.0f;
}

bool SM_SteeringAtTarget(SwerveModule* module, float target_angle, float tolerance) {
    float current = SM_GetCurrentAngle(module);

	#ifdef DEBUG_PRINT    // For debugging, you might print the pulse:
		printf("Current angle: %f Target angle: %f\n", current, target_angle);
	#endif

    return fabsf(current - target_angle) <= tolerance;
}

static float normalize_angle_error(float error) {
    // Wrap error to [-180, 180)
    error = fmodf(error + 180.0f, 360.0f);
    if (error < 0)
        error += 360.0f;
    return error - 180.0f;
}

// Private function implementations
static float steering_pid_update(SteeringMotor* motor, float target, float current) {
    float error = normalize_angle_error(target - current);

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
    pwm = (int16_t)fminf(fabsf(pwm), __HAL_TIM_GET_AUTORELOAD(motor->pwm_tim));

	#ifdef DEBUG_PRINT
		printf("Steering PSC: %lu, ARR: %lu\n", motor->pwm_tim->Instance->PSC, motor->pwm_tim->Instance->ARR);
		printf("Direction: %d, pwm: %d\n", direction, pwm);
	#endif

    HAL_GPIO_WritePin(motor->dir_gpio_port, motor->dir_gpio_pin, direction);
    __HAL_TIM_SET_COMPARE(motor->pwm_tim, motor->pwm_channel, (uint16_t)pwm);
}

static void constrain_pulse_width(uint16_t* pulse, uint16_t min, uint16_t max) {
    if(*pulse < min) *pulse = min;
    if(*pulse > max) *pulse = max;
}
