/*
 * swerve_module.h
 *
 *  Created on: Feb 3, 2025
 *      Author: buya_-
 */

#ifndef INC_SWERVE_MODULE_H_
#define INC_SWERVE_MODULE_H_

#include "main.h"
#include <stdbool.h>
#include <math.h>

// Steering Motor Configuration Structure
typedef struct {
    // GPIO Configuration
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;

    // PWM Timer Configuration
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_channel;

    // Encoder Timer Configuration
    TIM_HandleTypeDef* encoder_tim;

    // PID Parameters
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;
    float dt;

    // Internal state
    float prev_error;
    float integral;

    // Steering Motor Parameters
    uint32_t max_pwm;

} SteeringMotor;

// Driving Motor Configuration Structure
typedef struct {
    // PWM Timer Configuration
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_channel;

    // ESC Pulse Parameters
    uint16_t min_pulse;
    uint16_t max_pulse;
    uint16_t arming_pulse;
} DrivingMotor;

// Module Configuration Structure
typedef struct {
    SteeringMotor steering;
    DrivingMotor driving;
    float counts_per_degree;
} SwerveModule;

// Public functions
void SM_Init(SwerveModule* module);
void SM_UpdateSteering(SwerveModule* module, float target_angle);
void SM_UpdateDriving(SwerveModule* module, float speed);
float SM_GetCurrentAngle(SwerveModule* module);
bool SM_SteeringAtTarget(SwerveModule* module, float target_angle, float tolerance);
void SM_CalibrateESC(DrivingMotor* motor);

#endif /* INC_SWERVE_MODULE_H_ */
