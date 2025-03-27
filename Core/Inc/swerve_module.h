/*
 * swerve_module.h
 *
 *  Created on: Feb 3, 2025
 *      Author: buya_-
 */

#ifndef SWERVE_MODULE_H
#define SWERVE_MODULE_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_nucleo.h"
#include <stdio.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ODrive states
#define ODRIVE_STATE_IDLE 1
#define ODRIVE_STATE_CALIBRATION 3
#define ODRIVE_STATE_CLOSED_LOOP_CONTROL 8
#define ODRIVE_STATE_ERROR 0

// Steering Motor Configuration Structure
typedef struct {
    GPIO_TypeDef* dir_gpio_port;
    uint16_t dir_gpio_pin;
    TIM_HandleTypeDef* pwm_tim;
    uint32_t pwm_channel;
    TIM_HandleTypeDef* encoder_tim;
    float Kp;
    float Ki;
    float Kd;
    float integral_limit;
    float dt;
    uint32_t max_pwm;
    float prev_error;
    float integral;
} SteeringMotor;

// Driving Motor Configuration Structure
typedef struct {
    UART_HandleTypeDef* uart;  // UART handle for ODrive communication
    uint8_t axis;              // ODrive axis number (0 or 1)
    float max_velocity;        // Maximum velocity in turns/s
    float max_current;         // Maximum current in amps
} DrivingMotor;

// Module Configuration Structure
typedef struct {
    SteeringMotor steering;
    DrivingMotor driving;
    float counts_per_degree;
} SwerveModule;

// Function prototypes
void SM_Init(SwerveModule* module);
void SM_SetSteeringAngle(SwerveModule* module, float angle);
void SM_SetDrivingSpeed(SwerveModule* module, float speed);
void SM_CalibrateESC(SwerveModule* module);
void SM_Update(SwerveModule* module);

#ifdef __cplusplus
}
#endif

#endif /* SWERVE_MODULE_H */
