/*
 * odrive_uart.h
 *
 *  Created on: Mar 27, 2025
 *      Author: root
 */

#ifndef INC_ODRIVE_UART_H_
#define INC_ODRIVE_UART_H_

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// ODrive ASCII Protocol Commands
#define ODRIVE_CMD_READ "r"
#define ODRIVE_CMD_WRITE "w"
#define ODRIVE_CMD_CALL "c"
#define ODRIVE_CMD_SAVE "s"

// ODrive Axis States
typedef enum {
    AXIS_STATE_IDLE = 1,
    AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,
    AXIS_STATE_MOTOR_CALIBRATION = 4,
    AXIS_STATE_SENSORLESS_CONTROL = 8,
    AXIS_STATE_CLOSED_LOOP_CONTROL = 8,
    AXIS_STATE_DISERROR = 9
} ODriveAxisState;

// ODrive Control Modes
typedef enum {
    CONTROL_MODE_VOLTAGE_CONTROL = 0,
    CONTROL_MODE_CURRENT_CONTROL = 1,
    CONTROL_MODE_VELOCITY_CONTROL = 2,
    CONTROL_MODE_POSITION_CONTROL = 3,
    CONTROL_MODE_IMPEDANCE_CONTROL = 4
} ODriveControlMode;

// Function declarations
void ODrive_UART_Init(void);
void ODrive_SendCommand(const char* cmd);
void ODrive_ReadResponse(char* buffer, uint32_t size);
bool ODrive_WaitForResponse(const char* expected, uint32_t timeout_ms);

// High-level control functions
bool ODrive_SetAxisState(uint8_t axis, ODriveAxisState state);
bool ODrive_SetControlMode(uint8_t axis, ODriveControlMode mode);
bool ODrive_SetVelocity(uint8_t axis, float velocity);
bool ODrive_SetPosition(uint8_t axis, float position);
bool ODrive_SetCurrent(uint8_t axis, float current);
bool ODrive_Calibrate(uint8_t axis);
bool ODrive_ClearErrors(uint8_t axis);

// Status functions
float ODrive_GetVelocity(uint8_t axis);
float ODrive_GetPosition(uint8_t axis);
float ODrive_GetCurrent(uint8_t axis);
uint32_t ODrive_GetError(uint8_t axis);
bool ODrive_IsReady(uint8_t axis);

#endif /* INC_ODRIVE_UART_H_ */
