/*
 * odrive_uart.c
 *
 *  Created on: Mar 27, 2025
 *      Author: root
 */


#include "odrive_uart.h"
#include <string.h>
#include <stdio.h>

// Buffer for UART communication
static char uart_rx_buffer[256];
static char uart_tx_buffer[256];

// UART handle (you'll need to set this in your STM32 initialization)
extern UART_HandleTypeDef huart1;  // Change this to match your UART handle

void ODrive_UART_Init(void)
{
    // Initialize UART with ODrive settings:
    // - 115200 baud rate
    // - 8 data bits
    // - 1 stop bit
    // - No parity
    // - No flow control
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    HAL_UART_Init(&huart1);
}

void ODrive_SendCommand(const char* cmd)
{
    // Add newline if not present
    if (cmd[strlen(cmd) - 1] != '\n') {
        sprintf(uart_tx_buffer, "%s\n", cmd);
    } else {
        strcpy(uart_tx_buffer, cmd);
    }

    // Send command
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_tx_buffer, strlen(uart_tx_buffer), 100);
}

void ODrive_ReadResponse(char* buffer, uint32_t size)
{
    uint32_t rx_size = 0;

    // Read until newline or buffer full
    while (rx_size < size - 1) {
        uint8_t rx_byte;
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 100) == HAL_OK) {
            if (rx_byte == '\n') {
                break;
            }
            buffer[rx_size++] = rx_byte;
        }
    }
    buffer[rx_size] = '\0';
}

bool ODrive_WaitForResponse(const char* expected, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    char response[256];

    while (HAL_GetTick() - start_time < timeout_ms) {
        ODrive_ReadResponse(response, sizeof(response));
        if (strstr(response, expected) != NULL) {
            return true;
        }
    }
    return false;
}

bool ODrive_SetAxisState(uint8_t axis, ODriveAxisState state)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.requested_state %d\n", axis, state);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

bool ODrive_SetControlMode(uint8_t axis, ODriveControlMode mode)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.controller.config.control_mode %d\n", axis, mode);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

bool ODrive_SetVelocity(uint8_t axis, float velocity)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.controller.input_vel %.3f\n", axis, velocity);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

bool ODrive_SetPosition(uint8_t axis, float position)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.controller.input_pos %.3f\n", axis, position);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

bool ODrive_SetCurrent(uint8_t axis, float current)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.controller.input_current %.3f\n", axis, current);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

bool ODrive_Calibrate(uint8_t axis)
{
    return ODrive_SetAxisState(axis, AXIS_STATE_FULL_CALIBRATION_SEQUENCE);
}

bool ODrive_ClearErrors(uint8_t axis)
{
    char cmd[50];
    sprintf(cmd, "w axis%d.error 0\n", axis);
    ODrive_SendCommand(cmd);
    return ODrive_WaitForResponse("ok", 1000);
}

float ODrive_GetVelocity(uint8_t axis)
{
    char cmd[50];
    char response[50];
    float velocity = 0.0f;

    sprintf(cmd, "r axis%d.encoder.vel_estimate\n", axis);
    ODrive_SendCommand(cmd);
    ODrive_ReadResponse(response, sizeof(response));
    sscanf(response, "%f", &velocity);
    return velocity;
}

float ODrive_GetPosition(uint8_t axis)
{
    char cmd[50];
    char response[50];
    float position = 0.0f;

    sprintf(cmd, "r axis%d.encoder.pos_estimate\n", axis);
    ODrive_SendCommand(cmd);
    ODrive_ReadResponse(response, sizeof(response));
    sscanf(response, "%f", &position);
    return position;
}

float ODrive_GetCurrent(uint8_t axis)
{
    char cmd[50];
    char response[50];
    float current = 0.0f;

    sprintf(cmd, "r axis%d.motor.current_control.Iq_measured\n", axis);
    ODrive_SendCommand(cmd);
    ODrive_ReadResponse(response, sizeof(response));
    sscanf(response, "%f", &current);
    return current;
}

uint32_t ODrive_GetError(uint8_t axis)
{
    char cmd[50];
    char response[50];
    uint32_t error = 0;

    sprintf(cmd, "r axis%d.error\n", axis);
    ODrive_SendCommand(cmd);
    ODrive_ReadResponse(response, sizeof(response));
    sscanf(response, "%lu", &error);
    return error;
}

bool ODrive_IsReady(uint8_t axis)
{
    return ODrive_GetError(axis) == 0;
}
