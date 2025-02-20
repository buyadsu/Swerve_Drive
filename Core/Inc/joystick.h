/*
 * joystick.h
 *
 *  Created on: Feb 3, 2025
 *      Author: buya_-
 */

#ifndef INC_JOYSTICK_H_
#define INC_JOYSTICK_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>

typedef struct {
    int32_t axisX;
    int32_t axisY;
    int32_t axisRX;
    uint16_t buttons;
    bool new_data;
} JoystickData;

//#define KEY_GRAB   1
//#define KEY_THROW  2

// Initialization functions
void JOYSTICK_Init(UART_HandleTypeDef *huart);
void JOYSTICK_SetTimeout(uint32_t timeout);

// Data processing functions
void JOYSTICK_Process(void);
bool JOYSTICK_NewDataAvailable(void);
JoystickData JOYSTICK_GetData(void);

#endif /* INC_JOYSTICK_H_ */
