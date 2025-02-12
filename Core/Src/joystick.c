/*
 * joystick.c
 *
 *  Created on: Feb 3, 2025
 *      Author: buya_-
 */


#include "joystick.h"

// Private variables
static UART_HandleTypeDef *_huart;
static uint32_t _timeout = 100;  // Default 100ms timeout
static char _data_buffer[64];
static uint16_t _buf_index = 0;
static JoystickData _current_data = {0};
static bool _new_data_available = false;

void JOYSTICK_Init(UART_HandleTypeDef *huart) {
    _huart = huart;
    memset(_data_buffer, 0, sizeof(_data_buffer));
    _buf_index = 0;
}

void JOYSTICK_SetTimeout(uint32_t timeout) {
    _timeout = timeout;
}

void JOYSTICK_Process(void) {
    char c;

    // Read until buffer full or newline received
    while (_buf_index < sizeof(_data_buffer) - 1) {
        if (HAL_UART_Receive(_huart, (uint8_t *)&c, 1, _timeout) == HAL_OK) {
            if (c == '\n') {
                // Process complete message
                _data_buffer[_buf_index] = '\0';

                // Temporary variables for parsing
//                uint16_t dpad, buttons, misc;
//                int32_t ax, ay, rax, ray, brake, throttle;

                  int32_t ax, ay, rax;

//                sscanf(_data_buffer,
//                       "dpad: 0x%04hx, buttons: 0x%04hx, axis L: %ld, %ld, axis R: %ld, %ld, brake: %ld, throttle: %ld, misc: 0x%04hx",
//                       &dpad, &buttons, &ax, &ay, &rax, &ray, &brake, &throttle, &misc);

                sscanf(_data_buffer,
                       "axis L: %ld, %ld, axis R: %ld", &ax, &ay, &rax);

                // Update data structure
                _current_data.axisX = ax;
                _current_data.axisY = ay;
                _current_data.axisRX = rax;
                _new_data_available = true;

                // Reset buffer
                memset(_data_buffer, 0, sizeof(_data_buffer));
                _buf_index = 0;
                break;
            }
            _data_buffer[_buf_index++] = c;
        } else {
            // Timeout occurred
            break;
        }
    }
}

bool JOYSTICK_NewDataAvailable(void) {
    return _new_data_available;
}

JoystickData JOYSTICK_GetData(void) {
    _new_data_available = false;
    return _current_data;
}
