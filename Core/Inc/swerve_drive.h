/*
 * swerve_drive.h
 *
 *  Created on: Mar 27, 2025
 *      Author: root
 */

#ifndef INC_SWERVE_DRIVE_H_
#define INC_SWERVE_DRIVE_H_

#include "swerve_module.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Robot dimensions (adjust according to actual measurements)
#define ROBOT_LENGTH 0.5f // Distance from front to back wheels (meters)
#define ROBOT_WIDTH 0.5f  // Distance from left to right wheels (meters)
#define STEERING_DEADZONE 2.0f  // Degrees
#define SPEED_DEADZONE 0.05f

// Debug functionality
#ifdef DEBUG_PRINT
#define SWERVE_DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
#define SWERVE_DEBUG_PRINT(fmt, ...)
#endif

// Module indices for easy reference
typedef enum {
    MODULE_RF = 0,  // Right Front
    MODULE_LF = 1,  // Left Front
    MODULE_RB = 2,  // Right Back
    MODULE_LB = 3   // Left Back
} SwerveModuleIndex;

// Structure to hold wheel speeds and angles
typedef struct {
    float speed;
    float angle;
} WheelData;

// Structure to hold all wheel data
typedef struct {
    WheelData rf;  // Right Front
    WheelData lf;  // Left Front
    WheelData rb;  // Right Back
    WheelData lb;  // Left Back
} SwerveDriveData;

// External module declarations
extern SwerveModule moduleRF;
extern SwerveModule moduleLF;
extern SwerveModule moduleRB;
extern SwerveModule moduleLB;

// Function declarations
void SwerveDrive_Init(void);
void SwerveDrive_Update(float x_speed, float y_speed, float rotation);
void SwerveDrive_GetWheelData(SwerveDriveData* data);
void SwerveDrive_ApplyWheelData(SwerveDriveData* data);
void SwerveDrive_Stop(void);

// Debug functions
#ifdef DEBUG_PRINT
void SwerveDrive_PrintWheelData(const SwerveDriveData* data);
#endif

#endif /* INC_SWERVE_DRIVE_H_ */
