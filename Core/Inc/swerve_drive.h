#ifndef INC_SWERVE_DRIVE_H
#define INC_SWERVE_DRIVE_H

#include "main.h"
#include "swerve_module.h"
#include <stdbool.h>

// Robot dimensions (adjust according to actual measurements)
#define ROBOT_LENGTH 0.5f // Distance from front to back wheels (meters)
#define ROBOT_WIDTH 0.5f  // Distance from left to right wheels (meters)
#define ROBOT_STEERING_GEAR_RATIO 2.0f // Gear ratio of steering motors
#define STEERING_ENCODER_RESOLUTION 1000.0 // Encoder resolution
#define STEERING_DEADZONE 2.0f  // Degrees
#define SPEED_DEADZONE 0.02f    // Reduced from 0.05f to be more responsive at low speeds
#define DEADZONE 0.02f     // Reduced deadzone for more responsive control
#define MAX_CHANGE_RATE 0.7f  // Increased for faster response
#define TIMEOUT_MS 500         // Timeout for joystick disconnect detection

// Lock mode angles (in degrees)
#define LOCK_ANGLE_1 45.0f
#define LOCK_ANGLE_2 135.0f
#define LOCK_ANGLE_3 315.0f
#define LOCK_ANGLE_4 225.0f

// Module indices for easier reference
enum ModuleIndices {
    RF = 0,
    LF = 1,
    RB = 2,
    LB = 3
};

// Structure to hold wheel data
typedef struct {
    float angle;    // Steering angle in degrees
    float speed;    // Wheel speed (-1.0 to 1.0)
} WheelData;

// Structure to hold all wheel data
typedef struct {
    WheelData rf;   // Right Front
    WheelData lf;   // Left Front
    WheelData rb;   // Right Back
    WheelData lb;   // Left Back
} SwerveDriveData;

// Lock mode configuration
typedef enum {
    LOCK_MODE_DISABLED = 0,    // Wheels return to 0 degrees when stopped
    LOCK_MODE_LAST_ANGLE = 1,  // Wheels maintain last angle when stopped
    LOCK_MODE_45_DEG = 2       // Wheels lock at 45-degree angles when stopped
} LockMode;

// Function declarations
void SD_Init(void);
void SD_UpdateKinematics(float xSpeed, float ySpeed, float rot, SwerveDriveData* data);
void SD_OptimizeAngle(float* angle, float* speed, int module_idx);
void SD_ApplyDeadband(float* xSpeed, float* ySpeed, float* rot);
void SD_UpdateModules(SwerveDriveData* data, SwerveModule* moduleRF, SwerveModule* moduleLF, 
                     SwerveModule* moduleRB, SwerveModule* moduleLB);
void SD_SetLockMode(LockMode mode);
LockMode SD_GetLockMode(void);

#endif /* SWERVE_DRIVE_H */ 
