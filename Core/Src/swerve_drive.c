/*
 * swerve_drive.c
 *
 *  Created on: Mar 27, 2025
 *      Author: root
 */

#include "swerve_drive.h"
#include "main.h"
#include <string.h>
#include <stdio.h>

// Private variables
static SwerveDriveData wheel_data;
static float prev_angles[4] = {0};  // Previous angles for each module

// Private function declarations
static void optimize_angle(float* angle, float* speed, int module_idx);

void SwerveDrive_Init(void) {
    // Initialize wheel data structure
    memset(&wheel_data, 0, sizeof(SwerveDriveData));
    memset(prev_angles, 0, sizeof(prev_angles));
    
    SWERVE_DEBUG_PRINT("SwerveDrive initialized\n");
}

void SwerveDrive_Update(float x_speed, float y_speed, float rotation) {
    // Calculate wheel speeds and angles based on desired robot motion
    
    // Front Right (RF)
    float rf_x = x_speed - (rotation * (ROBOT_LENGTH / 2.0f));
    float rf_y = y_speed + (rotation * (ROBOT_WIDTH / 2.0f));
    wheel_data.rf.angle = atan2f(rf_x, rf_y) * (180.0f / (float)M_PI);
    wheel_data.rf.speed = sqrtf(rf_x * rf_x + rf_y * rf_y);

    // Front Left (LF)
    float lf_x = x_speed + (rotation * (ROBOT_LENGTH / 2.0f));
    float lf_y = y_speed + (rotation * (ROBOT_WIDTH / 2.0f));
    wheel_data.lf.angle = atan2f(lf_x, lf_y) * (180.0f / (float)M_PI);
    wheel_data.lf.speed = sqrtf(lf_x * lf_x + lf_y * lf_y);

    // Rear Right (RB)
    float rb_x = x_speed - (rotation * (ROBOT_LENGTH / 2.0f));
    float rb_y = y_speed - (rotation * (ROBOT_WIDTH / 2.0f));
    wheel_data.rb.angle = atan2f(rb_x, rb_y) * (180.0f / (float)M_PI);
    wheel_data.rb.speed = sqrtf(rb_x * rb_x + rb_y * rb_y);

    // Rear Left (LB)
    float lb_x = x_speed + (rotation * (ROBOT_LENGTH / 2.0f));
    float lb_y = y_speed - (rotation * (ROBOT_WIDTH / 2.0f));
    wheel_data.lb.angle = atan2f(lb_x, lb_y) * (180.0f / (float)M_PI);
    wheel_data.lb.speed = sqrtf(lb_x * lb_x + lb_y * lb_y);

    // Normalize speeds if any exceeds 1.0
    float max_speed = fmaxf(fmaxf(wheel_data.rf.speed, wheel_data.lf.speed), 
                           fmaxf(wheel_data.rb.speed, wheel_data.lb.speed));
    if (max_speed > 1.0f) {
        wheel_data.rf.speed /= max_speed;
        wheel_data.lf.speed /= max_speed;
        wheel_data.rb.speed /= max_speed;
        wheel_data.lb.speed /= max_speed;
    }

    // Apply optimization to all modules
    optimize_angle(&wheel_data.rf.angle, &wheel_data.rf.speed, MODULE_RF);
    optimize_angle(&wheel_data.lf.angle, &wheel_data.lf.speed, MODULE_LF);
    optimize_angle(&wheel_data.rb.angle, &wheel_data.rb.speed, MODULE_RB);
    optimize_angle(&wheel_data.lb.angle, &wheel_data.lb.speed, MODULE_LB);

    // Apply speed deadzone
    if (fabsf(wheel_data.rf.speed) < SPEED_DEADZONE) wheel_data.rf.angle = 0.0f;
    if (fabsf(wheel_data.lf.speed) < SPEED_DEADZONE) wheel_data.lf.angle = 0.0f;
    if (fabsf(wheel_data.rb.speed) < SPEED_DEADZONE) wheel_data.rb.angle = 0.0f;
    if (fabsf(wheel_data.lb.speed) < SPEED_DEADZONE) wheel_data.lb.angle = 0.0f;

    // Debug print if enabled
    SWERVE_DEBUG_PRINT("Input: x=%.2f y=%.2f rot=%.2f\n", x_speed, y_speed, rotation);
    SwerveDrive_PrintWheelData(&wheel_data);

    // Apply the calculated data to the wheels
    SwerveDrive_ApplyWheelData(&wheel_data);
}

void SwerveDrive_GetWheelData(SwerveDriveData* data) {
    // Copy current wheel data to provided structure
    memcpy(data, &wheel_data, sizeof(SwerveDriveData));
}

void SwerveDrive_ApplyWheelData(SwerveDriveData* data) {
    // Apply the wheel data to each module
    SM_SetSteeringAngle(&moduleRF, data->rf.angle);
    SM_SetDrivingSpeed(&moduleRF, data->rf.speed);
    
    SM_SetSteeringAngle(&moduleLF, data->lf.angle);
    SM_SetDrivingSpeed(&moduleLF, data->lf.speed);
    
    SM_SetSteeringAngle(&moduleRB, data->rb.angle);
    SM_SetDrivingSpeed(&moduleRB, data->rb.speed);
    
    SM_SetSteeringAngle(&moduleLB, data->lb.angle);
    SM_SetDrivingSpeed(&moduleLB, data->lb.speed);
}

void SwerveDrive_Stop(void) {
    // Set all wheel speeds to zero
    wheel_data.rf.speed = 0.0f;
    wheel_data.lf.speed = 0.0f;
    wheel_data.rb.speed = 0.0f;
    wheel_data.lb.speed = 0.0f;
    
    // Apply the stop command
    SwerveDrive_ApplyWheelData(&wheel_data);
    
    SWERVE_DEBUG_PRINT("SwerveDrive stopped\n");
}

#ifdef DEBUG_PRINT
void SwerveDrive_PrintWheelData(const SwerveDriveData* data) {
    printf("Wheel Data:\n");
    printf("RF: angle=%.1f speed=%.2f\n", data->rf.angle, data->rf.speed);
    printf("LF: angle=%.1f speed=%.2f\n", data->lf.angle, data->lf.speed);
    printf("RB: angle=%.1f speed=%.2f\n", data->rb.angle, data->rb.speed);
    printf("LB: angle=%.1f speed=%.2f\n", data->lb.angle, data->lb.speed);
    printf("\n");
}
#endif

static void optimize_angle(float* angle, float* speed, int module_idx) {
    // Preserve original vector direction
    float original_angle = *angle;
    *speed = fabsf(*speed);

    // Normalize to 0-360
    *angle = fmodf(*angle + 360.0f, 360.0f);

    // Calculate shortest path from previous angle
    float angle_diff = *angle - prev_angles[module_idx];
    if (angle_diff > 180.0f) {
        *angle -= 360.0f;
    } else if (angle_diff < -180.0f) {
        *angle += 360.0f;
    }

    // For pure backward motion (ySpeed = -1), maintain 180° orientation
    if (fabsf(original_angle - 180.0f) < STEERING_DEADZONE) {
        *angle = 180.0f;
    }

    // Update previous angle
    prev_angles[module_idx] = *angle;

    // Apply steering deadzone
    if (*speed < SPEED_DEADZONE) {
        *angle = 0.0f;  // Return to neutral when stopped
        *speed = 0.0f;
        prev_angles[module_idx] = 0.0f;
    }
}
