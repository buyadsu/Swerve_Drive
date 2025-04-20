#include "swerve_drive.h"
#include <math.h>

// Global state for previous angles
static float prev_angles[4] = {0};  // RF, LF, RB, LB
static LockMode current_lock_mode = LOCK_MODE_DISABLED;
static bool is_initialized = false;  // Track initialization state

void SD_Init(void) {
    // Initialize previous angles array to 0
    for(int i = 0; i < 4; i++) {
        prev_angles[i] = 0.0f;
    }
    current_lock_mode = LOCK_MODE_DISABLED;
    is_initialized = true;
}

void SD_SetLockMode(LockMode mode) {
    current_lock_mode = mode;
    // Reset previous angles when changing lock mode
    for(int i = 0; i < 4; i++) {
        prev_angles[i] = 0.0f;
    }
}

LockMode SD_GetLockMode(void) {
    return current_lock_mode;
}

void SD_UpdateKinematics(float xSpeed, float ySpeed, float rot, SwerveDriveData* data) {
    // Front Right (RF)
    float rf_x = xSpeed + (rot * (ROBOT_LENGTH / 2.0f));
    float rf_y = ySpeed - (rot * (ROBOT_WIDTH / 2.0f));
    data->rf.angle = atan2f(rf_x, rf_y) * (180.0f / (float)M_PI) + 180.0f;
    data->rf.speed = sqrtf(rf_x * rf_x + rf_y * rf_y);

    // Front Left (LF)
    float lf_x = xSpeed + (rot * (ROBOT_LENGTH / 2.0f));
    float lf_y = ySpeed + (rot * (ROBOT_WIDTH / 2.0f));
    data->lf.angle = atan2f(lf_x, lf_y) * (180.0f / (float)M_PI) + 180.0f;
    data->lf.speed = sqrtf(lf_x * lf_x + lf_y * lf_y);

    // Rear Right (RB)
    float rb_x = xSpeed - (rot * (ROBOT_LENGTH / 2.0f));
    float rb_y = ySpeed - (rot * (ROBOT_WIDTH / 2.0f));
    data->rb.angle = atan2f(rb_x, rb_y) * (180.0f / (float)M_PI) + 180.0f;
    data->rb.speed = sqrtf(rb_x * rb_x + rb_y * rb_y);

    // Rear Left (LB)
    float lb_x = xSpeed - (rot * (ROBOT_LENGTH / 2.0f));
    float lb_y = ySpeed + (rot * (ROBOT_WIDTH / 2.0f));
    data->lb.angle = atan2f(lb_x, lb_y) * (180.0f / (float)M_PI) + 180.0f;
    data->lb.speed = sqrtf(lb_x * lb_x + lb_y * lb_y);

    // Normalize speeds if any exceeds 1.0
    float max_speed = fmaxf(fmaxf(data->rf.speed, data->lf.speed), 
                          fmaxf(data->rb.speed, data->lb.speed));
    if (max_speed > 1.0f) {
        data->rf.speed /= max_speed;
        data->lf.speed /= max_speed;
        data->rb.speed /= max_speed;
        data->lb.speed /= max_speed;
    }
}

void SD_OptimizeAngle(float* angle, float* speed, int module_idx) {
    // Preserve original vector direction
    float original_angle = *angle;
    *speed = fabsf(*speed);

    // Normalize to 0-360
    *angle = fmodf(*angle + 360.0f, 360.0f);

    // Only calculate shortest path if we're not in the initial state
    if (is_initialized) {
        // Calculate shortest path from previous angle
        float angle_diff = *angle - prev_angles[module_idx];
        if (angle_diff > 180.0f) {
            *angle -= 360.0f;
        } else if (angle_diff < -180.0f) {
            *angle += 360.0f;
        }
    }

    // For pure backward motion (ySpeed = -1), maintain 180° orientation
    if (fabsf(original_angle - 180.0f) < STEERING_DEADZONE) {
        *angle = 180.0f;
    }

    // Update previous angle
    prev_angles[module_idx] = *angle;

    // Only apply lock modes when completely stopped (speed < SPEED_DEADZONE)
    if (*speed < SPEED_DEADZONE) {
        switch (current_lock_mode) {
            case LOCK_MODE_DISABLED:
                *angle = 0.0f;  // Return to neutral when stopped
                *speed = 0.0f;
                prev_angles[module_idx] = 0.0f;
                break;

            case LOCK_MODE_LAST_ANGLE:
                // Keep the last angle, just set speed to 0
                *speed = 0.0f;
                // Ensure we don't have any 180-degree flips
                if (fabsf(*angle - prev_angles[module_idx]) > 90.0f) {
                    *angle = prev_angles[module_idx];
                }
                break;

            case LOCK_MODE_45_DEG:
                // Set to one of the lock angles based on module index
                switch (module_idx) {
                    case RF:
                        *angle = LOCK_ANGLE_1;
                        break;
                    case LF:
                        *angle = LOCK_ANGLE_2;
                        break;
                    case RB:
                        *angle = LOCK_ANGLE_3;
                        break;
                    case LB:
                        *angle = LOCK_ANGLE_4;
                        break;
                }
                *speed = 0.0f;
                prev_angles[module_idx] = *angle;
                break;
        }
    }
}

void SD_ApplyDeadband(float* xSpeed, float* ySpeed, float* rot) {
    // Apply deadzone to joystick inputs
    if (fabsf(*xSpeed) < DEADZONE) *xSpeed = 0.0f;
    if (fabsf(*ySpeed) < DEADZONE) *ySpeed = 0.0f;
    if (fabsf(*rot) < DEADZONE) *rot = 0.0f;

    // Smooth input changes to prevent jerky motion
    static float prev_xSpeed = 0.0f;
    static float prev_ySpeed = 0.0f;
    static float prev_rot = 0.0f;

    *xSpeed = prev_xSpeed + fminf(fmaxf(*xSpeed - prev_xSpeed, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);
    *ySpeed = prev_ySpeed + fminf(fmaxf(*ySpeed - prev_ySpeed, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);
    *rot = prev_rot + fminf(fmaxf(*rot - prev_rot, -MAX_CHANGE_RATE), MAX_CHANGE_RATE);

    prev_xSpeed = *xSpeed;
    prev_ySpeed = *ySpeed;
    prev_rot = *rot;
}

void SD_UpdateModules(SwerveDriveData* data, SwerveModule* moduleRF, SwerveModule* moduleLF, 
                     SwerveModule* moduleRB, SwerveModule* moduleLB) {
    // Optimize angles for each module
    SD_OptimizeAngle(&data->rf.angle, &data->rf.speed, RF);
    SD_OptimizeAngle(&data->lf.angle, &data->lf.speed, LF);
    SD_OptimizeAngle(&data->rb.angle, &data->rb.speed, RB);
    SD_OptimizeAngle(&data->lb.angle, &data->lb.speed, LB);

    // Update each module
    SM_UpdateSteering(moduleRF, data->rf.angle);
    SM_UpdateDriving(moduleRF, data->rf.speed);

    SM_UpdateSteering(moduleLF, data->lf.angle);
    SM_UpdateDriving(moduleLF, data->lf.speed);

    SM_UpdateSteering(moduleRB, data->rb.angle);
    SM_UpdateDriving(moduleRB, data->rb.speed);

    SM_UpdateSteering(moduleLB, data->lb.angle);
    SM_UpdateDriving(moduleLB, data->lb.speed);
}
