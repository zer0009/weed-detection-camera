// Implementation of delta robot control functions
// delta_control.c

#include "delta_control.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char* TAG = "DELTA";

// Delta robot geometry parameters
#define EFFECTOR_RADIUS 60.0    // End effector radius in mm
#define BASE_RADIUS 100.0       // Base radius in mm
#define ARM_LENGTH 150.0        // Length of the upper arm in mm
#define FOREARM_LENGTH 200.0    // Length of the forearm (parallelogram) in mm

// Servo parameters
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2400

// State variables
static bool delta_moving = false;
static int current_x = 0;
static int current_y = 0;
static int current_z = 0;
static int gripper_state = 0; // 0=open, 1=closed

// Function prototypes for inverse kinematics calculations
static bool calculate_inverse_kinematics(float x, float y, float z, float* theta1, float* theta2, float* theta3);

void init_delta_control(void) {
    ESP_LOGI(TAG, "Initializing delta robot control system");
    
    // Move to home position on startup
    move_delta_to_home();
    
    ESP_LOGI(TAG, "Delta robot initialized");
}

bool move_delta_to_position(int x, int y, int z) {
    ESP_LOGI(TAG, "Moving delta robot to position: X=%d, Y=%d, Z=%d", x, y, z);
    
    // Validate target position
    if (x < 0 || x > MAX_X || y < 0 || y > MAX_Y || z < 0 || z > MAX_Z) {
        ESP_LOGW(TAG, "Target position out of range");
        return false;
    }
    
    // Convert to actual coordinates in mm
    // This conversion depends on your specific setup
    float x_mm = x - MAX_X/2; // Center origin
    float y_mm = y - MAX_Y/2;
    float z_mm = -z; // Z is negative downward
    
    // Calculate servo angles using inverse kinematics
    float theta1, theta2, theta3;
    if (!calculate_inverse_kinematics(x_mm, y_mm, z_mm, &theta1, &theta2, &theta3)) {
        ESP_LOGW(TAG, "Inverse kinematics calculation failed");
        return false;
    }
    
    // Log calculated angles
    ESP_LOGI(TAG, "Calculated angles: theta1=%.2f, theta2=%.2f, theta3=%.2f", theta1, theta2, theta3);
    
    // Set the delta robot as moving
    delta_moving = true;
    
    // Update current position
    current_x = x;
    current_y = y;
    current_z = z;
    
    // In a real implementation, you would now command the servos to move to these angles
    // This is a placeholder for the actual servo control code
    
    // Simulate servo movement time
    // vTaskDelay(pdMS_TO_TICKS(500));
    
    // Delta robot is now in position
    delta_moving = false;
    
    return true;
}

bool move_delta_to_home(void) {
    ESP_LOGI(TAG, "Moving delta robot to home position");
    
    // Home position is typically at center of workspace with Z at safe height
    int home_x = MAX_X / 2;
    int home_y = MAX_Y / 2;
    int home_z = 20; // Safe height
    
    return move_delta_to_position(home_x, home_y, home_z);
}

bool control_gripper(int action) {
    ESP_LOGI(TAG, "Setting gripper state to: %d", action);
    
    if (action != 0 && action != 1) {
        ESP_LOGW(TAG, "Invalid gripper action");
        return false;
    }
    
    gripper_state = action;
    
    // In a real implementation, you would now command the gripper servo
    // This is a placeholder for the actual servo control code
    
    return true;
}

bool is_delta_moving(void) {
    return delta_moving;
}

bool execute_weeding_sequence(int x, int y) {
    ESP_LOGI(TAG, "Executing weeding sequence at position: X=%d, Y=%d", x, y);
    
    // Step 1: Move to position above weed
    if (!move_delta_to_position(x, y, 30)) {
        ESP_LOGW(TAG, "Failed to move above weed");
        return false;
    }
    
    // Step 2: Open gripper if not already open
    if (gripper_state != 0) {
        if (!control_gripper(0)) {
            ESP_LOGW(TAG, "Failed to open gripper");
            return false;
        }
    }
    
    // Step 3: Move down to weed
    if (!move_delta_to_position(x, y, 0)) {
        ESP_LOGW(TAG, "Failed to move down to weed");
        return false;
    }
    
    // Step 4: Close gripper to grab weed
    if (!control_gripper(1)) {
        ESP_LOGW(TAG, "Failed to close gripper");
        return false;
    }
    
    // Step 5: Pull weed up
    if (!move_delta_to_position(x, y, 50)) {
        ESP_LOGW(TAG, "Failed to pull weed up");
        return false;
    }
    
    // Step 6: Move to discard position
    int discard_x = MAX_X - 10;
    int discard_y = MAX_Y / 2;
    if (!move_delta_to_position(discard_x, discard_y, 50)) {
        ESP_LOGW(TAG, "Failed to move to discard position");
        return false;
    }
    
    // Step 7: Open gripper to drop weed
    if (!control_gripper(0)) {
        ESP_LOGW(TAG, "Failed to open gripper for discard");
        return false;
    }
    
    // Step 8: Return to home position
    if (!move_delta_to_home()) {
        ESP_LOGW(TAG, "Failed to return to home position");
        return false;
    }
    
    ESP_LOGI(TAG, "Weeding sequence completed successfully");
    return true;
}

static bool calculate_inverse_kinematics(float x, float y, float z, float* theta1, float* theta2, float* theta3) {
    // This is a simplified inverse kinematics implementation for a delta robot
    // In a real implementation, you would use the actual geometry of your delta robot
    
    // Placeholder implementation - in reality you would use proper trigonometric calculations
    // based on the geometry of your specific delta robot
    
    // Convert cartesian coordinates to angular positions
    // This is a very simplified approximation, not actual delta robot kinematics
    float center_dist = sqrt(x*x + y*y);
    float height_factor = MAX_Z - z;
    
    *theta1 = 90 - atan2(center_dist, height_factor) * 180.0f / M_PI;
    
    // For a real delta robot, these would be calculated using proper inverse kinematics
    // Here we're just offsetting by 120 degrees to simulate the three arms
    *theta2 = *theta1;
    if (y < 0) {
        *theta2 += 10;
    } else {
        *theta2 -= 10;
    }
    
    *theta3 = *theta1;
    if (x < 0) {
        *theta3 += 10;
    } else {
        *theta3 -= 10;
    }
    
    // Ensure angles are within valid range
    if (*theta1 < SERVO_MIN_ANGLE || *theta1 > SERVO_MAX_ANGLE ||
        *theta2 < SERVO_MIN_ANGLE || *theta2 > SERVO_MAX_ANGLE ||
        *theta3 < SERVO_MIN_ANGLE || *theta3 > SERVO_MAX_ANGLE) {
        ESP_LOGW(TAG, "Calculated angles out of range");
        return false;
    }
    
    return true;
}