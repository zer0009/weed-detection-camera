// Implementation of delta robot control functions
// delta_control.cpp

#include "delta_control.h"
#include "config.h"
#include <math.h>
#include <Arduino.h>

// External servo objects from main.cpp
extern Servo servo1;
extern Servo servo2;
extern Servo servo3;
extern Servo gripper;

// Delta robot geometry parameters
#define EFFECTOR_RADIUS 60.0    // End effector radius in mm
#define BASE_RADIUS 100.0       // Base radius in mm
#define ARM_LENGTH 150.0        // Length of the upper arm in mm
#define FOREARM_LENGTH 200.0    // Length of the forearm (parallelogram) in mm

// Servo parameters
#define SERVO_MIN_ANGLE 30
#define SERVO_MAX_ANGLE 150
#define SERVO_HOME_POS 90

// State variables
static bool delta_moving = false;
static int current_x = 50;
static int current_y = 50;
static int current_z = 50;
static int gripper_state = 0; // 0=open, 1=closed

// Function prototypes for inverse kinematics calculations
static bool calculate_inverse_kinematics(float x, float y, float z, float* theta1, float* theta2, float* theta3);
static void move_servo_gradually(Servo &servo, int target_angle, int delay_ms = 15);

void init_delta_control(void) {
    Serial.println("Initializing delta robot control system");
    
    // Test servos on startup to verify they're working
    test_servos();
    
    // Move to home position on startup
    move_delta_to_home();
    
    Serial.println("Delta robot initialized and ready");
}

void test_servos(void) {
    Serial.println("Testing servos to verify operation...");
    
    // Test servo1 (small movement)
    Serial.println("Testing servo 1");
    move_servo_gradually(servo1, SERVO_HOME_POS - 20);
    delay(300);
    move_servo_gradually(servo1, SERVO_HOME_POS + 20);
    delay(300);
    move_servo_gradually(servo1, SERVO_HOME_POS);
    delay(300);
    
    // Test servo2 (small movement)
    Serial.println("Testing servo 2");
    move_servo_gradually(servo2, SERVO_HOME_POS - 20);
    delay(300);
    move_servo_gradually(servo2, SERVO_HOME_POS + 20);
    delay(300);
    move_servo_gradually(servo2, SERVO_HOME_POS);
    delay(300);
    
    // Test servo3 (small movement)
    Serial.println("Testing servo 3");
    move_servo_gradually(servo3, SERVO_HOME_POS - 20);
    delay(300);
    move_servo_gradually(servo3, SERVO_HOME_POS + 20);
    delay(300);
    move_servo_gradually(servo3, SERVO_HOME_POS);
    delay(300);
    
    // Test gripper
    Serial.println("Testing gripper");
    gripper.write(0);  // Open
    delay(500);
    gripper.write(90); // Close
    delay(500);
    gripper.write(0);  // Open
    delay(500);
    
    Serial.println("Servo test complete");
}

bool move_delta_to_position(int x, int y, int z) {
    Serial.printf("Moving delta robot to position: X=%d, Y=%d, Z=%d\n", x, y, z);
    
    // Validate target position
    if (x < 0 || x > MAX_X || y < 0 || y > MAX_Y || z < 0 || z > MAX_Z) {
        Serial.println("Target position out of range");
        return false;
    }
    
    // Convert to actual coordinates in mm
    float x_mm = map(x, 0, MAX_X, -100, 100);
    float y_mm = map(y, 0, MAX_Y, -100, 100);
    float z_mm = map(z, 0, MAX_Z, -100, 0);
    
    // Calculate servo angles using inverse kinematics
    float theta1, theta2, theta3;
    if (!calculate_inverse_kinematics(x_mm, y_mm, z_mm, &theta1, &theta2, &theta3)) {
        Serial.println("Inverse kinematics calculation failed");
        return false;
    }
    
    // Log calculated angles
    Serial.printf("Calculated angles: theta1=%.2f, theta2=%.2f, theta3=%.2f\n", 
                 theta1, theta2, theta3);
    
    // Set the delta robot as moving
    delta_moving = true;
    
    // Convert float angles to integers for servo write
    int angle1 = (int)theta1;
    int angle2 = (int)theta2;
    int angle3 = (int)theta3;
    
    // Constrain angles to valid servo range
    angle1 = constrain(angle1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    angle2 = constrain(angle2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    angle3 = constrain(angle3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Move servos to calculated positions
    move_servo_gradually(servo1, angle1);
    move_servo_gradually(servo2, angle2);
    move_servo_gradually(servo3, angle3);
    
    // Update current position
    current_x = x;
    current_y = y;
    current_z = z;
    
    // Delta robot is now in position
    delta_moving = false;
    
    return true;
}

bool move_delta_to_home(void) {
    Serial.println("Moving delta robot to home position");
    
    // Directly set servos to home position
    move_servo_gradually(servo1, SERVO_HOME_POS);
    move_servo_gradually(servo2, SERVO_HOME_POS);
    move_servo_gradually(servo3, SERVO_HOME_POS);
    
    // Open gripper
    gripper.write(0);
    
    // Update current position
    current_x = MAX_X / 2;  // Center X
    current_y = MAX_Y / 2;  // Center Y
    current_z = 50;         // Safe height
    
    Serial.println("Delta robot now at home position");
    return true;
}

bool control_gripper(int action) {
    Serial.printf("Setting gripper state to: %d\n", action);
    
    if (action != 0 && action != 1) {
        Serial.println("Invalid gripper action");
        return false;
    }
    
    // Set gripper state
    if (action == 0) {
        // Open gripper
        gripper.write(0);
    } else {
        // Close gripper
        gripper.write(90);
    }
    
    gripper_state = action;
    return true;
}

bool is_delta_moving(void) {
    return delta_moving;
}

bool execute_weeding_sequence(int x, int y) {
    Serial.printf("Executing weeding sequence at position: X=%d, Y=%d\n", x, y);
    
    // Step 1: Move to position above weed
    if (!move_delta_to_position(x, y, 30)) {
        Serial.println("Failed to move above weed");
        return false;
    }
    delay(500);
    
    // Step 2: Open gripper if not already open
    if (gripper_state != 0) {
        if (!control_gripper(0)) {
            Serial.println("Failed to open gripper");
            return false;
        }
    }
    delay(500);
    
    // Step 3: Move down to weed
    if (!move_delta_to_position(x, y, 0)) {
        Serial.println("Failed to move down to weed");
        return false;
    }
    delay(500);
    
    // Step 4: Close gripper to grab weed
    if (!control_gripper(1)) {
        Serial.println("Failed to close gripper");
        return false;
    }
    delay(1000);
    
    // Step 5: Pull weed up
    if (!move_delta_to_position(x, y, 50)) {
        Serial.println("Failed to pull weed up");
        return false;
    }
    delay(500);
    
    // Step 6: Move to discard position
    int discard_x = MAX_X - 10;
    int discard_y = MAX_Y / 2;
    if (!move_delta_to_position(discard_x, discard_y, 50)) {
        Serial.println("Failed to move to discard position");
        return false;
    }
    delay(500);
    
    // Step 7: Open gripper to drop weed
    if (!control_gripper(0)) {
        Serial.println("Failed to open gripper for discard");
        return false;
    }
    delay(500);
    
    // Step 8: Return to home position
    if (!move_delta_to_home()) {
        Serial.println("Failed to return to home position");
        return false;
    }
    
    Serial.println("Weeding sequence completed successfully");
    return true;
}

// Move servo gradually to prevent jerky motion
static void move_servo_gradually(Servo &servo, int target_angle, int delay_ms) {
    int current_angle = servo.read();
    
    // If current angle is 0 (default), start from SERVO_HOME_POS
    if (current_angle == 0) {
        current_angle = SERVO_HOME_POS;
    }
    
    // If current position is close to target, just move directly
    if (abs(current_angle - target_angle) < 5) {
        servo.write(target_angle);
        return;
    }
    
    // Move servo gradually to target position
    int step = (target_angle > current_angle) ? 1 : -1;
    
    while (current_angle != target_angle) {
        current_angle += step;
        servo.write(current_angle);
        delay(delay_ms);  // Small delay between movements
    }
}

static bool calculate_inverse_kinematics(float x, float y, float z, float* theta1, float* theta2, float* theta3) {
    // We'll use a simplified approach for delta robot kinematics
    // This will make the robot move in ways that approximate the correct movement
    
    // Base offset angles for the three arms (120 degrees apart)
    const float arm1_angle = 0.0;      // First arm at 0 degrees
    const float arm2_angle = 120.0;    // Second arm at 120 degrees
    const float arm3_angle = 240.0;    // Third arm at 240 degrees
    
    // Calculate distance from center
    float center_dist = sqrt(x*x + y*y);
    
    // Basic angle based on height (z) - the higher z is, the smaller the angle
    float base_angle = map(z, -100, 0, 60, 120);
    
    // Calculate angular offset based on x,y position
    float xy_angle = 0;
    if (center_dist > 0.1) {  // Avoid division by zero
        // Calculate direction angle in degrees
        xy_angle = atan2(y, x) * 180.0 / PI;
        
        // Normalize angle to 0-360 range
        if (xy_angle < 0) xy_angle += 360.0;
    }
    
    // Calculate offset for each servo based on direction
    // Servos move outward as the end effector moves toward them,
    // and inward as it moves away
    
    // Calculate angle offset for each arm based on direction
    float offset1 = 20.0 * cos((xy_angle - arm1_angle) * PI / 180.0) * (center_dist / 100.0);
    float offset2 = 20.0 * cos((xy_angle - arm2_angle) * PI / 180.0) * (center_dist / 100.0);
    float offset3 = 20.0 * cos((xy_angle - arm3_angle) * PI / 180.0) * (center_dist / 100.0);
    
    // Apply offsets to base angle
    *theta1 = base_angle + offset1;
    *theta2 = base_angle + offset2;
    *theta3 = base_angle + offset3;
    
    // Ensure angles are within valid range
    if (*theta1 < SERVO_MIN_ANGLE || *theta1 > SERVO_MAX_ANGLE ||
        *theta2 < SERVO_MIN_ANGLE || *theta2 > SERVO_MAX_ANGLE ||
        *theta3 < SERVO_MIN_ANGLE || *theta3 > SERVO_MAX_ANGLE) {
        Serial.println("Calculated angles out of range");
        
        // Constrain to valid range for safety
        *theta1 = constrain(*theta1, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        *theta2 = constrain(*theta2, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        *theta3 = constrain(*theta3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        
        return false;
    }
    
    return true;
}