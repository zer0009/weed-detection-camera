// Weed Farm Robot - ESP32 Arduino Code
// Controls a weed removal robot with navigation and delta robot functionality

// Include necessary libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <ESP32Servo.h>

// Include our project modules
#include "config.h"
#include "serial_comm.h"
#include "delta_control.h"

// LCD Configuration
LiquidCrystal_I2C lcd(0x27, 16, 2); // Set the LCD address to 0x27 for a 16 chars and 2 line display

// Button pins
#define BUTTON_UP 12
#define BUTTON_DOWN 14
#define BUTTON_SELECT 27

// Motor control pins
#define MOTOR_LEFT_ENA 25
#define MOTOR_LEFT_IN1 33
#define MOTOR_LEFT_IN2 32
#define MOTOR_RIGHT_ENA 26
#define MOTOR_RIGHT_IN1 4
#define MOTOR_RIGHT_IN2 2

// PWM settings for motors
#define MOTOR_FREQ 1000
#define MOTOR_RESOLUTION 8
#define MOTOR_CHANNEL_LEFT 0
#define MOTOR_CHANNEL_RIGHT 1

// Delta robot servo pins
#define SERVO_1_PIN 13
#define SERVO_2_PIN 15
#define SERVO_3_PIN 16
#define GRIPPER_PIN 17

// Navigation constants
#define WHEEL_DIAMETER_CM 22.0
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159)
#define MOTOR_SPEED 180 // 0-255 PWM value
#define TURN_DELAY 1500 // Time in ms to turn 90 degrees

// Robot states
enum RobotState {
  SETUP_MODE,
  RUNNING_MODE,
  WEED_REMOVAL_MODE,
  PAUSED_MODE
};

// Global variables
RobotState currentState = SETUP_MODE;
int numLines = 1;
int lineLength = 100; // in cm
int currentLine = 0;
float distanceTraveled = 0.0;
unsigned long lastEncoderTime = 0;
bool isMovingForward = false;
bool isWeeding = false;

// Servo objects for delta robot
Servo servo1;
Servo servo2;
Servo servo3;
Servo gripper;

// Function prototypes
void setupLCD();
void setupButtons();
void setupMotors();
void setupDeltaRobot();
void displaySetupMenu();
void handleButtons();
void moveForward();
void stopMotors();
void turnRight();
void turnLeft();
void updateNavigation();
void handleWeedRemoval();
void executeWeedingSequence(int x, int y);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println("Weed Farm Robot Initializing...");
  
  // Initialize components
  setupLCD();
  setupButtons();
  setupMotors();
  setupDeltaRobot();
  init_serial();
  init_delta_control();
  
  // Display welcome message
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Weed Farm Robot");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");
  delay(2000);
  
  // Start in setup mode
  displaySetupMenu();
}

void loop() {
  // Handle button presses
  handleButtons();
  
  // Handle serial communication with Raspberry Pi
  handle_serial_data();
  
  // State machine for robot operation
  switch (currentState) {
    case SETUP_MODE:
      // Setup mode is handled by button presses
      break;
      
    case RUNNING_MODE:
      // Update navigation and check for end of line
      updateNavigation();
      
      // Check for weed coordinates from Raspberry Pi
      int weedX, weedY;
      if (get_weed_coordinates(&weedX, &weedY)) {
        stopMotors();
        currentState = WEED_REMOVAL_MODE;
        executeWeedingSequence(weedX, weedY);
      }
      break;
      
    case WEED_REMOVAL_MODE:
      // Handle weed removal process
      handleWeedRemoval();
      break;
      
    case PAUSED_MODE:
      // Do nothing in paused mode, wait for button press
      break;
  }
  
  // Small delay to prevent CPU hogging
  delay(10);
}

void setupLCD() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
}

void setupButtons() {
  pinMode(BUTTON_UP, INPUT_PULLUP);
  pinMode(BUTTON_DOWN, INPUT_PULLUP);
  pinMode(BUTTON_SELECT, INPUT_PULLUP);
}

void setupMotors() {
  // Setup motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  
  // Configure PWM channels
  ledcSetup(MOTOR_CHANNEL_LEFT, MOTOR_FREQ, MOTOR_RESOLUTION);
  ledcSetup(MOTOR_CHANNEL_RIGHT, MOTOR_FREQ, MOTOR_RESOLUTION);
  
  // Attach pins to channels
  ledcAttachPin(MOTOR_LEFT_ENA, MOTOR_CHANNEL_LEFT);
  ledcAttachPin(MOTOR_RIGHT_ENA, MOTOR_CHANNEL_RIGHT);
  
  // Initially stop motors
  stopMotors();
}

void setupDeltaRobot() {
  // Initialize servo objects
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Configure servos
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo3.setPeriodHertz(50);
  gripper.setPeriodHertz(50);
  
  // Attach servos to pins
  servo1.attach(SERVO_1_PIN, 500, 2400);
  servo2.attach(SERVO_2_PIN, 500, 2400);
  servo3.attach(SERVO_3_PIN, 500, 2400);
  gripper.attach(GRIPPER_PIN, 500, 2400);
  
  // Move to home position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  gripper.write(0); // Gripper open
  
  delay(1000);
}

void displaySetupMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Lines: ");
  lcd.print(numLines);
  lcd.setCursor(0, 1);
  lcd.print("Length: ");
  lcd.print(lineLength);
  lcd.print("cm");
}

void handleButtons() {
  // Debounce variables
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 200;
  
  // Check if enough time has passed since last button press
  if ((millis() - lastDebounceTime) < debounceDelay) {
    return;
  }
  
  // Read button states
  int upState = digitalRead(BUTTON_UP);
  int downState = digitalRead(BUTTON_DOWN);
  int selectState = digitalRead(BUTTON_SELECT);
  
  // Button handling logic depends on current state
  if (currentState == SETUP_MODE) {
    // Persistent variable to track which parameter is being edited
    static bool editingLines = true;
    
    if (upState == LOW) {
      // UP button pressed
      if (editingLines) {
        numLines = min(numLines + 1, 20); // Maximum 20 lines
      } else {
        lineLength = min(lineLength + 10, 1000); // Increase by 10cm, max 1000cm
      }
      displaySetupMenu();
      lastDebounceTime = millis();
    }
    
    if (downState == LOW) {
      // DOWN button pressed
      if (editingLines) {
        numLines = max(numLines - 1, 1); // Minimum 1 line
      } else {
        lineLength = max(lineLength - 10, 10); // Decrease by 10cm, min 10cm
      }
      displaySetupMenu();
      lastDebounceTime = millis();
    }
    
    if (selectState == LOW) {
      // SELECT button pressed
      if (editingLines) {
        // Switch to editing line length
        editingLines = false;
        lcd.setCursor(0, 0);
        lcd.print("Lines: ");
        lcd.print(numLines);
        lcd.print("  ");
        lcd.setCursor(0, 1);
        lcd.print("Length: ");
        lcd.print(lineLength);
        lcd.print("cm *");
      } else {
        // Start the mission
        currentState = RUNNING_MODE;
        currentLine = 0;
        distanceTraveled = 0.0;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Starting mission");
        lcd.setCursor(0, 1);
        lcd.print("Line: 1/");
        lcd.print(numLines);
        
        // Start moving forward
        moveForward();
        isMovingForward = true;
        
        // Send status to Raspberry Pi
        send_status_to_pi("MISSION_STARTED");
      }
      lastDebounceTime = millis();
    }
  } else if (currentState == RUNNING_MODE || currentState == WEED_REMOVAL_MODE) {
    // During operation, SELECT button pauses/resumes
    if (selectState == LOW) {
      if (currentState != PAUSED_MODE) {
        // Pause operation
        stopMotors();
        currentState = PAUSED_MODE;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Mission Paused");
        lcd.setCursor(0, 1);
        lcd.print("Press SEL resume");
        send_status_to_pi("MISSION_PAUSED");
      } else {
        // Resume operation
        currentState = RUNNING_MODE;
        moveForward();
        isMovingForward = true;
        
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Line: ");
        lcd.print(currentLine + 1);
        lcd.print("/");
        lcd.print(numLines);
        lcd.setCursor(0, 1);
        lcd.print("Dist: ");
        lcd.print(distanceTraveled);
        lcd.print("cm");
        
        send_status_to_pi("MISSION_RESUMED");
      }
      lastDebounceTime = millis();
    }
  }
}

void moveForward() {
  // Set motor direction to forward
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  
  // Set motor speed via PWM
  ledcWrite(MOTOR_CHANNEL_LEFT, MOTOR_SPEED);
  ledcWrite(MOTOR_CHANNEL_RIGHT, MOTOR_SPEED);
  
  isMovingForward = true;
  lastEncoderTime = millis();
}

void stopMotors() {
  // Stop both motors
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  
  // Set PWM to 0
  ledcWrite(MOTOR_CHANNEL_LEFT, 0);
  ledcWrite(MOTOR_CHANNEL_RIGHT, 0);
  
  isMovingForward = false;
}

void turnRight() {
  // Set left motor forward, right motor backward
  digitalWrite(MOTOR_LEFT_IN1, HIGH);
  digitalWrite(MOTOR_LEFT_IN2, LOW);
  digitalWrite(MOTOR_RIGHT_IN1, LOW);
  digitalWrite(MOTOR_RIGHT_IN2, HIGH);
  
  // Set motor speed via PWM
  ledcWrite(MOTOR_CHANNEL_LEFT, MOTOR_SPEED);
  ledcWrite(MOTOR_CHANNEL_RIGHT, MOTOR_SPEED);
  
  // Wait for the turn to complete (90 degrees)
  delay(TURN_DELAY);
  
  // Stop motors after turning
  stopMotors();
  delay(500);
}

void turnLeft() {
  // Set left motor backward, right motor forward
  digitalWrite(MOTOR_LEFT_IN1, LOW);
  digitalWrite(MOTOR_LEFT_IN2, HIGH);
  digitalWrite(MOTOR_RIGHT_IN1, HIGH);
  digitalWrite(MOTOR_RIGHT_IN2, LOW);
  
  // Set motor speed via PWM
  ledcWrite(MOTOR_CHANNEL_LEFT, MOTOR_SPEED);
  ledcWrite(MOTOR_CHANNEL_RIGHT, MOTOR_SPEED);
  
  // Wait for the turn to complete (90 degrees)
  delay(TURN_DELAY);
  
  // Stop motors after turning
  stopMotors();
  delay(500);
}

void updateNavigation() {
  // Simple time-based distance estimation
  // In a real implementation, encoders would provide more accurate readings
  if (isMovingForward) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastEncoderTime;
    lastEncoderTime = currentTime;
    
    // Calculate distance traveled based on wheel rotation
    // This is a simplification - actual implementation should use encoders
    float distanceIncrement = (MOTOR_SPEED / 255.0) * (elapsedTime / 1000.0) * WHEEL_CIRCUMFERENCE_CM * 0.5;
    distanceTraveled += distanceIncrement;
    
    // Update LCD with current status every 500ms
    static unsigned long lastLcdUpdate = 0;
    if (currentTime - lastLcdUpdate > 500) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Line: ");
      lcd.print(currentLine + 1);
      lcd.print("/");
      lcd.print(numLines);
      lcd.setCursor(0, 1);
      lcd.print("Dist: ");
      lcd.print((int)distanceTraveled);
      lcd.print("cm");
      
      lastLcdUpdate = currentTime;
    }
    
    // Check if we've reached the end of current line
    if (distanceTraveled >= lineLength) {
      stopMotors();
      currentLine++;
      
      if (currentLine < numLines) {
        // We have more lines to process
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("End of line ");
        lcd.print(currentLine);
        lcd.setCursor(0, 1);
        lcd.print("Turning...");
        
        // Turn pattern: right, move a bit, right again (U-turn pattern)
        // This assumes the robot is moving in a back-and-forth pattern
        if (currentLine % 2 == 1) {
          // Turn right at the end of odd-numbered lines
          turnRight();
          moveForward(); // Move forward a bit
          delay(1000);
          stopMotors();
          turnRight();
        } else {
          // Turn left at the end of even-numbered lines
          turnLeft();
          moveForward(); // Move forward a bit
          delay(1000);
          stopMotors();
          turnLeft();
        }
        
        // Reset distance for new line
        distanceTraveled = 0.0;
        moveForward();
        
        char statusMessage[32];
        sprintf(statusMessage, "LINE_CHANGE:%d", currentLine + 1);
        send_status_to_pi(statusMessage);
      } else {
        // Mission complete
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Mission Complete!");
        lcd.setCursor(0, 1);
        lcd.print("Lines: ");
        lcd.print(numLines);
        
        send_status_to_pi("MISSION_COMPLETED");
        
        // Reset to setup mode after a delay
        delay(5000);
        currentState = SETUP_MODE;
        displaySetupMenu();
      }
    }
  }
}

void handleWeedRemoval() {
  // Check if weeding operation is complete
  if (!isWeeding) {
    // Return to normal operation
    currentState = RUNNING_MODE;
    moveForward();
    isMovingForward = true;
    
    // Update LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Line: ");
    lcd.print(currentLine + 1);
    lcd.print("/");
    lcd.print(numLines);
    lcd.setCursor(0, 1);
    lcd.print("Dist: ");
    lcd.print((int)distanceTraveled);
    lcd.print("cm");
    
    send_status_to_pi("WEEDING_COMPLETED");
  }
}

void executeWeedingSequence(int x, int y) {
  // Display weeding status
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Removing Weed");
  lcd.setCursor(0, 1);
  lcd.print("X:");
  lcd.print(x);
  lcd.print(" Y:");
  lcd.print(y);
  
  send_status_to_pi("WEEDING_STARTED");
  
  // Set weeding flag
  isWeeding = true;
  
  // Convert image coordinates to delta robot coordinates
  // This would depend on your specific delta robot setup and calibration
  move_delta_to_position(x, y, 20); // Move to position with Z=20 (above weed)
  delay(500);
  
  // Lower gripper
  move_delta_to_position(x, y, 0); // Lower to ground level (Z=0)
  delay(500);
  
  // Grip the weed
  gripper.write(90); // Close gripper
  delay(1000);
  
  // Pull the weed
  move_delta_to_position(x, y, 50); // Pull up
  delay(500);
  
  // Move to discard position
  move_delta_to_position(100, 0, 50); // Move to discard position
  delay(500);
  
  // Release the weed
  gripper.write(0); // Open gripper
  delay(500);
  
  // Return to home position
  move_delta_to_home();
  delay(500);
  
  // Weeding operation complete
  isWeeding = false;
}

// REMOVED the duplicate implementations of these functions
// They should only be defined in delta_control.cpp or similar implementation file

/* REMOVED:
void move_delta_to_position(int x, int y, int z) {
  // Convert Cartesian coordinates to servo angles
  // This is a simplified implementation - real delta robot requires inverse kinematics
  int angle1 = map(x, 0, 100, 45, 135);
  int angle2 = map(y, 0, 100, 45, 135);
  int angle3 = map(z, 0, 100, 45, 135);
  
  // Move servos to calculated positions
  servo1.write(angle1);
  servo2.write(angle2);
  servo3.write(angle3);
}

void move_delta_to_home() {
  // Move delta robot to home position
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  gripper.write(0); // Open gripper
}
*/