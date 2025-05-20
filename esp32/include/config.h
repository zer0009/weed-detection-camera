// Configuration file for Weed Farm Robot
// config.h

#ifndef CONFIG_H
#define CONFIG_H

// Motor control parameters
#define MAX_MOTOR_SPEED 255
#define DEFAULT_MOTOR_SPEED 180

// UART Communication parameters
#define UART_PORT 1
#define BAUD_RATE 115200
#define SERIAL_TX_PIN 10
#define SERIAL_RX_PIN 9

// Delta robot parameters
#define MAX_X 100  // Maximum X coordinate for delta robot
#define MAX_Y 100  // Maximum Y coordinate for delta robot
#define MAX_Z 100  // Maximum Z coordinate for delta robot

// Navigation parameters
#define MAX_LINES 20  // Maximum number of lines the robot can process
#define MAX_LINE_LENGTH 1000  // Maximum line length in cm

// Timings
#define DEBOUNCE_TIME 200  // Button debounce time in ms
#define WEEDING_TIMEOUT 10000  // Timeout for weeding operation in ms

#endif // CONFIG_H