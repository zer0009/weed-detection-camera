// Header file for serial communication with Raspberry Pi
// serial_comm.h

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <stdbool.h>

// Initialize serial communication with Raspberry Pi
void init_serial(void);

// Handle incoming serial data (should be called in the main loop)
void handle_serial_data(void);

// Parse weed coordinates from received message
void parse_weed_coordinates(const char* message);

// Get the latest weed coordinates received from Raspberry Pi
// Returns true if new coordinates are available, false otherwise
// Coordinates are returned through the x and y pointers
bool get_weed_coordinates(int* x, int* y);

// Send status message to Raspberry Pi
void send_status_to_pi(const char* status);

#endif // SERIAL_COMM_H