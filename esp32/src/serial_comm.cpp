// serial_comm.cpp - Handles UART communication with the Raspberry Pi
// Arduino/PlatformIO compatible version

#include <Arduino.h>
#include "serial_comm.h"
#include "config.h"
#include "delta_control.h"

#define BUF_SIZE 128
static char rx_buffer[BUF_SIZE];
static int buf_pos = 0;

// Coordinates received from Raspberry Pi
static int last_x = -1;
static int last_y = -1;
static bool new_coords_available = false;

// Serial object for communication with Raspberry Pi
// ESP32 has 3 hardware serial ports
// Serial (UART0) is typically used for programming/debug
// Serial1 (UART1) and Serial2 (UART2) are available for other use
HardwareSerial PiSerial(UART_PORT);

void init_serial(void) {
  Serial.println("Initializing UART communication with Raspberry Pi");
  
  // Configure the hardware serial port
  // Note: On ESP32, UART_PORT 2 corresponds to HardwareSerial(2)
  PiSerial.begin(BAUD_RATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  
  // Clear any existing data
  while (PiSerial.available()) {
    PiSerial.read();
  }
  
  Serial.printf("UART initialized on port %d, RX: %d, TX: %d, Baud: %d\n", 
                UART_PORT, SERIAL_RX_PIN, SERIAL_TX_PIN, BAUD_RATE);
}

void handle_serial_data(void) {
  // Check if data is available
  while (PiSerial.available()) {
    char data = (char)PiSerial.read();
    
    // Process the received byte
    if (data == '\n') {
      // End of message, process the complete line
      rx_buffer[buf_pos] = '\0';  // Null-terminate the string
      parse_weed_coordinates(rx_buffer);
      buf_pos = 0;  // Reset buffer position
    } else if (buf_pos < BUF_SIZE - 1) {
      // Add character to buffer
      rx_buffer[buf_pos++] = data;
    }
  }
}

void parse_weed_coordinates(const char* message) {
  Serial.printf("Received message: %s\n", message);
  
  // Look for X=<number> pattern
  char *x_str = strstr(message, "X=");
  char *y_str = strstr(message, "Y=");
  
  if (x_str != NULL && y_str != NULL) {
    int x = atoi(x_str + 2);  // Skip the "X=" part
    int y = atoi(y_str + 2);  // Skip the "Y=" part
    
    // Check if coordinates are valid
    if (x >= 0 && x <= MAX_X && y >= 0 && y <= MAX_Y) {
      last_x = x;
      last_y = y;
      new_coords_available = true;
      
      Serial.printf("Parsed coordinates: X=%d, Y=%d\n", last_x, last_y);
    } else {
      Serial.printf("Invalid coordinates: X=%d, Y=%d\n", x, y);
    }
  } else {
    Serial.println("Could not parse coordinates from message");
  }
}

bool get_weed_coordinates(int* x, int* y) {
  if (new_coords_available) {
    *x = last_x;
    *y = last_y;
    new_coords_available = false;
    return true;
  }
  return false;
}

void send_status_to_pi(const char* status) {
  char message[BUF_SIZE];
  snprintf(message, BUF_SIZE, "STATUS:%s\n", status);
  PiSerial.print(message);
  Serial.printf("Sent status to Pi: %s\n", status);
}
