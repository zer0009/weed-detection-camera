import logging
import yaml
import platform
import time

# Only import serial if not on Windows
if platform.system() != 'Windows':
    import serial
    import threading

class MockUARTCommunication:
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.info("Initialized mock UART communication for testing")
    
    def send_weed_coordinates(self, x_coord, y_coord):
        """Mock sending weed coordinates"""
        self.logger.info(f"Mock: Sending weed coordinates X={x_coord}, Y={y_coord}")
        return True
    
    def send_data(self, command):
        """Mock sending command data"""
        self.logger.info(f"Mock: Sending command: {command}")
        return True
    
    def process_esp32_response(self):
        """Mock processing ESP32 response"""
        return "WEED_REMOVED"
    
    def close(self):
        """Mock closing connection"""
        self.logger.info("Mock: Closing UART connection")
        return True

# Use MockUARTCommunication as the default UARTCommunication class on Windows
if platform.system() == 'Windows':
    UARTCommunication = MockUARTCommunication
else:
    class UARTCommunication:
        def __init__(self, config_path='config.yaml'):
            self.logger = logging.getLogger(self.__class__.__name__)
            with open(config_path, 'r') as file:
                config = yaml.safe_load(file)
            
            uart_config = config['uart']
            self.device = uart_config['device']
            self.baud_rate = uart_config['baud_rate']
            self.timeout = uart_config['timeout']
            
            try:
                self.ser = serial.Serial(self.device, self.baud_rate, timeout=self.timeout)
                self.logger.info(f"UART initialized on {self.device} with baud rate {self.baud_rate}")
            except serial.SerialException as e:
                self.logger.error(f"Failed to initialize UART: {e}")
                self.ser = None
            
            self.lock = threading.Lock()
        
        def send_data(self, data):
            """Send data to ESP32 over UART"""
            if self.ser and self.ser.is_open:
                try:
                    with self.lock:
                        # Add newline for proper parsing on ESP32 side
                        message = data + '\n'
                        self.ser.write(message.encode())
                        self.logger.debug(f"Sent data: {data}")
                        return True
                except Exception as e:
                    self.logger.error(f"Error sending data: {e}")
                    return False
            else:
                self.logger.warning("UART not initialized or closed. Cannot send data.")
                return False
        
        def send_weed_coordinates(self, x, y):
            """Send weed coordinates to ESP32 in the expected format X=<number> Y=<number>"""
            # Verify coordinates are within valid range
            if x < 0 or y < 0:
                self.logger.warning(f"Invalid coordinates: X={x}, Y={y}")
                return False
                
            # Format message in the format expected by the ESP32
            message = f"X={x} Y={y}"
            success = self.send_data(message)
            
            if success:
                self.logger.info(f"Sent weed coordinates: {message}")
            
            # Wait for ESP32 to process before sending more commands
            time.sleep(0.1)
            return success
        
        def receive_data(self):
            """Receive data from ESP32 over UART"""
            if self.ser and self.ser.is_open:
                try:
                    with self.lock:
                        if self.ser.in_waiting > 0:
                            # Read raw binary data
                            raw_data = self.ser.readline()
                            
                            try:
                                # Try to decode using UTF-8 with error handling
                                data = raw_data.decode('utf-8', errors='replace').strip()
                                
                                # Check if we have invalid replacement characters
                                if '\ufffd' in data:
                                    # This indicates there were decoding errors
                                    self.logger.debug(f"Received data with encoding issues: {repr(raw_data)}")
                                else:
                                    self.logger.debug(f"Received data: {data}")
                                    
                                return data
                            except Exception as e:
                                self.logger.error(f"Error decoding data: {e}, raw bytes: {repr(raw_data)}")
                                # Return a safe string representation of the raw data
                                return f"[Binary data: {raw_data.hex()}]"
                except Exception as e:
                    self.logger.error(f"Error receiving data: {e}")
            return None
        
        def process_esp32_response(self):
            """Process response messages from ESP32"""
            response = self.receive_data()
            if response:
                # Handle both text status responses and binary responses
                if response.startswith("STATUS:"):
                    status = response[7:]  # Remove "STATUS:" prefix
                    self.logger.info(f"ESP32 Status: {status}")
                    return status
                elif response.startswith("[Binary data:"):
                    # We received binary data that couldn't be decoded
                    self.logger.debug("Received binary data from ESP32")
                    # Attempt to extract status from binary data if possible
                    if "WEED_REMOVED" in response:
                        return "WEED_REMOVED"
                    elif "REMOVE_FAILED" in response:
                        return "REMOVE_FAILED"
                    elif "TRACK_COMPLETED" in response:
                        return "TRACK_COMPLETED"
                    return "BINARY_DATA_RECEIVED"
            return None
        
        def close(self):
            """Close UART connection"""
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    self.logger.debug("UART connection closed")
                except Exception as e:
                    self.logger.error(f"Error closing UART connection: {e}")