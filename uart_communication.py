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
                # Set a larger timeout for better reliability
                self.ser = serial.Serial(self.device, self.baud_rate, timeout=self.timeout)
                
                # Flush any existing data in the buffer
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
                
                self.logger.info(f"UART initialized on {self.device} with baud rate {self.baud_rate}")
            except serial.SerialException as e:
                self.logger.error(f"Failed to initialize UART: {e}")
                self.ser = None
            
            self.lock = threading.Lock()
            
            # Used to track status of last command
            self.last_command_status = None
            
            # Wait a moment for ESP32 to be ready (optional)
            time.sleep(0.5)
            
            # Send an initial message to check if ESP32 is ready
            if self.ser is not None:
                self.send_data("SYSTEM_READY")
        
        def send_data(self, data):
            """Send data to ESP32 over UART"""
            if self.ser and self.ser.is_open:
                try:
                    with self.lock:
                        # Ensure the message ends with a newline as expected by ESP32
                        if not data.endswith('\n'):
                            message = data + '\n'
                        else:
                            message = data
                        
                        # First flush any incoming data to keep things clean
                        self.ser.reset_input_buffer()
                        
                        # Log what we're about to send in hex for debugging
                        self.logger.debug(f"Sending raw bytes: {message.encode('ascii').hex()}")
                        
                        # Then write the message
                        self.ser.write(message.encode('ascii'))
                        self.ser.flush()  # Make sure data is sent immediately
                        
                        self.logger.debug(f"Sent data: {data}")
                        
                        # Wait briefly and check if there's an immediate response
                        time.sleep(0.1)
                        if self.ser.in_waiting > 0:
                            self.logger.debug(f"Immediate response detected ({self.ser.in_waiting} bytes available)")
                        
                        return True
                except Exception as e:
                    self.logger.error(f"Error sending data: {e}")
                    return False
            else:
                self.logger.warning("UART not initialized or closed. Cannot send data.")
                return False
        
        def send_weed_coordinates(self, x, y):
            """Send weed coordinates to ESP32 in the expected format X=<number> Y=<number>"""
            # Verify coordinates are within valid range (0-100 as defined in ESP32 code)
            if x < 0 or x > 100 or y < 0 or y > 100:
                self.logger.warning(f"Invalid coordinates outside range 0-100: X={x}, Y={y}")
                # Clamp values to valid range
                x = max(0, min(100, x))
                y = max(0, min(100, y))
                
            # Format message in the format expected by the ESP32 code
            # This exactly matches the format in serial_comm.cpp: parse_weed_coordinates()
            message = f"X={x} Y={y}"
            
            # Log that we're sending coordinates
            self.logger.info(f"Attempting to send coordinates: {message}")
            
            # Check if the device is responsive first
            if self.ser and self.ser.is_open:
                bytes_waiting = self.ser.in_waiting
                if bytes_waiting > 0:
                    self.logger.info(f"Found {bytes_waiting} bytes waiting before sending. Clearing buffer.")
                    self.ser.reset_input_buffer()
            
            # Now send the actual message
            success = self.send_data(message)
            
            if success:
                self.logger.info(f"Sent weed coordinates: {message}")
                
                # Wait for ESP32 to process coordinates before continuing
                # This matches the ESP32's processing time
                time.sleep(0.2)
                
                # Try to get confirmation response
                response = self.process_esp32_response()
                if response:
                    self.logger.info(f"ESP32 acknowledged: {response}")
            
            return success
        
        def receive_data(self):
            """Receive data from ESP32 over UART with improved error handling"""
            if self.ser and self.ser.is_open:
                try:
                    with self.lock:
                        in_waiting = self.ser.in_waiting
                        if in_waiting > 0:
                            self.logger.debug(f"Data available to read: {in_waiting} bytes")
                            
                            # Read raw binary data
                            raw_data = self.ser.readline()
                            self.logger.debug(f"Read raw bytes: {raw_data.hex()}")
                            
                            try:
                                # Try to decode using ASCII first (ESP32 should be sending ASCII)
                                data = raw_data.decode('ascii', errors='replace').strip()
                                
                                # Check if we have invalid replacement characters
                                if '\ufffd' in data:
                                    # This indicates there were decoding errors
                                    self.logger.debug(f"Received data with encoding issues: {repr(raw_data)}")
                                    
                                    # Try to decode as Latin-1 which can handle any byte
                                    data = raw_data.decode('latin1').strip()
                                else:
                                    self.logger.debug(f"Received data: {data}")
                                    
                                return data
                            except Exception as e:
                                self.logger.error(f"Error decoding data: {e}, raw bytes: {repr(raw_data)}")
                                # Return a safe string representation of the raw data
                                return f"[Binary data: {raw_data.hex()}]"
                        else:
                            # No data waiting
                            return None
                except Exception as e:
                    self.logger.error(f"Error receiving data: {e}")
            return None
        
        def process_esp32_response(self):
            """Process response messages from ESP32"""
            # Try multiple times to get a response, as the ESP32 might not respond immediately
            for _ in range(3):  # Try up to 3 times
                response = self.receive_data()
                if response:
                    # Handle both text status responses and binary responses
                    if response.startswith("STATUS:"):
                        status = response[7:]  # Remove "STATUS:" prefix
                        self.last_command_status = status
                        self.logger.info(f"ESP32 Status: {status}")
                        return status
                    elif "WEEDING_STARTED" in response:
                        return "WEEDING_STARTED"
                    elif "WEEDING_COMPLETED" in response:
                        return "WEEDING_COMPLETED"
                    elif "MISSION_COMPLETED" in response:
                        return "MISSION_COMPLETED"
                    elif "LINE_CHANGE" in response:
                        return "LINE_CHANGE"
                    elif "WEED_REMOVED" in response:
                        return "WEED_REMOVED"
                    elif "REMOVE_FAILED" in response:
                        return "REMOVE_FAILED"
                    elif response.startswith("[Binary data:"):
                        # We received binary data that couldn't be decoded
                        self.logger.debug("Received binary data from ESP32")
                        # Check for common status messages in binary data
                        hex_data = response.split(": ")[1].strip("]")
                        if "57454544" in hex_data:  # "WEED" in ASCII hex
                            return "WEED_DETECTION"
                        return "BINARY_DATA_RECEIVED"
                
                # Wait a bit before trying again
                time.sleep(0.1)
            
            return None
        
        def close(self):
            """Close UART connection"""
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                    self.logger.debug("UART connection closed")
                except Exception as e:
                    self.logger.error(f"Error closing UART connection: {e}")
        
        def check_esp32_connection(self):
            """Check if ESP32 is responsive and communicating properly"""
            self.logger.info("Testing ESP32 connection...")
            
            # First, flush any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Send a test message
            test_message = "SYSTEM_TEST\n"
            self.logger.info(f"Sending test message: {test_message.strip()}")
            
            try:
                self.ser.write(test_message.encode('ascii'))
                self.ser.flush()
                
                # Wait for a response
                time.sleep(1.0)
                
                # Check if there's any response
                if self.ser.in_waiting > 0:
                    raw_data = self.ser.read(self.ser.in_waiting)
                    self.logger.info(f"Received response: {raw_data}")
                    return True
                else:
                    self.logger.warning("No response from ESP32 to test message")
                    return False
            except Exception as e:
                self.logger.error(f"Error testing ESP32 connection: {e}")
                return False