import cv2
import yaml
import os
from pathlib import Path
import time
import logging
from ai_processor import AIProcessor
from uart_communication import UARTCommunication
import gc  # Add garbage collector
import subprocess  # For temperature monitoring
import math

def get_cpu_temperature():
    """Get the CPU temperature of the Raspberry Pi"""
    try:
        temp = subprocess.check_output(['vcgencmd', 'measure_temp']).decode()
        return float(temp.replace('temp=', '').replace("'C", ''))
    except:
        return 0.0

def setup_logging():
    """Configure logging settings"""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("weed_detection.log")
        ]
    )

def initialize_camera(config):
    """Initialize USB camera with configuration"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    
    # Print all available camera properties
    logger = logging.getLogger("MainControl")
    logger.info("Available camera properties:")
    for prop in range(0, 40):  # Check first 40 properties
        value = cap.get(prop)
        if value != -1:  # -1 means property not supported
            prop_name = None
            for name in dir(cv2):
                if name.startswith('CAP_PROP_') and getattr(cv2, name) == prop:
                    prop_name = name
                    break
            logger.info(f"Property {prop_name or prop}: {value}")
    
    # Try to set the camera to its widest possible view
    # First, try to disable any digital zoom
    cap.set(cv2.CAP_PROP_ZOOM, 0)
    
    # Try to set the widest possible view by adjusting various properties
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 1)  # Enable autofocus
    cap.set(cv2.CAP_PROP_FOCUS, 0)  # Set focus to infinity
    
    # Try to set the widest possible field of view
    cap.set(cv2.CAP_PROP_SETTINGS, 1)  # Some cameras use this for wide view
    
    # Configure basic camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['resolution'][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['resolution'][1])
    cap.set(cv2.CAP_PROP_FPS, config['framerate'])
    
    # Try to set exposure and other settings
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # Enable auto exposure
    cap.set(cv2.CAP_PROP_EXPOSURE, -6)
    cap.set(cv2.CAP_PROP_BRIGHTNESS, config['brightness'] / 100.0)
    cap.set(cv2.CAP_PROP_CONTRAST, config['contrast'] / 100.0)
    
    # Additional settings
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
    
    # Log the actual camera settings that were applied
    actual_width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    actual_zoom = cap.get(cv2.CAP_PROP_ZOOM)
    actual_focus = cap.get(cv2.CAP_PROP_FOCUS)
    
    logger.info("Actual camera settings after initialization:")
    logger.info(f"Resolution: {actual_width}x{actual_height}")
    logger.info(f"FPS: {actual_fps}")
    logger.info(f"Zoom: {actual_zoom}")
    logger.info(f"Focus: {actual_focus}")
    
    if actual_width != config['resolution'][0] or actual_height != config['resolution'][1]:
        logger.warning(f"Camera resolution mismatch. Requested: {config['resolution']}, Got: {actual_width}x{actual_height}")
    
    return cap

def main():
    setup_logging()
    logger = logging.getLogger("MainControl")
    
    # Load configuration
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    
    camera_config = config['camera']
    
    # Calculate and log the expected coverage area
    height_cm = 60  # Camera height in cm
    hfov_rad = math.radians(camera_config['hfov'])
    vfov_rad = math.radians(camera_config['vfov'])
    
    # Calculate coverage at ground level
    horizontal_coverage = 2 * height_cm * math.tan(hfov_rad / 2)
    vertical_coverage = 2 * height_cm * math.tan(vfov_rad / 2)
    
    logger.info(f"Camera setup at {height_cm}cm height")
    logger.info(f"Expected coverage at ground level:")
    logger.info(f"- Horizontal: {horizontal_coverage:.1f}cm")
    logger.info(f"- Vertical: {vertical_coverage:.1f}cm")
    
    # Temperature monitoring variables
    temp_check_interval = 5  # Check temperature every 5 seconds
    last_temp_check = time.time()
    temp_threshold = 75.0  # Temperature threshold in Celsius
    is_throttled = False
    
    # Initialize UART communication
    uart_comm = UARTCommunication()
    if uart_comm.send_data("SYSTEM_READY"):
        logger.info("Successfully connected to ESP32 over UART")
    else:
        logger.warning("Failed to connect to ESP32 over UART, continuing anyway...")
    
    # Initialize AI Processor
    ai_processor = AIProcessor()
    
    # Ensure directories exist
    Path('images').mkdir(exist_ok=True)
    Path('debug').mkdir(exist_ok=True)
    
    # FPS calculation variables
    fps_start_time = time.time()
    fps_counter = 0
    fps = 0
    
    # AI processing interval
    process_every_n_frames = config['ai'].get('process_every_n_frames', 3)
    frame_counter = 0
    
    # Keep track of the most recent detections
    last_detections = []
    
    try:
        cap = initialize_camera(camera_config)
        logger.info("Camera initialized successfully")
        logger.info(f"Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        logger.info(f"Target FPS: {cap.get(cv2.CAP_PROP_FPS)}")
        
        # Main processing loop
        while True:
            # Check temperature periodically
            current_time = time.time()
            if current_time - last_temp_check >= temp_check_interval:
                temp = get_cpu_temperature()
                logger.info(f"CPU Temperature: {temp:.1f}°C")
                
                # Throttle if temperature is too high
                if temp > temp_threshold and not is_throttled:
                    logger.warning(f"Temperature too high ({temp:.1f}°C), throttling...")
                    is_throttled = True
                    process_every_n_frames = max(process_every_n_frames, 5)  # Increase processing interval
                    # Reduce camera resolution temporarily
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)
                    cap.set(cv2.CAP_PROP_FPS, 10)
                elif temp < (temp_threshold - 5) and is_throttled:  # Resume normal operation when cooled
                    logger.info("Temperature normal, resuming normal operation")
                    is_throttled = False
                    process_every_n_frames = config['ai'].get('process_every_n_frames', 3)
                    # Restore camera settings
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_config['resolution'][0])
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_config['resolution'][1])
                    cap.set(cv2.CAP_PROP_FPS, camera_config['framerate'])
                
                last_temp_check = current_time
            
            # Check for ESP32 status messages
            esp32_status = uart_comm.process_esp32_response()
            if esp32_status:
                logger.info(f"ESP32 status: {esp32_status}")
                
                # Handle specific status messages
                if esp32_status == "TRACK_COMPLETED":
                    logger.info("Robot completed track and turned around")
            
            # Grab frame from camera
            ret, frame = cap.read()
            if not ret:
                logger.warning("Failed to grab frame")
                time.sleep(0.1)  # Add small delay to prevent CPU spinning
                continue
            
            frame_counter += 1
            
            # Calculate FPS
            fps_counter += 1
            if current_time - fps_start_time > 1:
                fps = fps_counter
                fps_counter = 0
                fps_start_time = current_time
                logger.debug(f"Current FPS: {fps}")
            
            # Only process every nth frame for AI detection
            if frame_counter % process_every_n_frames == 0:
                # Save frame for AI processing
                timestamp = int(time.time())
                image_path = f"images/capture_{timestamp}.jpg"
                cv2.imwrite(image_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                logger.debug(f"Captured image saved to {image_path}")
                
                # Process with AI
                detections = ai_processor.predict(image_path)
                
                # Update last_detections if we got results
                if detections:
                    last_detections = detections
                    
                    # Log detection results
                    weed_count = sum(1 for d in detections if d['class'] == 'weed')
                    crop_count = sum(1 for d in detections if d['class'] == 'crop')
                    
                    if weed_count > 0 or crop_count > 0:
                        logger.info(f"Detected {weed_count} weeds and {crop_count} crops")
                
                # Force garbage collection after processing
                gc.collect()
            
            # Create a copy of the frame for display
            display_frame = frame.copy()
            
            # Draw bounding boxes on the display frame if we have detections
            if last_detections:
                # Draw detections on the live frame
                for det in last_detections:
                    # Only show recent detections (within the last 3 seconds)
                    box = det['box']
                    label = f"{det['class']} {det['confidence']:.2f}"
                    
                    # Green for crop, Red for weed
                    color = (0, 255, 0) if det['class'] == 'crop' else (0, 0, 255)
                    
                    # Draw thicker box
                    cv2.rectangle(display_frame, 
                                (int(box[0]), int(box[1])), 
                                (int(box[2]), int(box[3])), 
                                color, 3)
                    
                    # Add semi-transparent overlay for visibility
                    overlay = display_frame.copy()
                    cv2.rectangle(overlay,
                                (int(box[0]), int(box[1])),
                                (int(box[2]), int(box[3])),
                                color, -1)
                    cv2.addWeighted(overlay, 0.2, display_frame, 0.8, 0, display_frame)
                    
                    # Draw label with background
                    label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(display_frame,
                                (int(box[0]), int(box[1] - label_size[1] - 10)),
                                (int(box[0] + label_size[0]), int(box[1])),
                                color, -1)
                    cv2.putText(display_frame, label,
                              (int(box[0]), int(box[1] - 5)),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                              (255, 255, 255), 2)
                    
                    # Add additional info for weeds
                    if det['class'] == 'weed':
                        info_text = f"Dist: {det.get('distance', 0):.1f}cm"
                        cv2.putText(display_frame, info_text,
                                  (int(box[0]), int(box[3] + 20)),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                  (255, 255, 255), 2)
            
            # Add status info to display frame
            cv2.putText(display_frame, f"FPS: {fps}", (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(display_frame, f"Temp: {get_cpu_temperature():.1f}C", (10, 60),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Add detection stats
            if last_detections:
                weed_count = sum(1 for d in last_detections if d['class'] == 'weed')
                crop_count = sum(1 for d in last_detections if d['class'] == 'crop')
                cv2.putText(display_frame, f"Weeds: {weed_count}, Crops: {crop_count}", (10, 90),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            if is_throttled:
                cv2.putText(display_frame, "THROTTLED", (10, 120),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Add overlay grid for better visualization
            h, w = display_frame.shape[:2]
            # Draw horizontal and vertical center lines
            cv2.line(display_frame, (w//2, 0), (w//2, h), (200, 200, 200), 1)
            cv2.line(display_frame, (0, h//2), (w, h//2), (200, 200, 200), 1)
            
            # Show live preview
            cv2.imshow('Weed Detection Live', display_frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to quit
                logger.info("Termination signal received. Exiting...")
                uart_comm.send_data("STOP")  # Send stop command to ESP32
                break
            elif key == ord('s'):  # Press 's' to save image
                image_path = f"images/manual_capture_{int(time.time())}.jpg"
                cv2.imwrite(image_path, display_frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                logger.info(f"Manual image saved to {image_path}")
            elif key == ord('e'):  # Press 'e' for emergency stop
                logger.info("Emergency stop activated!")
                uart_comm.send_data("STOP")
                break
            elif key == ord('c'):  # Press 'c' to clear detections
                last_detections = []
                logger.info("Cleared all detections from display")
            
            # Add small delay to prevent CPU spinning
            time.sleep(0.01)
    
    except Exception as e:
        logger.error(f"Error occurred: {e}")
        import traceback
        logger.error(traceback.format_exc())
    
    finally:
        # Clean up resources
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()
        
        # Close UART connection
        if 'uart_comm' in locals():
            uart_comm.close()
            
        logger.info("Resources released. Program terminated.")

if __name__ == "__main__":
    main() 