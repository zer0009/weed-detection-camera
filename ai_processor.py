import logging
import yaml
import cv2
import numpy as np
from pathlib import Path
import time
import sys
import math
from ultralytics import YOLO
from uart_communication import UARTCommunication
import gc  # Add garbage collector

class AIProcessor:
    def __init__(self, config_path='config.yaml'):
        self.logger = logging.getLogger(self.__class__.__name__)
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        ai_config = config['ai']
        self.model_path = ai_config['model_path']
        self.confidence_threshold = ai_config['confidence_threshold']  # Use threshold from config
        self.input_size = tuple(ai_config['input_size'])
        self.class_names = ai_config['class_names']
        
        # Additional configurations for optimization
        self.enable_debug = ai_config.get('enable_debug', False)
        self.num_threads = ai_config.get('num_threads', 2)  # Reduced default threads
        
        # Camera calibration parameters
        self.image_width = config['camera']['resolution'][0]
        self.image_height = config['camera']['resolution'][1]
        self.hfov = config['camera']['hfov']  # Horizontal Field of View in degrees
        self.vfov = config['camera']['vfov']  # Vertical Field of View in degrees
        
        # Initialize model with optimized settings
        self.model = self.load_model()
        
        # Pre-allocate memory for detection results
        self.max_detections = 50  # Reduced from 100
        self.detection_buffer = np.zeros((self.max_detections, 6), dtype=np.float32)
        
        # Performance monitoring
        self.inference_times = []
        self.last_inference_time = 0
        
        self.logger.info("AI Processor initialized with NCNN model using Ultralytics YOLO")
        
        # Initialize UART Communication with ESP32
        self.uart_comm = UARTCommunication()
            
    def load_model(self):
        """Load NCNN model for crop/weed detection using Ultralytics YOLO"""
        try:
            self.logger.info(f"Loading NCNN model from {self.model_path}")
            
            if not Path(self.model_path).exists():
                raise FileNotFoundError(f"Model directory not found: {self.model_path}")
            
            # Load the NCNN model using YOLO class with optimized settings
            model = YOLO(self.model_path)
            
            # Set model parameters for optimization
            model.conf = self.confidence_threshold
            model.iou = 0.45  # Reduced IOU threshold for faster NMS
            
            return model
            
        except Exception as e:
            self.logger.error(f"Failed to load NCNN model: {e}")
            raise

    def predict(self, image_path):
        """YOLO-based inference with performance monitoring"""
        try:
            start_time = time.perf_counter()
            
            # Read original image to get its dimensions
            original_image = cv2.imread(image_path)
            if original_image is None:
                self.logger.error(f"Failed to read image: {image_path}")
                return []
            
            # Ensure we have the right dimensions for the camera calibration
            self.image_width = original_image.shape[1]
            self.image_height = original_image.shape[0]
            
            # Resize image to smaller size for faster processing
            resized_image = cv2.resize(original_image, self.input_size)
            
            # Run inference using YOLO with optimized settings
            results = self.model(resized_image, conf=self.confidence_threshold, max_det=self.max_detections, verbose=False)
            
            # Process all detections
            detections = []
            for result in results:
                boxes = result.boxes
                
                # Log detection count for debugging
                if len(boxes) > 0:
                    self.logger.debug(f"Model detected {len(boxes)} objects")
                
                for box in boxes:
                    # Get box coordinates (in pixel values)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
                    # Scale coordinates back to original image size
                    scale_x = original_image.shape[1] / self.input_size[0]
                    scale_y = original_image.shape[0] / self.input_size[1]
                    x1, x2 = x1 * scale_x, x2 * scale_x
                    y1, y2 = y1 * scale_y, y2 * scale_y
                    
                    # Get confidence score
                    confidence = float(box.conf[0].cpu().numpy())
                    
                    # Get class ID
                    class_id = int(box.cls[0].cpu().numpy())
                    
                    # Skip detections with invalid dimensions
                    if x2 <= x1 or y2 <= y1 or x2 - x1 < 5 or y2 - y1 < 5:  # Reduced minimum size
                        self.logger.debug(f"Skipping detection with invalid dimensions: {x1},{y1},{x2},{y2}")
                        continue
                    
                    # Create detection object
                    class_name = self.class_names[class_id] if class_id < len(self.class_names) else "unknown"
                    self.logger.debug(f"Detected {class_name} with confidence {confidence:.2f}")
                    
                    detection = {
                        'class': class_name,
                        'confidence': float(confidence),
                        'box': [float(x1), float(y1), float(x2), float(y2)]
                    }
                    detections.append(detection)
                
                    # Calculate offset angles and distance for all detections
                    offset_angle_x, offset_angle_y = self.calculate_offset_angles(detection['box'])
                    distance = self.estimate_distance(detection['box'])
                    detection['offset_angle_x'] = offset_angle_x
                    detection['offset_angle_y'] = offset_angle_y
                    detection['distance'] = distance
                    
                    # Only handle weed detections for robot action
                    if class_name == 'weed':
                        self.handle_weed_detection(detection)
            
            # Performance monitoring
            inference_time = time.perf_counter() - start_time
            self.inference_times.append(inference_time)
            if len(self.inference_times) > 50:  # Reduced from 100
                self.inference_times.pop(0)
            self.last_inference_time = inference_time
            
            # Force garbage collection
            gc.collect()
            
            # If debug is enabled, save visualization
            if self.enable_debug and detections:
                self._save_debug_visualization(original_image, detections)
                
            # Add total detection count in debug log
            self.logger.debug(f"Total detections after processing: {len(detections)}")
            weed_count = sum(1 for d in detections if d['class'] == 'weed')
            crop_count = sum(1 for d in detections if d['class'] == 'crop')
            self.logger.debug(f"Detection counts: Weeds={weed_count}, Crops={crop_count}")
            
            return detections
                
        except Exception as e:
            self.logger.error(f"Prediction failed: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return []

    def get_performance_stats(self):
        """Get performance statistics"""
        if not self.inference_times:
            return None
        
        return {
            'last_inference_time': self.last_inference_time * 1000,  # Convert to ms
            'average_inference_time': np.mean(self.inference_times) * 1000,
            'min_inference_time': np.min(self.inference_times) * 1000,
            'max_inference_time': np.max(self.inference_times) * 1000,
            'fps': 1.0 / np.mean(self.inference_times) if self.inference_times else 0
        }

    def calculate_offset_angles(self, bbox):
        """
        Calculate the horizontal and vertical offset angles of the detected weed relative to the camera's center.
        
        Args:
            bbox (list): Bounding box coordinates [x1, y1, x2, y2]
        
        Returns:
            tuple: (offset_angle_x, offset_angle_y)
        """
        try:
            x1, y1, x2, y2 = bbox
            weed_center_x = (x1 + x2) / 2
            weed_center_y = (y1 + y2) / 2

            # Calculate pixel offsets from image center
            pixel_offset_x = weed_center_x - (self.image_width / 2)
            pixel_offset_y = weed_center_y - (self.image_height / 2)

            # Convert pixel offsets to angles
            angle_per_pixel_x = self.hfov / self.image_width
            angle_per_pixel_y = self.vfov / self.image_height

            offset_angle_x = pixel_offset_x * angle_per_pixel_x
            offset_angle_y = pixel_offset_y * angle_per_pixel_y

            return offset_angle_x, offset_angle_y

        except Exception as e:
            self.logger.error(f"Error calculating offset angles: {e}")
            return 0.0, 0.0

    def estimate_distance(self, bbox):
        """
        Estimate the distance to the weed based on the bounding box height.
        Placeholder implementation. Replace with accurate distance estimation if available.
        
        Args:
            bbox (list): Bounding box coordinates [x1, y1, x2, y2]
        
        Returns:
            float: Estimated distance in cm.
        """
        try:
            x1, y1, x2, y2 = bbox
            box_height = y2 - y1
            # Assume average height of weed is 10 cm
            real_height_cm = 10.0
            focal_length = self.focal_length()
            distance = (real_height_cm * focal_length) / box_height
            return distance
        except Exception as e:
            self.logger.error(f"Distance estimation failed: {e}")
            return 0.0

    def focal_length(self):
        """
        Calculate the focal length based on camera calibration.
        Using similar triangles for estimation.
        
        Returns:
            float: Focal length in pixels.
        """
        try:
            # Using formula: focal_length = (image_width / 2) / tan(hfov / 2 in radians)
            focal_length = (self.image_width / 2) / math.tan(math.radians(self.hfov / 2))
            return focal_length
        except Exception as e:
            self.logger.error(f"Focal length calculation failed: {e}")
            return 1.0

    def handle_weed_detection(self, detection):
        """
        Handle actions based on weed detection, such as moving towards the weed and operating the delta arm.
        """
        try:
            # Extract detection parameters
            angle_x = detection['offset_angle_x']
            angle_y = detection['offset_angle_y']
            distance = detection['distance']
            
            # Use standard ASCII text instead of special characters for the degree symbol
            self.logger.info(f"Weed detected at angle_x: {angle_x:.2f} deg, angle_y: {angle_y:.2f} deg, distance: {distance:.2f} cm")
            
            # Convert angle and distance to ESP32 coordinate system
            # ESP32 code expects coordinates in range 0-100 for both X and Y
            
            # Map camera FOV angles to ESP32's expected 0-100 coordinate range
            # Map -hfov/2 to +hfov/2 -> 0 to 100
            # First normalize angle to range [-1, 1]
            normalized_x = angle_x / (self.hfov/2)  # Will be -1 to 1
            normalized_y = angle_y / (self.vfov/2)  # Will be -1 to 1
            
            # Then map to ESP32's 0-100 range
            x_coord = int(50 + (normalized_x * 50))
            y_coord = int(50 + (normalized_y * 50))
            
            # Ensure coordinates are within valid range (0-100 as defined in ESP32 config.h)
            x_coord = max(0, min(100, x_coord))
            y_coord = max(0, min(100, y_coord))
            
            self.logger.info(f"Sending weed coordinates to ESP32: X={x_coord}, Y={y_coord}")
            
            # Send weed coordinates to ESP32 over UART
            success = self.uart_comm.send_weed_coordinates(x_coord, y_coord)
            
            if success:
                self.logger.info(f"Successfully sent coordinates X={x_coord}, Y={y_coord} to ESP32")
                
                # Check for response from ESP32
                status = self.uart_comm.process_esp32_response()
                if status:
                    if status == "WEED_REMOVED":
                        self.logger.info("Weed successfully removed by delta arm")
                    elif status == "WEEDING_STARTED":
                        self.logger.info("ESP32 started weed removal process")
                    elif status == "WEEDING_COMPLETED":
                        self.logger.info("Weed removal process completed")
                    elif status == "REMOVE_FAILED":
                        self.logger.warning("Failed to remove weed, delta arm operation failed")
                    else:
                        self.logger.info(f"ESP32 responded with status: {status}")
                else:
                    self.logger.warning("No response received from ESP32 after sending coordinates")
                
                return True
            else:
                self.logger.error("Failed to send weed coordinates to ESP32")
                return False
            
        except Exception as e:
            self.logger.error(f"Handling weed detection failed: {e}")
            return False

    def send_command_to_robot(self, command):
        """Send command string to ESP32 via UART"""
        try:
            # Log the command being sent
            self.logger.info(f"Sending command to ESP32: {command}")
            
            # Send the command to the ESP32
            result = self.uart_comm.send_data(command)
            
            # Wait briefly for any response
            time.sleep(0.2)
            
            # Try to get a response from the ESP32
            response = self.uart_comm.process_esp32_response()
            if response:
                self.logger.info(f"Received response to command: {response}")
            else:
                self.logger.debug("No immediate response to command")
            
            return result
        except Exception as e:
            self.logger.error(f"Error sending command to robot: {e}")
            return False

    def _save_debug_visualization(self, image, detections):
        """Save debug visualization of detections"""
        try:
            # Clone image for visualization
            image_vis = image.copy()
            
            # Draw detections
            for det in detections:
                box = det['box']
                label = f"{det['class']} {det['confidence']:.2f}"
                
                # Green for crop, Red for weed
                color = (0, 255, 0) if det['class'] == 'crop' else (0, 0, 255)
                
                # Draw thicker box with rounded corners
                cv2.rectangle(image_vis, 
                            (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), 
                            color, 3)
                
                # Add semi-transparent overlay
                overlay = image_vis.copy()
                cv2.rectangle(overlay,
                            (int(box[0]), int(box[1])),
                            (int(box[2]), int(box[3])),
                            color, -1)
                cv2.addWeighted(overlay, 0.2, image_vis, 0.8, 0, image_vis)
                
                # Draw label with background
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(image_vis,
                            (int(box[0]), int(box[1] - label_size[1] - 10)),
                            (int(box[0] + label_size[0]), int(box[1])),
                            color, -1)
                cv2.putText(image_vis, label,
                           (int(box[0]), int(box[1] - 5)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                           (255, 255, 255), 2)
                
                # Add additional info for weeds
                if det['class'] == 'weed':
                    info_text = f"Dist: {det.get('distance', 0):.1f}cm"
                    cv2.putText(image_vis, info_text,
                              (int(box[0]), int(box[3] + 20)),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                              (255, 255, 255), 2)
            
            # Add FPS counter if available
            if self.inference_times:
                fps = 1.0 / np.mean(self.inference_times)
                cv2.putText(image_vis, f"FPS: {fps:.1f}",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                           (0, 255, 255), 2)
            
            # Save debug image
            debug_path = f"debug/detection_{int(time.time())}.jpg"
            Path('debug').mkdir(exist_ok=True)
            cv2.imwrite(debug_path, image_vis)
            self.logger.debug(f"Debug visualization saved to {debug_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to save debug visualization: {e}")