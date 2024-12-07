import cv2
import yaml
import os
from pathlib import Path
import time
from ai_processor import AIProcessor

def initialize_camera(config):
    """Initialize USB camera with configuration"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    
    # Configure camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['resolution'][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['resolution'][1])
    cap.set(cv2.CAP_PROP_FPS, config['framerate'])
    
    # Additional settings to reduce lag
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer size
    return cap

def main():
    # Load configuration
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    
    camera_config = config['camera']
    
    # Initialize AI Processor
    ai_processor = AIProcessor()
    
    # Ensure image directory exists
    Path('images').mkdir(exist_ok=True)
    
    # FPS calculation variables
    fps_start_time = time.time()
    fps_counter = 0
    fps = 0
    
    # AI processing interval
    process_every_n_frames = 3  # Process every 3rd frame
    frame_counter = 0
    
    try:
        cap = initialize_camera(camera_config)
        print("Camera initialized successfully")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            frame_counter += 1
            current_time = time.time()
            
            # Calculate FPS
            fps_counter += 1
            if current_time - fps_start_time > 1:
                fps = fps_counter
                fps_counter = 0
                fps_start_time = current_time
            
            # Only process every nth frame
            if frame_counter % process_every_n_frames == 0:
                # Save frame for AI processing
                cv2.imwrite(camera_config['image_path'], frame)
                
                # Process with AI
                image_tensor = ai_processor.preprocess_image(camera_config['image_path'])
                if image_tensor is not None:
                    detections = ai_processor.predict(image_tensor)
                    
                    # Draw detections on frame
                    for det in detections:
                        box = det['box']
                        label = f"{det['class']} {det['score']:.2f}"
                        
                        # Different colors for different classes
                        color = (0, 255, 0) if det['class'] == 'crop' else (0, 0, 255)
                        
                        cv2.rectangle(frame, 
                                    (int(box[0]), int(box[1])), 
                                    (int(box[2]), int(box[3])), 
                                    color, 2)
                        
                        # Improved label visibility
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                        cv2.rectangle(frame,
                                    (int(box[0]), int(box[1] - label_size[1] - 10)),
                                    (int(box[0] + label_size[0]), int(box[1])),
                                    color, -1)
                        cv2.putText(frame, label,
                                  (int(box[0]), int(box[1] - 5)),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                  (255, 255, 255), 2)
            
            # Display FPS
            cv2.putText(frame, f"FPS: {fps}", (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show live preview
            cv2.imshow('Weed Detection Test', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to quit
                break
            elif key == ord('s'):  # Press 's' to save image
                image_path = f"images/capture_{int(time.time())}.jpg"
                cv2.imwrite(image_path, frame)
                print(f"Image saved to {image_path}")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()