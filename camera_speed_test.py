import cv2
import yaml
import time
from pathlib import Path

def initialize_camera(config):
    """Initialize USB camera with configuration"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise IOError("Cannot open webcam")
    
    # Configure camera settings
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config['resolution'][0])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config['resolution'][1])
    cap.set(cv2.CAP_PROP_FPS, config['framerate'])
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer size
    
    return cap

def main():
    # Load configuration
    with open('config.yaml', 'r') as file:
        config = yaml.safe_load(file)
    
    camera_config = config['camera']
    
    # Ensure image directory exists
    Path('images').mkdir(exist_ok=True)
    
    # FPS calculation variables
    fps_start_time = time.time()
    fps_counter = 0
    fps = 0
    
    try:
        cap = initialize_camera(camera_config)
        print("Camera initialized successfully")
        print(f"Resolution: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
        print(f"Target FPS: {cap.get(cv2.CAP_PROP_FPS)}")
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Calculate FPS
            fps_counter += 1
            current_time = time.time()
            if current_time - fps_start_time > 1:
                fps = fps_counter
                fps_counter = 0
                fps_start_time = current_time
                print(f"Current FPS: {fps}")
            
            # Display FPS on frame
            cv2.putText(frame, f"FPS: {fps}", (10, 30),
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Show live preview
            cv2.imshow('Camera Speed Test', frame)
            
            # Handle key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):  # Press 'q' to quit
                break
            elif key == ord('s'):  # Press 's' to save image
                image_path = f"images/speed_test_{int(time.time())}.jpg"
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