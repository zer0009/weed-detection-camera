import cv2
import yaml
import time
import logging
import os
from pathlib import Path
from ai_processor import AIProcessor

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/test_detection.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger('TestDetection')

def draw_detections(image, detections):
    """Draw detection boxes and labels on the image"""
    # Create a copy of the image for drawing
    result_image = image.copy()
    
    # Draw each detection
    for det in detections:
        box = det['box']
        confidence = det['confidence']
        class_name = det['class']
        label = f"{class_name} {confidence:.2f}"
        
        # Green for crop, Red for weed
        color = (0, 255, 0) if class_name == 'crop' else (0, 0, 255)
        
        # Ensure box coordinates are within image bounds and are integers
        x1 = max(0, int(box[0]))
        y1 = max(0, int(box[1]))
        x2 = min(image.shape[1], int(box[2]))
        y2 = min(image.shape[0], int(box[3]))
        
        # Skip if box is invalid
        if x2 <= x1 or y2 <= y1:
            continue
            
        # Draw box with thick line for better visibility
        cv2.rectangle(result_image, 
                     (x1, y1), 
                     (x2, y2), 
                     color, 4)
        
        # Draw label with background
        text_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        cv2.rectangle(result_image,
                     (x1, y1 - text_size[1] - 10),
                     (x1 + text_size[0], y1),
                     color, -1)
        cv2.putText(result_image, label,
                   (x1, y1 - 5),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                   (255, 255, 255), 2)
    
    # Add detection count
    count_text = f"Detections: {len(detections)}"
    cv2.putText(result_image, 
                count_text, 
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, (0, 0, 255), 3)
    
    return result_image

def test_detection():
    """Test weed detection on specific images"""
    try:
        logger.info("Initializing AI Processor...")
        ai_processor = AIProcessor()
        
        # Test images
        test_images = ['agri_0_39.jpeg', 'agri_0_126.jpeg']
        
        for image_name in test_images:
            image_path = Path('images') / image_name
            if not image_path.exists():
                logger.error(f"Image not found: {image_path}")
                continue
                
            logger.info(f"\nTesting image: {image_name}")
            
            # Read and process image
            image = cv2.imread(str(image_path))
            if image is None:
                logger.error(f"Failed to read image: {image_path}")
                continue
                
            # Get detections
            detections = ai_processor.predict(str(image_path))
            
            # Log detection results
            logger.info(f"Found {len(detections)} detections:")
            for det in detections:
                logger.info(f"Class: {det['class']}, Confidence: {det['confidence']:.2f}")
            
            # Draw and save results
            result_image = draw_detections(image.copy(), detections)
            output_path = Path('test_results') / f"result_{image_name}"
            output_path.parent.mkdir(exist_ok=True)
            cv2.imwrite(str(output_path), result_image)
            logger.info(f"Saved result to: {output_path}")
            
            # Display image
            cv2.imshow('Detection Results', result_image)
            cv2.waitKey(0)
        
        cv2.destroyAllWindows()
        logger.info("Test completed")
        
    except Exception as e:
        logger.error(f"Test failed: {e}")
        import traceback
        logger.error(traceback.format_exc())

if __name__ == "__main__":
    test_detection() 