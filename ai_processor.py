import logging
import yaml
import torch
import cv2
import numpy as np
from pathlib import Path
import time

class AIProcessor:
    def __init__(self, config_path='config.yaml'):
        self.logger = logging.getLogger(self.__class__.__name__)
        with open(config_path, 'r') as file:
            config = yaml.safe_load(file)
        
        ai_config = config['ai']
        self.model_path = ai_config['model_path']
        self.confidence_threshold = ai_config['confidence_threshold']
        self.input_size = tuple(ai_config['input_size'])
        self.class_names = ai_config['class_names']
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        
        # Additional configurations for optimization
        self.enable_half = ai_config.get('enable_half', False)  # Use FP16 for faster inference if CUDA is available
        self.warmup_iterations = ai_config.get('warmup_iterations', 3)
        self.max_batch_size = ai_config.get('max_batch_size', 1)
        self.enable_debug = ai_config.get('enable_debug', False)
        
        # Initialize model
        self.model = self.load_model()
        self.warmup()
        
        self.logger.info(f"AI Processor initialized on {self.device}")
        
    def load_model(self):
        """Load and optimize YOLOv5 model for crop/weed detection"""
        try:
            self.logger.info(f"Loading custom YOLOv5 model from {self.model_path}")
            
            # Ensure model file exists
            if not Path(self.model_path).exists():
                raise FileNotFoundError(f"Custom model file not found: {self.model_path}")
            
            # Load custom model
            model = torch.hub.load('ultralytics/yolov5', 
                                   'custom',
                                   path=self.model_path,
                                   force_reload=True,
                                   trust_repo=True,
                                   device=self.device)
            
            # Set model parameters
            model.conf = self.confidence_threshold
            model.iou = 0.45
            model.classes = [0, 1]  # Only crop and weed classes
            model.max_det = 1000
            
            if self.enable_half and self.device.type != 'cpu':
                model.half()
            
            model.eval()
            
            # Verify class names
            if len(self.class_names) != 2:
                self.logger.warning("Expected 2 classes (crop, weed), but got a different number")
            
            return model
            
        except Exception as e:
            self.logger.error(f"Failed to load custom model: {e}")
            raise

    def warmup(self):
        """Perform model warmup"""
        if self.model is None:
            return
        
        self.logger.debug("Performing model warmup")
        try:
            # Create dummy input for warmup
            dummy_input = torch.zeros((1, 3, *self.input_size), device=self.device)
            if self.enable_half and self.device.type != 'cpu':
                dummy_input = dummy_input.half()
            
            # Perform warmup iterations
            for _ in range(self.warmup_iterations):
                self.model(dummy_input)
                
            self.logger.debug("Model warmup completed")
            
        except Exception as e:
            self.logger.error(f"Warmup failed: {e}")

    def preprocess_image(self, image_path):
        """Preprocess image for inference"""
        try:
            # Read image
            image = cv2.imread(image_path)
            if image is None:
                raise ValueError(f"Failed to read image: {image_path}")
            
            # Record original size for later
            self.original_size = image.shape[:2]
            
            # Basic image preprocessing
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Resize
            image = cv2.resize(image, self.input_size)
            
            # Normalize
            image = image.astype(np.float32) / 255.0
            
            # Convert to tensor
            image_tensor = torch.from_numpy(image).permute(2, 0, 1).unsqueeze(0)
            
            # Move to device and convert to half if needed
            image_tensor = image_tensor.to(self.device)
            if self.enable_half and self.device.type != 'cpu':
                image_tensor = image_tensor.half()
            
            return image_tensor
            
        except Exception as e:
            self.logger.error(f"Preprocessing failed: {e}")
            return None

    def predict(self, image_tensor):
        """Perform prediction specifically for crop/weed detection"""
        if image_tensor is None:
            return []
        
        try:
            start_time = time.time()
            
            with torch.no_grad():
                results = self.model(image_tensor)
            
            detections = []
            
            # Process predictions - results is now a tensor
            if isinstance(results, (list, tuple)):
                predictions = results[0] if len(results) > 0 else results
            else:
                predictions = results
            
            # Convert predictions to numpy for processing
            if torch.is_tensor(predictions):
                predictions = predictions.cpu().numpy()
            
            # Process each detection
            if len(predictions.shape) == 3:  # [batch, num_boxes, box_params]
                predictions = predictions[0]  # Take first batch
            
            for pred in predictions:
                if len(pred) >= 6:  # box coords (4) + confidence + class_id
                    x1, y1, x2, y2, conf, cls_id = pred[:6]
                    
                    if conf >= self.confidence_threshold:
                        class_id = int(cls_id)
                        if class_id < len(self.class_names):
                            class_name = self.class_names[class_id]
                            
                            detection = {
                                'box': [float(x1), float(y1), float(x2), float(y2)],
                                'score': float(conf),
                                'class': class_name,
                                'center': [float((x1 + x2) / 2), float((y1 + y2) / 2)]
                            }
                            detections.append(detection)
            
            inference_time = time.time() - start_time
            self.logger.debug(f"Inference completed in {inference_time:.3f} seconds")
            
            if self.enable_debug:
                self._save_debug_visualization(image_tensor, detections)
            
            return detections
            
        except Exception as e:
            self.logger.error(f"Prediction failed: {e}")
            import traceback
            self.logger.error(traceback.format_exc())
            return []

    def _save_debug_visualization(self, image_tensor, detections):
        """Save debug visualization of detections"""
        try:
            # Convert tensor to numpy image
            image = (image_tensor[0].permute(1, 2, 0).cpu().numpy() * 255).astype(np.uint8)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            
            # Draw detections
            for det in detections:
                box = det['box']
                label = f"{det['class']} {det['score']:.2f}"
                
                # Green for crop, Red for weed
                color = (0, 255, 0) if det['class'] == 'crop' else (0, 0, 255)
                
                # Draw box
                cv2.rectangle(image, 
                            (int(box[0]), int(box[1])), 
                            (int(box[2]), int(box[3])), 
                            color, 2)
                
                # Draw label with background
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(image,
                            (int(box[0]), int(box[1] - label_size[1] - 10)),
                            (int(box[0] + label_size[0]), int(box[1])),
                            color, -1)
                cv2.putText(image, label,
                           (int(box[0]), int(box[1] - 5)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                           (255, 255, 255), 2)
            
            # Save debug image
            debug_path = f"debug/detection_{int(time.time())}.jpg"
            Path('debug').mkdir(exist_ok=True)
            cv2.imwrite(debug_path, image)
            self.logger.debug(f"Debug visualization saved to {debug_path}")
            
        except Exception as e:
            self.logger.error(f"Failed to save debug visualization: {e}")