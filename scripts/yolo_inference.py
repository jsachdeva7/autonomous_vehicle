"""Traffic light detection using YOLO.

This script loads a YOLO model and detects traffic light colors in a given image.

Dependencies:
    - ultralytics (YOLO)
    - torch
    - numpy
    - opencv-python (cv2)
"""

from ultralytics import YOLO
import torch
import numpy as np
import cv2

# Load YOLO Model
model = YOLO("../models/best.pt", verbose=False)

def warm_up_model() -> None:
    """Runs a dummy inference to warm up the YOLO model."""
    dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
    model(dummy_img, verbose=False)

def detect_traffic_light(image_bytes: bytes, width: int, height: int) -> str:
    """Detects traffic light colors in an image using YOLO.
    
    Args:
        image_bytes (bytes): The raw image bytes (BGRA).
        width (int): The width of the image.
        height (int): The height of the image.

    Returns:
        str: Comma-separated list of detected traffic light colors
            ('red', 'yellow', 'green') or 'none' if no lights detected
    """
    # Convert raw bytes to NumPy array
    img_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((height, width, 4)) # BGRA

    # Convert BGRA to BGR (YOLO expects BGR)
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

    if img_bgr is None:
        print("Error: Could not read image")
        return "none"   
    
    # Run YOLO inference
    results = model(img_bgr, verbose=False)
    
    detected_lights = []
    for result in results:
        if not hasattr(result, "boxes") or result.boxes is None:
            print("No bounding boxes detected.")
            continue
        
        for box in result.boxes:
            cls = int(box.cls[0]) # get class index

            # Retrieve class label
            if isinstance(model.names, dict):
                label = model.names.get(cls, "unknown")
            else:
                label = model.names[cls] if cls < len(model.names) else "unknown"

            # Store detected traffic light colors
            if label in ["red", "green", "yellow"]:
                detected_lights.append(label)

    # Join detected colors in comma-separated list
    result_str = ",".join(detected_lights) if detected_lights else "none"
    return result_str