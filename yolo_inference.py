from ultralytics import YOLO
import torch
import numpy as np
import cv2

model = YOLO("best.pt", verbose=False)

def warm_up_model():
    dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
    model(dummy_img, verbose=False)

def detect_traffic_light(image_bytes, width, height):
    # Convert raw bytes to NumPy array
    img_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((height, width, 4)) # BGRA

    # Convert BGRA to BGR (YOLO needs BGR)
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

    if img_bgr is None:
        print("Error: Could not read image")
        return "none"   
    
    # run YOLO inference
    results = model(img_bgr, verbose=False)
    
    detected_lights = []
    for result in results:
        if not hasattr(result, "boxes") or result.boxes is None:
            print("No bounding boxes detected.")
            continue

        for box in result.boxes:
            cls = int(box.cls[0]) # get class index

            if isinstance(model.names, dict):
                label = model.names.get(cls, "unknown")
            else:
                label = model.names[cls] if cls < len(model.names) else "unknown"

            if label in ["red", "green", "yellow"]:
                detected_lights.append(label)

    result_str = ",".join(detected_lights) if detected_lights else "none"
    return result_str