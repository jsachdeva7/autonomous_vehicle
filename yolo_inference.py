from ultralytics import YOLO
import torch
import numpy as np
import cv2

model = YOLO("best.pt")

def detect_traffic_light(image_bytes, width, height):
    # Convert raw bytes to NumPy array
    img_array = np.frombuffer(image_bytes, dtype=np.uint8).reshape((height, width, 4)) # BGRA

    # Convert BGRA to BGR (YOLO needs BGR)
    img_bgr = cv2.cvtColor(img_array, cv2.COLOR_BGRA2BGR)

    if img_bgr is None:
        print("Error: Could not read image")
        return "none"   
    
    # run YOLO inference
    results = model(img_bgr)
    detected_lights = []

    for result in results:
        if not hasattr(result, "boxes"):
            continue

        for box in result.boxes:
            cls = int(box.cls[0]) # get class index
            label = model.names[cls] if isinstance(model.names, dict) else model.names[int(cls)]
            
            if label in ["red_light", "green_light", "yellow_light"]:
                detected_lights.append(label)

    return ",".join(detected_lights) if detected_lights else "none"