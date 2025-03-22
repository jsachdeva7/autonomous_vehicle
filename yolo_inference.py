from ultralytics import YOLO
import cv2

model = YOLO("best.pt")

def detect_traffic_light(image_path):
    img = cv2.imread(image_path)

    if img is None:
        print("Error: Could not read image")
        return ""   
    
    results = model(img)
    detected_lights = []

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0]) # get class index
            label = model.names[cls]
            if label in ["red_light", "green_light", "yellow_light"]:
                detected_lights.append(label)

    return ",".join(detected_lights) if detected_lights else "none"