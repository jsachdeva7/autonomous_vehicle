import torch
from ultralytics import YOLO
import cv2
import numpy as np
import socket

model = YOLO("best.pt")

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = ('127.0.0.1', 5005)

def detect_traffic_light(img):
    results = model(img)
    detected_lights = []

    for result in results:
        for box in result.boxes:
            cls = int(box.cls[0]) # get class index
            if model.names[cls] in ["red_light", "green_light", "yellow_light"]:
                detected_lights.append(model.names[cls])

    return detected_lights

cap = cv2.VideoCapture(0)
while True:
    ret, frame = cap.read()
    if not ret:
        continue

    lights = detect_traffic_light(frame)

    if lights:
        message = ",".join(lights)
        sock.sendto(message.encode(), server_address)

cap.release()
sock.close()