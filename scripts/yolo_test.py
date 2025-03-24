"""Traffic Light Detection Test with YOLO

This script tests the YOLO model trained on the LISA traffic light data set to detect for traffic lights in a video.
It processes each frame, annotates detected traffic lights, and saves the output video.

Dependencies:
- ultralytics (YOLO)
- opencv-python (cv2)
"""

from ultralytics import YOLO
import cv2

# Load the trained YOLO model
model = YOLO("../models/best.pt") 

# Load input video
video_path = "input_video.mp4" 
cap = cv2.VideoCapture(video_path)

# Get video properties
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# Define output video file
output_path = f"{video_path}_output.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4
out = cv2.VideoWriter(output_path, fourcc, fps, (frame_width, frame_height))

# Process the video frame-by-frame
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break  # Stop if the video ends / cannot be read

    # Run YOLOv8 inference on the frame
    results = model(frame)

    # Visualize results on the frame
    annotated_frame = results[0].plot()

    # Write the frame to the output video
    out.write(annotated_frame)

    # Display the frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):  # Press 'q' to quit
        break

# Release video resources
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Inference complete. Output saved to {output_path}")