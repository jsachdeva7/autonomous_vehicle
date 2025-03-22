from ultralytics import YOLO
import cv2

# Load the trained model
model = YOLO("best.pt") 

# Load video
video_path = "test_video_2.mp4" 
cap = cv2.VideoCapture(video_path)

# Get video properties
frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# Swap input dimensions to make output dimensions
output_width = frame_height
output_height = frame_width

# Define output video file
output_path = f"{video_path}_output.mp4"
fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # Codec for .mp4
out = cv2.VideoWriter(output_path, fourcc, fps, (output_width, output_height))

# Process the video frame-by-frame
while cap.isOpened():
    success, frame = cap.read()
    if not success:
        break  # Stop if the video ends

    rotated_frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Run YOLOv8 inference on the frame
    results = model(rotated_frame)

    # Visualize results on the frame
    annotated_frame = results[0].plot()

    # Write the frame to the output video
    out.write(annotated_frame)

    # Display the frame (optional)
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    if cv2.waitKey(1) & 0xFF == ord("q"):  # Press 'q' to quit
        break

# Release everything
cap.release()
out.release()
cv2.destroyAllWindows()

print(f"Inference complete. Output saved to {output_path}")