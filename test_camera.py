import cv2
import mediapipe as mp
import time
import os

print("Initializing camera and MediaPipe...")
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# Open webcam (usually 0)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit(1)

print("Camera opened. Capturing a test frame...")
time.sleep(2) # Allow camera to warm up

success, frame = cap.read()
if success:
    print("Frame captured! Analyzing with MediaPipe...")
    # Convert BGR to RGB for MediaPipe
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = pose.process(rgb_frame)
    
    if results.pose_landmarks:
        print("SUCCESS: Person detected!")
        # Draw landmarks on the frame
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing.draw_landmarks(
            frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
    else:
        print("NOTE: No person detected in the frame. (Stand in front of the camera next time)")
    
    # Save test image
    save_path = "test_capture.jpg"
    cv2.imwrite(save_path, frame)
    print(f"Test image saved to {os.path.abspath(save_path)}")
else:
    print("Error: Failed to capture a frame from the camera.")

cap.release()
print("Camera closed.")
