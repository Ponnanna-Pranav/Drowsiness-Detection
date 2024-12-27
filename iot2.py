import serial
from scipy.spatial import distance as dist
from imutils.video import VideoStream
from imutils import face_utils
import numpy as np
import argparse
import imutils
import time
import dlib
import cv2

# Initialize serial connection
ser = serial.Serial('COM4', 9600)  # Change 'COM4' to the appropriate port and 9600 to your Arduino's baud rate

def send_command_to_arduino(command):
    ser.write(command.encode())

# Function to calculate eye aspect ratio
def eye_aspect_ratio(eye):
    A = dist.euclidean(eye[1], eye[5])
    B = dist.euclidean(eye[2], eye[4])
    C = dist.euclidean(eye[0], eye[3])
    eye = (A + B) / (2.0 * C)
    return eye

# Function to calculate lip distance
def lip_distance(shape):
    top_lip = shape[50:53]
    top_lip = np.concatenate((top_lip, shape[61:64]))
    low_lip = shape[56:59]
    low_lip = np.concatenate((low_lip, shape[65:68]))
    top_mean = np.mean(top_lip, axis=0)
    low_mean = np.mean(low_lip, axis=0)
    distance = abs(top_mean[1] - low_mean[1])
    return distance

# Construct the argument parser and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-w", "--webcam", type=int, default=0, help="index of webcam on system")
args = vars(ap.parse_args())

# Initialize dlib's face detector (HOG-based) and then create the facial landmark predictor
print("-> Loading the predictor and detector...")
detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor('shape_predictor_68_face_landmarks (1).dat')

# Define eye and yawning thresholds
EYE_AR_THRESH = 0.3
EYE_AR_CONSEC_FRAMES = 30
YAWN_THRESH = 20

# Initialize variables
alarm_status = False
COUNTER = 0
no_face_count = 0

# Start the video stream
print("-> Starting Video Stream")
vs = VideoStream(src=args["webcam"]).start()
time.sleep(1.0)

# Loop over frames from the video stream
while True:
    frame = vs.read()
    frame = imutils.resize(frame, width=450)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the grayscale frame
    rects = detector(gray, 0)

    # If no face detected, display "Driver not paying attention" after 5 seconds
    if len(rects) == 0:
        no_face_count += 1
        if no_face_count >= 5 * 30:  # Assuming 30 frames per second
            cv2.putText(frame, "Driver not paying attention", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    else:
        no_face_count = 0
        # Loop over the face detections
        for rect in rects:
            # Determine the facial landmarks for the face region
            shape = predictor(gray, rect)
            shape = face_utils.shape_to_np(shape)

            # Extract the eye regions and calculate eye aspect ratio
            leftEye = shape[36:42]
            rightEye = shape[42:48]
            leftEAR = eye_aspect_ratio(leftEye)
            rightEAR = eye_aspect_ratio(rightEye)
            eye = (leftEAR + rightEAR) / 2.0

            # Extract the mouth region and calculate lip distance
            distance = lip_distance(shape)

            # Check for drowsiness
            if eye < EYE_AR_THRESH:
                COUNTER += 1
                if COUNTER >= EYE_AR_CONSEC_FRAMES:
                    if not alarm_status:
                        alarm_status = True
                        send_command_to_arduino('D')  # Sending command for drowsiness alert
                    cv2.putText(frame, "DROWSINESS ALERT!", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                COUNTER = 0
                alarm_status = False
                send_command_to_arduino('N')  # Sending command to stop drowsiness alert

            # Check for yawning
            if distance > YAWN_THRESH:
                if not alarm_status:
                    alarm_status = True
                    send_command_to_arduino('Y')  # Sending command for yawn alert
                    cv2.putText(frame, "YAWN DETECTED!", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)  # Display "YAWN DETECTED" on the frame
            else:
                alarm_status = False

            # Draw contours around eyes and mouth
            leftEyeHull = cv2.convexHull(leftEye)
            rightEyeHull = cv2.convexHull(rightEye)
            cv2.drawContours(frame, [leftEyeHull], -1, (0, 255, 0), 1)
            cv2.drawContours(frame, [rightEyeHull], -1, (0, 255, 0), 1)
            
            # Define the mouth region points
            mouthPts = shape[48:68]
            mouthHull = cv2.convexHull(mouthPts)
            cv2.drawContours(frame, [mouthHull], -1, (0, 255, 0), 1)  # Draw contours around the mouth region

            cv2.putText(frame, "EYE: {:.2f}".format(eye), (300, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, "YAWN: {:.2f}".format(distance), (300, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Display the frame
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF

    # If the 'q' key is pressed, break from the loop
    if key == ord("q"):
        break

# Cleanup
cv2.destroyAllWindows()
vs.stop()
