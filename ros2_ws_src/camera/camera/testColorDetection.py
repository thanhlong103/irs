import cv2
import numpy as np
from PIL import Image
import serial 
import time

def get_limits(color):
    """
    Calculate HSV limits for the given BGR color.
    :param color: list, BGR values of the target color
    :return: tuple, lower and upper HSV limits
    """
    c = np.uint8([[color]])  # BGR values
    hsv_c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsv_c[0][0][0]  # Get the hue value

    # Handle green hue limits
    lower_limit = np.array([45, 100, 100], dtype=np.uint8)
    upper_limit = np.array([75, 255, 255], dtype=np.uint8)

    return lower_limit, upper_limit

def send_velocity(vL, vR, ser):
    """
    Send left and right wheel velocities to Arduino.
    :param vL: float, velocity of the left wheel in m/s
    :param vR: float, velocity of the right wheel in m/s
    """
    try:
        response = ser.readline().decode().strip()
        print(f'Arduino Response: {response}')
        # Format the data as 'vL vR' followed by a newline character
        data = f"{vL:.2f} {vR:.2f}\n"
        ser.write(data.encode('utf-8'))  # Send data to Arduino
        #time.sleep(0.0)
        print(f"Sent: {data.strip()}")
    except Exception as e:
        print(f"Error sending data: {e}")

def main():
    # Camera setup
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    
    # ser = serial.Serial("COM5", 9600, timeout=.1)
    # time.sleep(2)

    while True:
        ret, frame = cap.read()
        if not ret:
            print('Failed to capture image from camera')
            break

        height, width, _ = frame.shape
        middle_x = width // 2

        # Convert the frame to HSV
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green color in BGR
        color = [87, 139, 46]  # Adjust if necessary
        lower_limit, upper_limit = get_limits(color=color)

        # Create a mask for the green color
        mask = cv2.inRange(hsvImage, lower_limit, upper_limit)

        # Get bounding box from the mask
        mask_ = Image.fromarray(mask)
        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            bbox_center_x = (x1 + x2) // 2
            distance_to_middle = bbox_center_x - middle_x

            # Draw bounding box and center line
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
            text = f"Distance to middle: {distance_to_middle}px"
            cv2.putText(frame, text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            frame = cv2.line(frame, (middle_x, 0), (middle_x, height), (255, 0, 0), 2)
            
        # send_velocity(0.1,0.1,ser)

        # Display the frame
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
