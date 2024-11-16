import cv2
from PIL import Image
import numpy as np

def color_pid_control(error, kp, ki, kd, iError, prevError, omega):
    iError = iError + error
    dError = error - prevError    
    
    P = kp * error
    I = ki * (iError)
    D = kd * (dError)
    
    PID = P + I + D
    
    prevError = error
    omega += PID
    
    return omega

def get_limits(color):
    c = np.uint8([[color]])  # BGR values
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    hue = hsvC[0][0][0]  # Get the hue value

    # Handle red hue wrap-around
    if hue >= 165:  # Upper limit for divided red hue
        lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
        upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    elif hue <= 15:  # Lower limit for divided red hue
        lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
        upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    else:
        lowerLimit = np.array([45, 100, 100], dtype=np.uint8)
        upperLimit = np.array([75, 255, 255], dtype=np.uint8)

    return lowerLimit, upperLimit
    
    
def main():
    color = [87,139,46]  # green in BGR colorspace
    
    omega = 0
    prevError = 0
    v = 0.5
    iError = 0
    
    while True:
        ret, frame = cap.read()
        
        height, width, _ = frame.shape
        
        middle_x = width // 2
       
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lowerLimit, upperLimit = get_limits(color=color)

        mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

        mask_ = Image.fromarray(mask)

        bbox = mask_.getbbox()

        if bbox is not None:
            x1, y1, x2, y2 = bbox
            
            bbox_center_x = (x1+x2) // 2
            
            distance_to_middle = bbox_center_x - middle_x
            
            omega = color_pid_control(distance_to_middle, 0.002, 0.0001, 0.004, iError, prevError, omega)
            
            print(omega)

            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
            
            text = f"Distance to middle: {distance_to_middle}px"
            
            cv2.putText(frame, text, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            
            frame = cv2.line(frame, (middle_x,0), (middle_x, height), (255,0,0), 2)

        cv2.imshow('frame', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
   
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    
    main()
    
    cap.release()

    cv2.destroyAllWindows()
    

