import cv2 

cap= cv2.VideoCapture(0)

while True:
    ret,frame= cap.read()
    
    cv2.imshow('img', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.imwrite('2.png',frame)
        cv2.destroyAllWindows
        break
        
cap.release()
