import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from PIL import Image

class ColorControlNode(Node):
    def __init__(self):
        super().__init__('color_control_node')
        
        # ROS 2 publisher for omega and v
        self.omega_publisher = self.create_publisher(Float32, '/omega', 10)
        self.v_publisher = self.create_publisher(Float32, '/v', 10)
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)
        
        # PID control parameters
        self.kp = 0.002
        self.ki = 0.0001
        self.kd = 0.004
        self.omega = 0
        self.prev_error = 0
        self.i_error = 0
        self.v = 0.5
                
        # Timer to call the callback at a fixed rate
        self.timer = self.create_timer(0.1, self.image_processing_callback)  # 10 Hz
        
        self.get_logger().info("Color_Control_Node Initialized")
    
    def color_pid_control(self, error):
        self.i_error += error
        d_error = error - self.prev_error
        
        P = self.kp * error
        I = self.ki * self.i_error
        D = self.kd * d_error
        
        pid = P + I + D
        
        self.prev_error = error
        self.omega += pid
        
        return self.omega
    
    def get_limits(self, color):
        c = np.uint8([[color]])  # BGR values
        hsv_c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

        hue = hsv_c[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
        if hue >= 165:  # Upper limit for red hue
            lower_limit = np.array([hue - 10, 100, 100], dtype=np.uint8)
            upper_limit = np.array([180, 255, 255], dtype=np.uint8)
        elif hue <= 15:  # Lower limit for red hue
            lower_limit = np.array([0, 100, 100], dtype=np.uint8)
            upper_limit = np.array([hue + 10, 255, 255], dtype=np.uint8)
        else:
            lower_limit = np.array([45, 100, 100], dtype=np.uint8)
            upper_limit = np.array([75, 255, 255], dtype=np.uint8)

        return lower_limit, upper_limit
    
    def image_processing_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture image from camera')
            return
        
        height, width, _ = frame.shape
        middle_x = width // 2
        
        hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        color = [87, 139, 46]  # Green in BGR
        lower_limit, upper_limit = self.get_limits(color=color)

        mask = cv2.inRange(hsvImage, lower_limit, upper_limit)

        mask_ = Image.fromarray(mask)

        bbox = mask_.getbbox()
        
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            
            bbox_center_x = (x1+x2) // 2
            
            distance_to_middle = bbox_center_x - middle_x
            
            omega = self.color_pid_control(distance_to_middle)
            
            print(omega)

            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
            
            text = f"Distance to middle: {distance_to_middle}px"
            
            cv2.putText(frame, text, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
            
            frame = cv2.line(frame, (middle_x,0), (middle_x, height), (255,0,0), 2)
            
            msg = Float32()
            msg.data = omega
            self.omega_publisher.publish(msg)
            
            msg = Float32()
            msg.data = omega
            self.v_publisher.publish(msg)

        # Show the image in a window (optional, can be removed in headless systems)
        cv2.imshow('frame', frame)
        cv2.waitKey(1)
    
    def destroy(self):
        self.cap.release()
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    
    color_control_node = ColorControlNode()

    try:
        rclpy.spin(color_control_node)
    except KeyboardInterrupt:
        pass
    
    color_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

