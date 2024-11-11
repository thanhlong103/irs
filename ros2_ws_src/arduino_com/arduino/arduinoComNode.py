#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import time

class ArduinoCom(Node):
    def __init__(self, port, baudrate):
        super().__init__('arduino_com')  # Initialize the ROS2 Node
        self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=1)
        time.sleep(2)  # Allow time for the serial connection to initialize

        # Timer to call the 'read_and_publish' method at a fixed rate
        self.timer = self.create_timer(0.5, self.read_and_publish)  # 10 Hz
        self.get_logger().info('Arduino Communication Node Started')

    def send_velocity(self, linear, angular):
        # Create the message in the format "<linear,angular>"
        message = f"<{linear},{angular}>"
        self.arduino.write(message.encode('utf-8'))     

    def read_xyz(self):
        if self.arduino.in_waiting > 0:
            response = self.arduino.readline().decode('utf-8').strip()
            if response.startswith("<") and response.endswith(">"):
                # Remove the < and > characters and split by comma
                values = response[1:-1].split(',')
                if len(values) == 3:       
                    x = float(values[0])
                    y = float(values[1])
                    theta = float(values[2])
                    print(x, y, theta)
                    return x, y, theta
        return None,None,None

    def read_and_publish(self):
        self.send_velocity(0,0)

def main(args=None):
    rclpy.init(args=args)
    
    # Initialize the ArduinoReader class
    reader = ArduinoCom(port='/dev/ttyUSB2', baudrate=9600)

    try:
    #     # Spin the ROS2 node to keep it alive and processing callbacks
        rclpy.spin(reader)
    except:
        reader.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        # Shutdown ROS2 when done
        reader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
