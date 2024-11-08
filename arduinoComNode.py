import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import serial
import time

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        
        # Set up the serial connection to Arduino
        self.arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)
        time.sleep(2)  # Allow time for the serial connection to initialize

        # Subscriber to receive velocity commands
        self.subscription = self.create_subscription(
            Twist,
            '/velocities',
            self.send_velocity_to_arduino,
            10
        )

        # Publisher for XYZ data
        self.xyz_publisher = self.create_publisher(Float32MultiArray, '/xyz_data', 10)

        # Timer for continuously reading from Arduino
        self.timer = self.create_timer(0.1, self.read_xyz_from_arduino)

    def send_velocity_to_arduino(self, msg):
        # Extract linear and angular velocities from Twist message
        linear = msg.linear.x
        angular = msg.angular.z
        message = f"<{linear},{angular}>"
        
        # Send to Arduino
        self.arduino.write(message.encode('utf-8'))
        self.get_logger().info(f"Sent velocities to Arduino: {message}")

    def read_xyz_from_arduino(self):
        # Check if there is data available to read
        if self.arduino.in_waiting > 0:
            response = self.arduino.readline().decode('utf-8').strip()
            if response.startswith("<") and response.endswith(">"):
                # Parse the response to extract x, y, z values
                values = response[1:-1].split(',')
                if len(values) == 3:
                    try:
                        x = float(values[0])
                        y = float(values[1])
                        z = float(values[2])
                        
                        # Publish the XYZ data
                        xyz_msg = Float32MultiArray()
                        xyz_msg.data = [x, y, z]
                        self.xyz_publisher.publish(xyz_msg)
                        
                        self.get_logger().info(f"Received and published XYZ data: x={x}, y={y}, z={z}")
                    except ValueError:
                        self.get_logger().warning("Received invalid data from Arduino")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Close the serial connection and shutdown the node
        node.arduino.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
