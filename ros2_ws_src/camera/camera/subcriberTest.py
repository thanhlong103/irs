import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class ColorControlSubscriberNode(Node):
    def __init__(self):
        super().__init__('color_control_subscriber_node')
        
        # Subscriber for 'omega' topic
        self.omega_subscription = self.create_subscription(
            Float32,
            '/omega',
            self.omega_callback,
            10
        )
        
        # Subscriber for 'v' topic
        self.v_subscription = self.create_subscription(
            Float32,
            '/v',
            self.v_callback,
            10  # QoS profile
        )
        
        self.get_logger().info("Subscriber Node Initialized")

    def omega_callback(self, msg):
        # Callback for omega topic
        print(msg)
        self.get_logger().info(f"Received omega: {msg.data}")

    def v_callback(self, msg):
        # Callback for v topic
        self.get_logger().info(f"Received v: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    
    # Create the subscriber node
    color_control_subscriber_node = ColorControlSubscriberNode()

    try:
        rclpy.spin(color_control_subscriber_node)  # Spin the node to keep it alive and process incoming messages
    except KeyboardInterrupt:
        pass
    finally:
        color_control_subscriber_node.destroy_node()  # Cleanup node
        rclpy.shutdown()

if __name__ == '__main__':
    main()

