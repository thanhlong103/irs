import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, TransformStamped
import tf2_ros
import serial
import time
import cv2
from std_msgs.msg import Float32
from PIL import Image
import math
import heapq

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Parameters
        # self.path = np.load('path.npy')  # Load saved path
        self.path = None
        self.grid_map = np.load('grid_map.npy')  # Load grid map
        self.grid_size = 0.02  # Grid resolution in meters per cell
        self.pose = np.array([0.6, 1.3, 0])  # Start pose based on the first path point
        self.prev_scan = None  # Store the previous scan
        self.path_index = 0
        self.goal_tolerance = 0.1  # Tolerance for reaching a path point
        self.ahead_distance = 0.2  # Distance to look ahead on the path
        self.x_min = -1.034
        self.y_min = -0.627
        self.goalx = 1.5
        self.goaly = -0.1
        # TF broadcaster for map -> odom
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Subscribers and Publishers
        self.scan_subscriber = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.grid_map_publisher = self.create_publisher(OccupancyGrid, '/grid_map', 10)
        self.path_publisher = self.create_publisher(Marker, '/path_marker', 10)
        self.robot_marker_publisher = self.create_publisher(Marker, '/robot_marker', 10)
        self.ahead_point_publisher = self.create_publisher(Marker, '/robot_ahead_point_marker', 10)

        # Timer to periodically publish the map and path
        self.create_timer(1.0, self.publish_grid_map)
        self.create_timer(1.0, self.publish_path)
        self.create_timer(1.0, self.publish_robot_marker)
        self.create_timer(1.0, self.publish_ahead_point_marker)
        self.create_timer(0.1, self.publish_transform)  # Publish TF at 10 Hz
        
        self.arduino_port = '/dev/ttyUSB1'  # Replace with the correct port for your Arduino
        self.baud_rate = 9600
        self.serial_timeout = 1
        
        # Camera setup
        self.cap = cv2.VideoCapture(0)

        self.arduino_port = '/dev/ttyUSB1'  # Replace with the correct port for your Arduino
        self.baud_rate = 9600
        self.serial_timeout = 1

        # PID control parameters
        self.kp = 0.0015
        self.ki = 0.000002
        self.kd = 0.00006
        self.omega = 0
        self.prev_error = 0
        self.i_error = 0
        self.v = 0.05
        
        self.d = 0.172

        self.ser = serial.Serial(self.arduino_port, self.baud_rate, timeout=self.serial_timeout)
        time.sleep(2)
        self.get_logger().info(f"Connected to Arduino on {self.arduino_port}")

        # Add the publisher for the closest point marker
        self.closest_point_publisher = self.create_publisher(Marker, '/closest_point_marker', 10)
        self.create_timer(1.0, self.publish_closest_point_marker)

        self.get_logger().info('Path Follower Node has started.')

    def publish_grid_map(self):
        """
        Publish the grid map as an OccupancyGrid message.
        """
        occupancy_grid = OccupancyGrid()

        # Header
        occupancy_grid.header.frame_id = 'map'
        occupancy_grid.header.stamp = self.get_clock().now().to_msg()

        # Map metadata
        occupancy_grid.info.resolution = self.grid_size  # Map resolution in meters per cell
        occupancy_grid.info.width = self.grid_map.shape[1]
        occupancy_grid.info.height = self.grid_map.shape[0]
        occupancy_grid.info.origin = Pose()
        occupancy_grid.info.origin.position.x = 0.0
        occupancy_grid.info.origin.position.y = 0.0
        occupancy_grid.info.origin.position.z = 0.0

        # Flatten and normalize the grid map to match OccupancyGrid format
        grid_data = (self.grid_map.flatten() * 100).astype(np.int8)  # Scale [0, 1] to [0, 100]
        grid_data = np.clip(grid_data, 0, 100).astype(np.int8)  # Clamp to [0, 100]
        occupancy_grid.data = [int(value) if 0 <= value <= 100 else -1 for value in grid_data]  # Use -1 for unknown cells

        self.grid_map_publisher.publish(occupancy_grid)
        
    def color_pid_control(self, error):
        error = -error / 10
        self.i_error += error
        d_error = error - self.prev_error

        P = self.kp * error
        I = self.ki * self.i_error
        D = self.kd * d_error

        self.omega = P + I + D

        self.prev_error = error

        vL = self.v - (self.omega*self.d)/2
        vR = self.v + (self.omega*self.d)/2

        self.send_velocity(vL, vR)
        
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

    def send_velocity(self, vL, vR):
        """
        Send left and right wheel velocities to Arduino.
        :param vL: float, velocity of the left wheel in m/s
        :param vR: float, velocity of the right wheel in m/s
        """
        try:
            # Format the data as 'vL vR' followed by a newline character
            data = f"{vL:.2f} {vR:.2f}\n"
            self.ser.write(data.encode('utf-8'))  # Send data to Arduino
            #time.sleep(0.0)
            print(f"Sent: {data.strip()}")
        except Exception as e:
            print(f"Error sending data: {e}")

    def image_processing(self):
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

            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

            text = f"Distance to middle: {distance_to_middle}px"

            cv2.putText(frame, text, (10,50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

            frame = cv2.line(frame, (middle_x,0), (middle_x, height), (255,0,0), 2)
        else:
            self.send_velocity(self.v,0.0)
        # Show the image in a window (optional, can be removed in headless systems)
        # cv2.imshow('frame', frame)
        # cv2.waitKey(1)


    def publish_path(self):
        """
        Publish the path as a Marker message.
        """
        
        if self.path == None:
            return
        
        path_marker = Marker()

        # Header
        path_marker.header.frame_id = 'map'
        path_marker.header.stamp = self.get_clock().now().to_msg()

        # Marker properties
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.02  # Line width in meters

        # Path color (red)
        path_marker.color.r = 1.0
        path_marker.color.g = 0.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        # Convert path points to geometry_msgs/Point
        for point in self.path:
            path_point = Point()
            path_point.x = point[0] * self.grid_size  # Scale grid to world coordinates
            path_point.y = point[1] * self.grid_size
            path_point.z = 0.0
            path_marker.points.append(path_point)

        self.path_publisher.publish(path_marker)

    def publish_robot_marker(self):
        # Create a Marker message for the robot position
        robot_marker = Marker()
        robot_marker.header.frame_id = 'map'
        robot_marker.header.stamp = self.get_clock().now().to_msg()
        robot_marker.ns = 'robot'
        robot_marker.id = 1
        robot_marker.type = Marker.ARROW
        robot_marker.action = Marker.ADD
        robot_marker.color.a = 1.0  # Fully opaque
        robot_marker.color.g = 1.0  # Green
        robot_marker.scale.x = 0.3  # Length of the arrow
        robot_marker.scale.y = 0.05  # Width of the arrow
        robot_marker.scale.z = 0.05  # Height of the arrow
        robot_marker.pose.position.x = self.pose[0]
        robot_marker.pose.position.y = self.pose[1]
        robot_marker.pose.position.z = 0.1  # Slightly above the ground

        # Set the orientation of the arrow to match the robot's theta
        quaternion = R.from_euler('z', self.pose[2]).as_quat()
        robot_marker.pose.orientation.x = quaternion[0]
        robot_marker.pose.orientation.y = quaternion[1]
        robot_marker.pose.orientation.z = quaternion[2]
        robot_marker.pose.orientation.w = quaternion[3]



        # Publish the robot marker
        self.robot_marker_publisher.publish(robot_marker)

    def publish_ahead_point_marker(self):
        # Get the ahead point
        ahead_point = self.get_ahead_point(self.pose)

        # Create a Marker message for the ahead point
        ahead_point_marker = Marker()
        ahead_point_marker.header.frame_id = 'map'
        ahead_point_marker.header.stamp = self.get_clock().now().to_msg()
        ahead_point_marker.ns = 'ahead_point'
        ahead_point_marker.id = 2
        ahead_point_marker.type = Marker.SPHERE
        ahead_point_marker.action = Marker.ADD
        ahead_point_marker.color.a = 1.0  # Fully opaque
        ahead_point_marker.color.b = 1.0  # Blue
        ahead_point_marker.scale.x = 0.1  # Diameter of the sphere
        ahead_point_marker.scale.y = 0.1
        ahead_point_marker.scale.z = 0.1
        ahead_point_marker.pose.position.x = ahead_point[0]
        ahead_point_marker.pose.position.y = ahead_point[1]
        ahead_point_marker.pose.position.z = 0.1  # Slightly above the ground

        # Publish the ahead point marker
        self.ahead_point_publisher.publish(ahead_point_marker)

    def publish_closest_point_marker(self):
        # Get the ahead point
        ahead_point = self.get_ahead_point(self.pose)

        # Find the closest path point to the ahead point
        closest_point, _ = self.find_closest_path_point(ahead_point)

        # Create a Marker message for the closest point
        closest_point_marker = Marker()
        closest_point_marker.header.frame_id = 'map'
        closest_point_marker.header.stamp = self.get_clock().now().to_msg()
        closest_point_marker.ns = 'closest_point'
        closest_point_marker.id = 3
        closest_point_marker.type = Marker.SPHERE
        closest_point_marker.action = Marker.ADD
        closest_point_marker.color.a = 1.0  # Fully opaque
        closest_point_marker.color.r = 1.0  # Red
        closest_point_marker.scale.x = 0.1  # Diameter of the sphere
        closest_point_marker.scale.y = 0.1
        closest_point_marker.scale.z = 0.1
        closest_point_marker.pose.position.x = closest_point[0]
        closest_point_marker.pose.position.y = closest_point[1]
        closest_point_marker.pose.position.z = 0.1  # Slightly above the ground

        # Publish the closest point marker
        self.closest_point_publisher.publish(closest_point_marker)

    def publish_transform(self):
        """
        Publish the map -> odom transformation.
        """
        transform = TransformStamped()

        # Header
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'odom'

        # Set the transformation based on the robot's pose
        transform.transform.translation.x = 1.0 # self.pose[0]
        transform.transform.translation.y = 1.0 # self.pose[1]
        transform.transform.translation.z = 0.0

        # Convert yaw to quaternion
        quaternion = R.from_euler('z', self.pose[2]).as_quat()
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        self.tf_broadcaster.sendTransform(transform)

    def lidar_callback(self, msg):
        response = self.ser.readline().decode().strip()
        print(f'Arduino Response: {response}')
        
        # Convert LaserScan data to Cartesian coordinates
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        ranges = np.nan_to_num(ranges, nan=0.0, posinf=0.0, neginf=0.0)
        points = np.array([
            ranges * np.cos(angles),
            ranges * np.sin(angles)
        ]).T

        # Remove invalid points (e.g., range = 0)
        points = points[np.linalg.norm(points, axis=1) > 0]

        if self.prev_scan is not None:
            # Apply ICP to calculate the relative transformation
            delta_pose = self.icp(self.prev_scan, points)
            self.pose = self.update_pose(self.pose, delta_pose)
            self.get_logger().info(f'Updated Pose: {self.pose}')

            # Publish the robot marker to visualize the position
            self.publish_robot_marker()
            
            if response == 1:
                # Define start and goal positions in grid coordinates
                self.send_velocity(0.0, 0.0)
                time.sleep(2)
                
                start = self.to_grid_indices(self.pose[0], self.pose[1], self.x_min, self.y_min, self.grid_size)
                goal = self.to_grid_indices(self.goalx, self.goaly, self.x_min, self.y_min, self.grid_size)
                
                self.path = self.astar(self.grid_map, start, goal)

                # Control the robot to follow the path
                self.control_robot()
            else:
                self.image_processing()

        self.prev_scan = points
        
    # Helper function to convert coordinates to grid indices
    def to_grid_indices(self, x, y, x_min, y_min, resolution):
        grid_x = int((x - x_min) / resolution)
        grid_y = int((y - y_min) / resolution)
        return grid_x, grid_y
        
    # Define A* algorithm for pathfinding
    def astar(self, grid_map, start, goal):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        open_set = []
        heapq.heappush(open_set, (0, start))
        g_costs = {start: 0}
        f_costs = {start: self.heuristic(start, goal)}
        came_from = {}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                if not (0 <= neighbor[0] < grid_map.shape[1] and 0 <= neighbor[1] < grid_map.shape[0]):
                    continue
                if grid_map[neighbor[1], neighbor[0]] == 1:
                    continue

                tentative_g_cost = g_costs[current] + (1.414 if direction[0] != 0 and direction[1] != 0 else 1)

                if neighbor not in g_costs or tentative_g_cost < g_costs[neighbor]:
                    came_from[neighbor] = current
                    g_costs[neighbor] = tentative_g_cost
                    f_costs[neighbor] = tentative_g_cost + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_costs[neighbor], neighbor))

        return None

    def heuristic(self, a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def icp(self, source, target, max_iterations=25, tolerance=1e-3):
        """
        Perform ICP between source and target point clouds.
        """
        source_copy = source.copy()
        transformation = np.eye(3)

        for i in range(max_iterations):
            # Find the nearest neighbors between the source and target
            nbrs = NearestNeighbors(n_neighbors=1, algorithm='kd_tree').fit(target)
            distances, indices = nbrs.kneighbors(source_copy)

            # Compute the centroids of the matched points
            target_matched = target[indices[:, 0]]
            source_centroid = np.mean(source_copy, axis=0)
            target_centroid = np.mean(target_matched, axis=0)

            # Subtract centroids to align the points
            source_centered = source_copy - source_centroid
            target_centered = target_matched - target_centroid

            # Compute the optimal rotation using SVD
            H = np.dot(source_centered.T, target_centered)
            U, _, Vt = np.linalg.svd(H)
            R_opt = np.dot(Vt.T, U.T)

            # Ensure R_opt is a proper rotation matrix
            if np.linalg.det(R_opt) < 0:
                Vt[1, :] *= -1
                R_opt = np.dot(Vt.T, U.T)

            # Compute the translation
            t_opt = target_centroid - np.dot(source_centroid, R_opt)

            # Update the transformation matrix
            current_transform = np.eye(3)
            current_transform[:2, :2] = R_opt
            current_transform[:2, 2] = t_opt
            transformation = np.dot(current_transform, transformation)

            # Apply the transformation to the source points
            source_copy = (np.dot(R_opt, source_copy.T).T + t_opt)

            # Check for convergence
            mean_error = np.mean(distances)
            if mean_error < tolerance:
                break

        # Extract translation and rotation (angle) from the final transformation
        dx = transformation[0, 2]
        dy = transformation[1, 2]
        dtheta = np.arctan2(transformation[1, 0], transformation[0, 0])

        return np.array([dx, dy, -dtheta])

    def update_pose(self, pose, delta_pose):
        """
        Update the robot's pose based on the delta pose.
        """
        dx, dy, dtheta = delta_pose
        theta = pose[2]

        # Update the pose with respect to the robot's current orientation
        x_new = pose[0] + dx * np.cos(theta) - dy * np.sin(theta)
        y_new = pose[1] + dx * np.sin(theta) + dy * np.cos(theta)
        theta_new = (pose[2] + dtheta + np.pi) % (2 * np.pi) - np.pi  # update and limit angle to [-pi, pi]

        return np.array([x_new, y_new, theta_new])

    def get_ahead_point(self,pose):
        theta = pose[2]
        x_ap = pose[0] + self.ahead_distance * np.cos(theta)
        y_ap = pose[1] + self.ahead_distance * np.sin(theta)
        return np.array([x_ap, y_ap])

    def find_closest_path_point(self, ahead_point):
        """
        Find the point on the path closest to the ahead point.
        """
        distances = np.linalg.norm(self.path * self.grid_size - ahead_point, axis=1)
        closest_index = np.argmin(distances)
        closest_point = self.path[closest_index] * self.grid_size  # Convert grid to meters
        return closest_point, closest_index

    def control_robot(self):
        """
        Control the robot to follow the path using a simple proportional controller.
        """

        # Check if the robot has reached the goal
        goal_point = self.path[-1] * self.grid_size  # Last point in the path
        distance_to_goal = np.linalg.norm(self.pose[:2] - goal_point)
        if distance_to_goal < 0.2:
            self.get_logger().info('Goal reached.')
            self.stop_robot()
            return

        # Get the closest path point to the ahead point
        ahead_point = self.get_ahead_point(self.pose)
        closest_point, self.path_index = self.find_closest_path_point(ahead_point)

        # Compute the distance between the closest point and the line connecting robot pose and ahead point
        robot_to_ahead_vector = ahead_point - self.pose[:2]
        robot_to_closest_vector = closest_point - self.pose[:2]
        cross_product = np.cross(robot_to_ahead_vector, robot_to_closest_vector) # |v1|*|v2|*sin(<v1,v2>)
        current_error = cross_product / np.linalg.norm(robot_to_ahead_vector) # |v1|*sin()
        # Note: due to cross prodcut, the distance is signed (+ve if on left, -ve if on right)

        Kp = 2.5
        Ki = 0.05
        Kd = 0.7

        # Initialize PID errors
        if not hasattr(self, 'error_sum'):
            self.error_sum = 0.0
            self.prev_error = 0.0

        # Compute PID errors
        self.error_sum += current_error * 0.1  # Integral term
        derivative = (current_error - self.prev_error) / 0.1  # Derivative term

        # Compute control commands
        angular_speed = Kp * current_error + Ki * self.error_sum + Kd * derivative

        # Update previous errors
        self.prev_error = current_error
        
        linear_speed = 0.08
        
        vL = linear_speed - (angular_speed*self.d)/2
        vR = linear_speed + (angular_speed*self.d)/2

        # Publish the Twist message
        # twist_msg = Twist()
        # twist_msg.linear.x = 0.1  # Constant linear speed
        # twist_msg.angular.z = angular_speed # np.clip(angular_speed, -1.0, 1.0)
        # self.cmd_vel_publisher.publish(twist_msg)
        self.send_velocity(vL, vR)

        self.get_logger().info(f'Distance to line: {current_error} - Angular speed: {angular_speed}')
    def stop_robot(self):
        """
        Stop the robot by publishing a zero Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
