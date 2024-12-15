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

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')

        # Initialize serial communication with the Arduino
        self.arduino_serial = serial.Serial('/dev/ttyUSB1', 9600, timeout=1)  # Replace with the correct port
        time.sleep(2)

        # Robot parameters
        self.wheel_base = 0.185  # Distance between wheels in meters

        # Parameters
        self.path = np.load('hehe_path.npy')  # Load saved path
        #self.path = self.path_before[:, [1, 0]]
        self.grid_map = np.load('map3.npy')  # Load grid map
        self.grid_size = 0.03  # Grid resolution in meters per cell
        self.pose = np.array([(self.path[0][0] * self.grid_size), (self.path[0][1] * self.grid_size), 0])  # Start pose based on the first path point
        self.prev_scan = None  # Store the previous scan
        self.path_index = 0
        self.goal_tolerance = 0.4  # Tolerance for reaching a path point
        self.ahead_distance = 0.2  # Distance to look ahead on the path

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

        # Add the publisher for the closest point marker
        self.closest_point_publisher = self.create_publisher(Marker, '/closest_point_marker', 10)
        self.create_timer(1.0, self.publish_closest_point_marker)

        #PID gains:
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0
        
        #PID state variables:
        self.previous_error = 0.0
        self.integral_error = 0.0

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
        # occupancy_grid.info.origin.orientation.z = 1.571


        # Flatten and normalize the grid map to match OccupancyGrid format
        grid_data = (self.grid_map.flatten() * 100).astype(np.int8)  # Scale [0, 1] to [0, 100]
        grid_data = np.clip(grid_data, 0, 100).astype(np.int8)  # Clamp to [0, 100]
        occupancy_grid.data = [int(value) if 0 <= value <= 100 else -1 for value in grid_data]  # Use -1 for unknown cells

        self.grid_map_publisher.publish(occupancy_grid)

    def publish_path(self):
        """
        Publish the path as a Marker message.
        """
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
        print(f"Closest point: {closest_point}, type: {type(closest_point)}")
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
        closest_point_marker.pose.position.x = float(closest_point[0]) * self.grid_size
        closest_point_marker.pose.position.y = float(closest_point[1]) * self.grid_size
        closest_point_marker.pose.position.z = 0.1  # Slightly above the ground

        print(f"Publishing marker at: ({closest_point_marker.pose.position.x}, {closest_point_marker.pose.position.y})")

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
        self.current_scan = points

        if self.prev_scan is not None:
            # Apply ICP to calculate the relative transformation
            delta_pose = self.icp(self.prev_scan, points)
            self.pose = self.update_pose(self.pose, delta_pose)
            self.get_logger().info(f'Updated Pose: {self.pose}')

            # Publish the robot marker to visualize the position
            self.publish_robot_marker()

            # Control the robot to follow the path
            self.control_robot()

        self.prev_scan = self.current_scan

    def icp(self, source, target, max_iterations=50, tolerance=1e-3):
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

        return np.array([-dx, dy, -dtheta])

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
        # NEED TO FIND THE AHEAD POINT
        # x_ap = 1.2
        # y_ap = 0.8
        # return np.array([x_ap, y_ap])
        x,y,theta =  pose 
        self.ahead_distance = 0.2
        x_ap = x + self.ahead_distance * np.cos(theta)
        y_ap = y + self.ahead_distance * np.sin(theta)

        return np.array([x_ap, y_ap])

    def find_closest_path_point(self, ahead_point):
        ahead_point = np.array(ahead_point)  # Ensure ahead_point is a numpy array
        # self.get_logger().info(f"Ahead point: {ahead_point}")
    # Compute distances from the ahead point to all points in the path
        distances = np.linalg.norm(self.path*self.grid_size - ahead_point, axis=1)  # axis=1 ensures distance is calculated for each [x, y]
        # self.get_logger().info(f"Distance: {distances}")
        # self.get_logger().info(f"Path: {self.path}")
        closest_index = np.argmin(distances)  # Find the index of the closest point

        closest_point = self.path[closest_index]  # Get the closest point
        # self.get_logger().info(f"Closest point: {closest_point} at index {closest_index}")
        return closest_point,closest_index

    
    def calculate_pid(self, error):
        P = error 
        self.integral_error += error 
        D = error - self.previous_error
        pid_output = (self.kp * P) + (self.ki * self.integral_error) + (self.kd * D)
        self.previous_error = error 

        return pid_output
    
    def calculate_lateral_error(self, pose, closest_point, ahead_point):
        look_ahead_distance = 0.2
        x_robot, y_robot, theta = pose
        print(f"Coordinate robot: x = {x_robot}, y = {y_robot}")
        x_closest, y_closest = closest_point * self.grid_size
        print(f"Coordinate path: x = {x_closest}, y = {y_closest}")
        x_ahead, y_ahead = ahead_point
        print(f"Coordinate ahead point: x = {x_ahead}, y = {y_ahead}")

    # Vectors in 2D space
        v_ahead = np.array([x_ahead - x_robot, y_ahead - y_robot])  # Vector from robot to ahead point
        
        v_closest = np.array([x_closest - x_robot, y_closest - y_robot])  # Vector from robot to closest point

        print(f"Vector ahead: {v_ahead}")
        print(f"Vector closest: {v_closest}")

    # Compute 2D cross product
        cross_product = np.cross(v_ahead, v_closest)  # This gives the 2D equivalent cross product scalar value
        print(f"Cross product value: {cross_product}")
    # Normalize by the distance magnitude
        distance_magnitude = np.linalg.norm(v_ahead)

        if distance_magnitude != 0:
            lateral_error = cross_product / distance_magnitude
        else:
            lateral_error = 0.0  # Avoid division by zero

        return lateral_error


    def stop_robot(self):
        """
        Stop the robot by publishing a zero Twist message.
        """
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg)


    def control_robot(self):
        if self.prev_scan is not None:
            ahead_point = self.get_ahead_point(self.pose)
            closest_point, _ = self.find_closest_path_point(ahead_point)
        
        # Calculate lateral error
            lateral_error = self.calculate_lateral_error(self.pose, closest_point, ahead_point)
        
        # Debugging print for lateral error
            print(f"Lateral error: {lateral_error}")

        # Compute angular velocity from PID
            angular_velocity = self.calculate_pid(lateral_error)
            angular_velocity = np.clip(angular_velocity, -0.5, 0.5)
            linear_velocity = 0.06  # You can adjust this value as needed for your robot's speed

        # Debugging print for PID output (angular velocity)
            print(f"PID output - Angular velocity: {angular_velocity}")
           
        # Compute left and right wheel velocities
            linear_velocity = 0.06  # Constant forward speed
            v_left = linear_velocity - (angular_velocity * self.wheel_base) / 2
            v_right = linear_velocity + (angular_velocity * self.wheel_base) / 2
            

        # Debugging print for PID output (angular velocity)
            print(f"V_L: {v_left}")
            print(f"V_R: {v_right}")

        # Send velocities to Arduino
            self.send_to_arduino(v_left, v_right)  
              
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(twist_msg)

            distances = np.linalg.norm(self.path[-1]*self.grid_size - self.pose[:2])
            if distances < 0.05:
                self.stop_robot()


    def send_to_arduino(self, vL, vR):
        command = f"{vL} {vR}\n"
        self.arduino_serial.write(command.encode())
        # Create the command string
        command = f"{vL} {vR}\n"

        # Send the command to Arduino
        self.arduino_serial.write(command.encode())

        # Print the sent command
        print(f"Sent: {command.strip()} to Arduino")    

            

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