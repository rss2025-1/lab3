#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools

class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")
        
        # Declare parameters
        # self.WALL_TOPIC = "/wall"
        
        # Update these parameters for the physical racecar
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")  # Updated for racecar
        self.declare_parameter("side", 1)  # 1 for left, -1 for right
        self.declare_parameter("velocity", 1.0)
        self.declare_parameter("desired_distance", 0.5)

        # Fetch parameters
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # PID Controller parameters - tuned for real robot
        self.Kp = 3 # You may need to tune these for the physical robot
        self.Kd = 1
        self.prev_e = 0
        self.prev_time = self.get_clock().now()
        
        # LIDAR parameters
        self.max_scan_distance = 3.0
        self.left_or_right = self.SIDE
        
        # Publishers and Subscribers
        # self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_cb, 10)
        
        # Parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        self.get_logger().info("Wall follower initialized")
        self.get_logger().info("LEFT") if self.SIDE == 1 else self.get_logger().info("RIGHT")

    def split_by_ang_range(self, drive_msg, ang_bounds):
        """Takes in ranges and returns slice of angle from start_angle to end_angle"""
        num_ranges = len(drive_msg.ranges)
        ranges = np.array(drive_msg.ranges)
        angles = np.linspace(drive_msg.angle_min, drive_msg.angle_max, num_ranges)
        angle_start, angle_end = ang_bounds

        scan_polar_vectors = np.vstack((ranges, angles))
        scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1,:]<= angle_end) & (scan_polar_vectors[1,:]>= angle_start)]
        scan_polar_vectors = scan_polar_vectors[:, scan_polar_vectors[0, :] <= self.max_scan_distance]
        return scan_polar_vectors[0,:], scan_polar_vectors[1,:]

    def interpolate_line(self, ranges, angles):
        """Fit a line to the wall points"""
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        slope, intercept = np.polyfit(x_coords, y_coords, 1)
        new_y = slope * x_coords + intercept 

        # VisualizationTools.plot_line(x_coords, new_y, self.line_pub, frame="/laser")
        return slope, intercept

    def pid_controller(self, y, r):
        """PID controller for wall following"""
        e = y - r
        curr_time = self.get_clock().now()
        
        deriv = (e - self.prev_e)
        P = (self.Kp * e)
        D = (self.Kd * deriv)
        
        if self.SIDE == 1:
            u = -(P + D)
        else:
            u = P + D
            
        self.prev_e = e
        self.prev_time = curr_time
        return u

    def listener_cb(self, scan):
        """Main callback for processing LIDAR data and controlling the robot"""
        # Update parameters
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # Define scan angles based on which wall we're following
        wall_angle_start, wall_angle_end = (-15 * (np.pi/180), 135 * (np.pi/180)) if self.SIDE == 1 else (-135 * (np.pi/180), 15*(np.pi/180))
        
        # Process LIDAR data
        ranges, angles = self.split_by_ang_range(scan, [wall_angle_start, wall_angle_end])
        
        # Skip processing if we don't have enough points
        if len(ranges) < 2:
            self.get_logger().warn("Not enough points detected")
            return
            
        m, b = self.interpolate_line(ranges, angles)
        
        # Calculate distance to wall
        perp_dist = abs(b) / np.sqrt(m**2 + 1)
        
        # Calculate steering angle using PID
        steering_angle = self.pid_controller(self.DESIRED_DISTANCE, perp_dist)
        
        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.speed = self.VELOCITY
        drive_msg.drive.steering_angle = steering_angle
        
        self.drive_pub.publish(drive_msg)

    def parameters_callback(self, params):
        """Callback for parameter updates"""
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
