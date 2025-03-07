#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        
        # Declare parameters
        self.declare_parameter("estop_dist", 1.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety")

        # Retrieve parameter values
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lidar_dist = 0.1  # Distance from lidar to front of car
        self.default_velocity = 1
        self.ang_bounds = -np.pi/2, np.pi/2
        self.car_width = 0.5
        self.count_threshold = 5  # Define threshold for stopping

        # ROS 2 Subscribers & Publishers
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/vesc/low_level/ackermann_cmd', self.drive_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.estop_cb, 10)

        self.should_estop = False

    def drive_callback(self, drive_msg):
        """ Stops the car if emergency stop condition is met """
        if self.should_estop:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            self.drive_pub.publish(stop_msg)
            self.get_logger().info("Emergency stop triggered!")

    def estop_cb(self, scan_msg):
        """ Processes LIDAR scan data and determines if an emergency stop is needed """
        self.estop_dist = (self.default_velocity ** 2) / 9.81

        angle_start, angle_end = self.ang_bounds
        num_ranges = len(scan_msg.ranges)
        ranges = np.array(scan_msg.ranges)

        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_ranges)
        mask_min_dist = np.where(ranges > self.lidar_dist)

        ranges = ranges[mask_min_dist]
        angles = angles[mask_min_dist]

        scan_polar_vectors = np.vstack((ranges, angles))
        scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1, :] <= angle_end) & 
                                                (scan_polar_vectors[1, :] >= angle_start)]

        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        mask_estop = (np.abs(y_coords) <= self.car_width) & (x_coords <= self.estop_dist)
        close_points_count = np.sum(mask_estop)

        self.should_estop = (close_points_count >= self.count_threshold)

    def listener_cb(self, scan):
        """ Updates driving speed based on e-stop conditions """
        e_stop = self.should_estop

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        if e_stop:
            drive_msg.drive.speed = 0
        else:
            self.estop_dist = self.default_velocity ** 2 / 9.81
            drive_msg.drive.speed = self.default_velocity

        self.drive_pub.publish(drive_msg)


def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
