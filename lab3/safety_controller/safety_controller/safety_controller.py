#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        self.i = 0
        # Declare parameters
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/low_level/input/safety")

        # Retrieve parameter values
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lidar_dist = 0.1  # Distance from lidar to front of car
        # self.default_velocity = 1.0
        self.ang_bounds = -np.pi/6, np.pi/6
        self.car_width = 0.25
        self.count_threshold = 20  # Define threshold for stopping

        # ROS 2 Subscribers & Publishers
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/vesc/low_level/ackermann_cmd', self.drive_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.estop_cb, 10)

        self.should_estop = False
        self.estop_dist = 0.5
    def drive_callback(self, drive_msg):
        """ Stops the car if emergency stop condition is met """
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"

        if self.should_estop:
            drive_msg.drive.speed = 0.0
            self.get_logger().info("Emergency stop triggered!")
            self.drive_pub.publish(drive_msg)

        else:
            self.estop_dist = 0.5 * drive_msg.drive.speed
            # drive_msg.drive.speed = self.default_velocity
            # self.get_logger().info("Emergency stop triggered!")


    def estop_cb(self, scan_msg):
        """ Processes LIDAR scan data and determines if an emergency stop is needed """
        
        if self.i == 100:
            self.i = 0
            self.get_logger().info(f"estop_dist is {self.estop_dist}")
            self.get_logger().info(f"drivespeed is {self.estop_dist}")
        self.i+=1
        # angle_start, angle_end = self.ang_bounds
        # num_ranges = len(scan_msg.ranges)[180:220]
        ranges = np.array(scan_msg.ranges)[180:220]
        if np.any(ranges) < .5:
            self.get_logger().info(f"WE NEED TO STOP")
            self.should_estop = True
        else:
            self.should_estop = False
        # angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_ranges)
        # # mask_min_dist = np.where(ranges > self.lidar_dist)

        # # ranges = ranges[mask_min_dist]
        # # angles = angles[mask_min_dist]

        # mask_max_dist = np.where(ranges < self.estop_dist)

        # ranges = ranges[mask_max_dist]
        # angles = angles[mask_max_dist]

        # scan_polar_vectors = np.vstack((ranges, angles))
        # scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1, :] <= angle_end) & 
        #                                         (scan_polar_vectors[1, :] >= angle_start)]
        
        # # x_coords = ranges * np.cos(angles)
        # # y_coords = ranges * np.sin(angles)


        # mask_estop = (np.abs(y_coords) <= self.car_width) & (x_coords <= self.estop_dist)
        # close_points_count = np.sum(mask_estop)

        # self.should_estop = (close_points_count >= self.count_threshold)
        # if (close_points_count >= self.count_threshold):
        #     self.get_logger().info(f"close_points_count: {close_points_count}")




def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()