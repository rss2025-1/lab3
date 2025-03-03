#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
from wall_follower.visualization_tools import VisualizationTools


class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("estop_dist", 1.0)


        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.lidar_dist = .1 #distance from lidar to front of car
        self.add_on_set_parameters_callback(self.parameters_callback)
  

        self.default_velocity = 1
        self.ang_bounds = -np.pi/2, np.pi/2
        self.car_width = .5
        self.drive_sub = self.create_subscription(AckermannDriveStamped, '/vesc/low_level/ackermann_cmd',self.drive_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/vesc/low_level/input/safety', 10)
        self.lidar_sub = self.create_subscription(LaserScan,'\scan', self.estop_cb, 10)


        self.should_estop = False
    def drive_callback(self, drive_msg):
        
        if self.should_estop:
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0
            self.publisher.publish(stop_msg)
            self.logger.info("stop")

    def estop_cb(self,drive_msg):
        """
        Takes in ranges (100,0 )
        Returns slice of angle from start_angle to end_angle
        ang_bounds is ang_start, ang_end
        """
        self.estop_dist = (drive_msg.drive.speed ** 2) / 9.81

        angle_start, angle_end = self.ang_bounds
        num_ranges = len(drive_msg.ranges)
        ranges = np.array(drive_msg.ranges)
        
        angles = np.linspace(drive_msg.angle_min, drive_msg.angle_max, num_ranges)
        mask_min_dist = np.where(ranges > self.lidar_dist)

        ranges = ranges[mask_min_dist]
        angles = angles[mask_min_dist]


        scan_polar_vectors = np.vstack((ranges, angles))
        scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1,:]<= angle_end) & (scan_polar_vectors[1,:]>= angle_start)]

        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)
        mask_estop = (np.abs(y_coords) <= self.body_width) & (x_coords <= estop_dist) #d
        close_points_count = np.sum(mask_estop)

        self.should_estop = (close_points_count >= self.count_threshold)
     
    def listener_cb(self, scan):
           
        wall_angle_start, wall_angle_end = -np.pi/2, np.pi/2
      
        e_stop = self.e_stop(scan, [wall_angle_start, wall_angle_end])
      
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        if e_stop:
           drive_msg.drive.speed = 0
        else:
            self.estop_dist = self.default_velocity  ** 2 / 9.81
            drive_msg.drive.speed = self.default_velocity


        self.drive_pub.publish(drive_msg)

    

def main():
    rclpy.init()
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

