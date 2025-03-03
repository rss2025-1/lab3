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
        super().__init__("safety_controller_sim")
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")

        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value #"/scan"#
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value #"/drive"#

        self.lidar_dist = .1 #distance from lidar to front of car
        self.add_on_set_parameters_callback(self.parameters_callback)
        self.count_threshold = 10

        self.default_velocity = 4.0
        self.estop_dist = self.default_velocity  ** 2 / 9.81#self.get_parameter('estop_dist').get_parameter_value().double_value

        self.car_width = .3
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC,self.listener_cb, 10)

    def estop(self,drive_msg, ang_bounds):
        """
        Takes in ranges (100,0 )
        Returns slice of angle from start_angle to end_angle
        ang_bounds is ang_start, ang_end
        """
        angle_start, angle_end = ang_bounds
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
        mask_estop = (np.abs(y_coords) <= self.car_width) & (x_coords <= self.estop_dist)
        close_points_count = np.sum(mask_estop)

        if close_points_count >= self.count_threshold:
            return True
        else:
            return False
    def listener_cb(self, scan):
           
        wall_angle_start, wall_angle_end = -np.pi/2, np.pi/2
      
        estop = self.estop(scan, [wall_angle_start, wall_angle_end])
      
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        if estop:
            drive_msg.drive.speed = self.default_velocity/3
            
            drive_msg.drive.speed = 0.0
        else:
            self.estop_dist = self.default_velocity  ** 2 / 9.81
            drive_msg.drive.speed = self.default_velocity


        self.drive_pub.publish(drive_msg)
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
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
    safety_controller = SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
