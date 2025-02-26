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
        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.WALL_TOPIC = "/wall"

        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)
  
        self.Kp = 5
        self.Kd = 6
        self.prev_e = 0  # Previous error for derivative term

        self.max_scan_distance = 3
        
        self.left_or_right = self.SIDE
        
        self.line_pub = self.create_publisher(Marker, self.WALL_TOPIC , 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.lidar_sub = self.create_subscription(LaserScan, self.SCAN_TOPIC,self.listener_cb, 10)
        self.get_logger().info("LEFT") if self.SIDE == 1 else self.get_logger().info("RIGHT")

    def split_by_ang_range(self,drive_msg, ang_bounds):
       """
       Takes in ranges (100,0 )
       Returns slice of angle from start_angle to end_angle
       ang_bounds is ang_start, ang_end
       """
       num_ranges = len(drive_msg.ranges)
       ranges = np.array(drive_msg.ranges)
       angles = np.linspace(drive_msg.angle_min, drive_msg.angle_max, num_ranges)
       angle_start, angle_end = ang_bounds

       scan_polar_vectors = np.vstack((ranges, angles))
       scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1,:]<= angle_end) & (scan_polar_vectors[1,:]>= angle_start)]
       scan_polar_vectors = scan_polar_vectors[:, scan_polar_vectors[0, :] <= self.max_scan_distance]
       return scan_polar_vectors[0,:], scan_polar_vectors[1,:] #sliced ranges, sliced angles
    
    def interpolate_line(self, ranges, angles):

       x_coords = ranges * np.cos(angles)
       y_coords = ranges * np.sin(angles)

       slope, intercept  = np.polyfit(x_coords, y_coords, 1) #regression
       
       new_y = slope * x_coords + intercept 

       VisualizationTools.plot_line(x_coords,new_y, self.line_pub, frame="/laser")

       return slope, intercept
    def pid_controller(self, y, r):
       e = y - r
    #    self.left_or_right = (-1 if self.prev_e <= e else 1) * self.left_or_right
       curr_time = self.get_clock().now()
       
       deriv = (e-self.prev_e) #/ (curr_time - self.prev_time)
       P = (self.Kp * e)
       D = (self.Kd * deriv)
       if self.SIDE == 1:
        u = - P - D#* self.left_or_right#+ (self.Kd * deriv)
       else:
        u = P + D
       self.prev_e = e
       self.prev_time = curr_time

       return u  # Clamp steering angle
    def listener_cb(self, scan):
       #check whcih wall we want to scan and pick a wall based on it
               #slice into left wall (-90, 0 )
       self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
       self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
       self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

       wall_angle_start, wall_angle_end = (-15 * (np.pi/180), 135 * (np.pi/180)) if self.SIDE == 1 else ( -135 * (np.pi/180), 15*(np.pi/180))
      
       # self.get_logger().info(f'{wall_angle_start, wall_angle_end}')
       ranges, angles = self.split_by_ang_range(scan, [wall_angle_start, wall_angle_end])
       m, b =self.interpolate_line(ranges, angles)

       
       perp_dist = abs(b) / np.sqrt(m**2 + 1)
       r = perp_dist #np.pi/2 -  np.arctan(m)
       y = self.DESIRED_DISTANCE#np.pi/2
       steering_angle = self.pid_controller(y,r)
    #    self.get_logger().info(f"steering angle: {steering_angle}")


       #DRIVING
       drive_msg = AckermannDriveStamped()
       drive_msg.header.stamp = self.get_clock().now().to_msg()
       drive_msg.header.frame_id = "base_link"
      
       drive_msg.drive.speed = self.VELOCITY
       drive_msg.drive.steering_angle = steering_angle#steering_angle# #find steering angle based on u of pd control


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
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

