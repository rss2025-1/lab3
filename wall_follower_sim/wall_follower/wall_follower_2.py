#!/usr/bin/env python3
import numpy as np
import rclpy
import random
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class WallFollower(Node):

    WALL_TOPIC = "/wall"

    def __init__(self):
        super().__init__("wall_follower")

        # Declare parameters to make them available for use
        self.declare_parameter("scan_topic", "default")
        self.declare_parameter("drive_topic", "default")
        self.declare_parameter("side", "default")
        self.declare_parameter("velocity", "default")
        self.declare_parameter("desired_distance", "default")

        # Fetch constants from the ROS parameter server

        #self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.SCAN_TOPIC = "/scan"
        #self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = "/drive"
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        # self.SIDE = 1 # test left side
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        # Initialize publishers and subscribers
        self.publisher1 = self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        self.subscription = self.create_subscription(LaserScan, self.SCAN_TOPIC, self.listener_callback, 10)
        self.publisher2 = self.create_publisher(Float32, 'error', 10)

        # Initialize variables for PD controller
        self.error_prev = 0.
        self.Kp = 3. # 50
        self.Kd = 1 # 30
        self.Km = 0. # 10
        

        ## Angle Constants
        self.side_min_angle_magnitude = -15* (np.pi/180)
        self.side_max_angle_magnitude = 135 * (np.pi/180)
        self.max_scan_distance = 3

    def listener_callback(self, msg):
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        # self.SIDE = 1 # test left side
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value

        drive_msg = AckermannDriveStamped()

        ## Begin of Old ##
        # # Get polar coordinates from laser scan msg
        # ranges = np.array(msg.ranges)
        # angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)

        # #Split scan data into left & right sections within angle limits
        # right_mask = np.where(angles <= -np.pi/16)
        # # left_mask  = np.where(angles >= np.pi/16)
        # # right_mask = np.where((angles >= -np.pi/2) & (angles <= -np.pi/16))
        # left_mask  = np.where((angles >= 0) & (angles <= np.pi/2))
        # right_ranges = ranges[right_mask]
        # right_angles = angles[right_mask]
        # left_ranges  = ranges[left_mask]
        # left_angles  = angles[left_mask]

        # # Compute Cartesian coordinates from polar coordinates for side of interest
        # if self.SIDE == -1: # right side
        #     side_ranges, side_angles = right_ranges, right_angles
        # else: # left side
        #     side_ranges, side_angles = left_ranges, left_angles
        # x, y = self.polar_to_cartesian_coords(side_ranges, side_angles)

        # # Remove outliers (very far points)
        # outlier_mask = np.where(side_ranges < 5)
        # x = x[outlier_mask]
        # y = y[outlier_mask]
        ## End of Old ##

        num_points = len(msg.ranges)
        distances = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, num_points)
        scan_polar_vectors = np.vstack((distances, angles))
        
        # Filter Points based on Side & Angles
        if self.SIDE == 1:
            scan_polar_vectors = scan_polar_vectors[:, (scan_polar_vectors[1,:]<= self.side_max_angle_magnitude) & (scan_polar_vectors[1,:]>= self.side_min_angle_magnitude)]
        else:
            scan_polar_vectors = scan_polar_vectors[:, (-scan_polar_vectors[1,:] <= self.side_max_angle_magnitude) & (-scan_polar_vectors[1,:] >= self.side_min_angle_magnitude)]
        
        # Filter Points based on Look-Ahead Distance
        scan_polar_vectors = scan_polar_vectors[:, scan_polar_vectors[0, :] <= self.max_scan_distance]
            
        # Find X,Y points 
     
    
        x = scan_polar_vectors[0,:]*np.cos(scan_polar_vectors[1,:])
        y = scan_polar_vectors[0,:]*np.sin(scan_polar_vectors[1,:])


        # Do linear regression on data points and visualize line
        m, b = self.linear_regression(x, y)
        # m, b = self.ransac(x,y,len(x))
        x_endpoints = np.array([x[0], x[-1]])
        y_endpoints = x_endpoints*m + b

        # PD controller
        measured_dist = abs(b)/np.sqrt(m**2+1)
        error = self.DESIRED_DISTANCE - measured_dist

        P = self.Kp*error
        D = self.Kd*(error-self.error_prev)
        M = self.Km*m

        if self.SIDE == -1: # right side
            steering_angle = P+D+M
        else: # left side
            steering_angle = -(P+D)+M

        self.error_prev = error

        # Publish drive msg
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = self.VELOCITY
        # drive_msg.drive.steering_angle = 0.
        # drive_msg.drive.speed = 0.
        self.publisher1.publish(drive_msg)

        # Publish error for plotting
        error_msg = Float32()
        error_msg.data = error
        self.publisher2.publish(error_msg)

    def polar_to_cartesian_coords(self, ranges, angles):
        return np.array([r*np.cos(a) for r,a in zip(ranges, angles)]), np.array([r*np.sin(a) for r,a in zip(ranges, angles)])

    def linear_regression(self, x, y):
        A = np.vstack([x, np.ones(len(x))]).T
        m, b = np.linalg.lstsq(A, y, rcond=None)[0]
        return m, b

    def ransac(self,X, y, sample_size, n_iterations=50, inlier_threshold=1.0):
        best_model = None
        max_inliers = 0

        #Take 75% of the points in the sample 
        if sample_size > 1: 
            sample_size_prime = int(sample_size * (3/4))

            for _ in range(n_iterations):
                # Step 1: Randomly sample a subset of points
                random_indices = random.sample(range(len(X)), sample_size_prime)
                X_sampled = X[random_indices]
                y_sampled = y[random_indices]

                # Step 2: Fit a model to the sampled points
                model = np.polyfit(X_sampled, y_sampled, 1)  # Linear regression model, adjust degree as needed

                # Step 3: Calculate inliers based on the model
                residuals = np.abs(y - np.polyval(model, X))
                inliers = np.where(residuals < inlier_threshold)[0]

                # Step 4: Check if the current model has more inliers
                if len(inliers) > max_inliers:
                    max_inliers = len(inliers)
                    best_model = model

        else:
            best_model  = (0,0)

        return best_model

def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
