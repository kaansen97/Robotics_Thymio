#!/usr/bin/env python3

import rclpy
import math
import numpy as np
# from utils import ThymioController
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf_transformations



class ThymioController:

    def __init__(self, node_name):
        """Initialization."""

        # initialize the node
        # rclpy.init()
        self.node = rclpy.create_node(node_name)
        self.name=node_name
        self.node.get_logger().info('Controlling')

        # create velocity publisher
        self.velocity_publisher = self.node.create_publisher(Twist,'cmd_vel', 10)

        # create subscribers
        self.pose_subscriber = self.node.create_subscription(Odometry,'odom', self.log_odometry, 10)
        self.proximity_center_subscriber = self.node.create_subscription(Range,'proximity/center', self.update_proximity_center, 10)
        self.proximity_left_subscriber = self.node.create_subscription(Range,'proximity/left', self.update_proximity_left, 10)
        self.proximity_right_subscriber = self.node.create_subscription(Range,'proximity/right', self.update_proximity_right, 10)

        self.node.get_logger().info('Subscribed to topics')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        self.pose = None  # initialize pose to (X=0, Y=0, theta=0)
        self.velocity = None  # initialize linear and angular velocities to 0
        self.proximity_center = Range()
        self.proximity_left = Range()
        self.proximity_right = Range()
        self.proximity_center.range = 1.0
        self.proximity_left.range = 1.0
        self.proximity_right.range = 1.0
        self.rate = self.node.create_rate(10)  # set update frequency in Hz


    def update_proximity_center(self, data):
        self.proximity_center = data

    def update_proximity_left(self, data):
        self.proximity_left = data

    def update_proximity_right(self, data):
        self.proximity_right = data


    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        result = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return result


    def log_odometry(self, data):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = data.pose.pose
        self.velocity = data.twist.twist
        
        pose2d=self.pose3d_to_2d(self.pose)
        self.node.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        
    def euclidean_distance_2d(self, goal_x, goal_y):
        """Euclidean distance between current pose and the goal."""
        (current_x, current_y, _) = self.pose3d_to_2d(self.pose)
        return math.sqrt(math.pow((goal_x - current_x), 2) +
                         math.pow((goal_y - current_y), 2))
    
    def move(self, linear_vel=0.15, angular_vel=0.0):
        """Moves the robot accordingly to given velocities."""

        velocity = Twist(
        linear=Vector3(x=linear_vel, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=angular_vel))
        self.velocity_publisher.publish(velocity)
        self.rate.sleep()

    def stop(self):
        """Stops the robot."""

        velocity=Twist()
        self.velocity_publisher.publish(velocity)

    def rotate(self, angle, speed=100):
        vel_msg = Twist()
        clockwise = True if angle < 0 else False
        # Converting from angles to radians
        relative_angle = np.deg2rad(abs(angle))
        angular_speed = np.deg2rad(speed)

        if clockwise:
            angular_speed = -angular_speed

        # Setting the current time for distance calculus
        t0 = self.node.get_clock().now().to_msg().sec
        current_angle = 0

        while current_angle < relative_angle:
            self.move(0, angular_speed)
            t1 = self.node.get_clock().now().to_msg().sec
            current_angle = abs(angular_speed)*(t1-t0)
    
    def is_exactly_facing_an_obstacle(self, tolerance=0.003):
        return  self.proximity_left.range < self.proximity_left.max_range and \
                self.proximity_right.range < self.proximity_right.max_range and \
                abs(self.proximity_left.range - self.proximity_right.range) < tolerance

    def collision(self, distance = 0.08):
        return  self.proximity_center.range < distance or \
                self.proximity_left.range < distance or \
                self.proximity_right.range < distance


    

    


class ThymioController_Task1(ThymioController):

    def run(self, desired_radius = 0.25):
        """Controls the Thymio."""

        self.node.get_logger().info('%s Following an 8 trajectory...' % self.name)

        linear_speed = 0.14
        angular_speed = linear_speed / desired_radius
        circumference = 2 * np.pi * desired_radius

        time_needed = (circumference / linear_speed)

        self.node.get_logger().info('%s Time required to complete one full tour: %f' % (self.name, time_needed))
        time_start = self.node.get_clock().now()

        while rclpy.ok():
            self.move(linear_speed, angular_speed)

            if (self.node.get_clock().now() - time_start) > time_needed:
                time_start = self.node.get_clock().now()
                angular_speed = -angular_speed


def main(args=None):

    rclpy.init(args=args)

    controller = ThymioController_Task1('assignment2_task1')

    try:
        controller.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        controller.node.get_logger().error('Unhandled exception: ' + str(e))

    controller.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

