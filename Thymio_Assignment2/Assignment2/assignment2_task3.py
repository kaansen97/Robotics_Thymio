#!/usr/bin/env python3

import rclpy
import math
from math import pi
import numpy as np
from rclpy.node import Node
# from utils import ThymioController
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from geometry_msgs.msg import Pose, Twist, Vector3
from tf2_ros import TransformBroadcaster, TransformStamped
from tf2_ros import TransformListener, Buffer
import tf2_ros
import tf_transformations


class ThymioController(Node):

    def __init__(self):
        super().__init__('assignnment2_task3')
        """Initialization."""

        self.name='thymio0'
        self.get_logger().info('Controlling')

        # create velocity publisher
        self.velocity_publisher = self.create_publisher(Twist,'cmd_vel', 10)

        # create subscribers
        self.pose_subscriber = self.create_subscription(Odometry,'/thymio0/odom', self.log_odometry, 10)
        self.proximity_center_subscriber = self.create_subscription(Range,'/thymio0/proximity/center', self.update_proximity_center, 10)
        self.proximity_left_subscriber = self.create_subscription(Range,'/thymio0/proximity/left', self.update_proximity_left, 10)
        self.proximity_right_subscriber = self.create_subscription(Range,'/thymio0/proximity/right', self.update_proximity_right, 10)
        self.proximity_rear_left_subscriber = self.create_subscription(Range,'/thymio0/proximity/rear_left', self.update_proximity_rear_left, 10)
        self.proximity_rear_right_subscriber = self.create_subscription(Range,'/thymio0/proximity/rear_right', self.update_proximity_rear_right, 10)
        self.get_logger().info('Subscribed to topics')

        self.pose = None  # initialize pose to (X=0, Y=0, theta=0)
        self.velocity = None  # initialize linear and angular velocities to 0
        self.proximity_center = Range()
        self.proximity_left = Range()
        self.proximity_right = Range()
        self.proximity_rear_left = Range()
        self.proximity_rear_right = Range()
        self.proximity_center= 1.0
        self.proximity_left = 1.0
        self.proximity_right = 1.0
        self.proximity_rear_left = 1.0
        self.proximity_rear_right = 1.0
        self.linear_vel=0.15
        self.angular_vel=0.0
        self.finish_rotate=False
        self.facing_the_wall=False
        self.turned=False
        self.obstacle=False
        self.rate = self.create_rate(10)  # set update frequency in Hz


    def update_proximity_center(self, msg):
        self.proximity_center = msg.range

    def update_proximity_left(self, msg):
        self.proximity_left = msg.range

    def update_proximity_right(self, msg):
        self.proximity_right = msg.range
    
    def update_proximity_rear_right(self, msg):
        self.proximity_rear_right = msg.range
        
    def update_proximity_rear_left(self, msg):
        self.proximity_rear_left = msg.range
    
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


    def log_odometry(self, msg):
        """Updates robot pose and velocities, and logs pose to console."""

        self.pose = msg.pose.pose
        self.velocity = msg.twist.twist

        pose2d=self.pose3d_to_2d(self.pose)
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        
    def euclidean_distance_2d(self, goal_x, goal_y):
        """Euclidean distance between current pose and the goal."""
        (current_x, current_y, _) = self.pose3d_to_2d(self.pose)
        return math.sqrt(math.pow((goal_x - current_x), 2) +
                         math.pow((goal_y - current_y), 2))
    
    def move(self, linear_vel=0.15,angular_vel=0.0):
        velocity = Twist(
        linear=Vector3(x=linear_vel, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=angular_vel))
        self.velocity_publisher.publish(velocity)

    def update_callback(self):
        # self.get_logger().info("Update Callback works now")
        wall_distance = 0.08
        goal_x=0
        goal_y=0
        if (not self.collision(wall_distance)and self.finish_rotate==False):
            self.move()
        if (self.collision(wall_distance)and (self.obstacle==False)): 
            self.get_logger().info("Wall Ahead")
            self.obstacle=True
            self.stop()
            (init_x,init_y,init_theta)=self.pose3d_to_2d(self.pose)
            goal_x = 2 * math.cos(init_theta) + init_x
            goal_y = 2 * math.sin(init_theta) + init_y
        elif(self.obstacle and not self.turned):
            self.get_logger().info("Facing to the Wall")
            self.facing_wall()
        elif(self.obstacle and self.facing_the_wall and not self.finish_rotate):
            self.get_logger().info("Rotating")
            self.rotate(180)
        if(self.turned and self.finish_rotate):
            if(self.euclidean_distance_2d(goal_x,goal_y)<wall_distance):
                    self.move()
            else:
                self.stop()
                self.get_logger().info("Done")


    def stop(self):
        """Stops the robot."""

        velocity=Twist()
        self.velocity_publisher.publish(velocity)


    def facing_wall(self):
        angular_value = abs(self.proximity_left-self.proximity_right)/10.0
        error = abs(self.proximity_left-self.proximity_right)
        if(error > 0.05):
            if(self.proximity_left > self.proximity_right):
                self.move(0.0,angular_value)
                self.get_logger().info("Turning left")
            else:
                self.move(0.0,-angular_value)
                self.get_logger().info("Turning right")
        else:
            self.angular_speed_z = 0.0
            self.turned = True
            self.facing_the_wall=True
        return
    def rotate(self,angle=180):
        vel_msg = Twist()
        clockwise = True if angle < 0 else False
        relative_angle = np.deg2rad(abs(angle))
        angular_speed =0.5

        if clockwise:
            angular_speed = -angular_speed

        t0 = self.get_clock().now().to_msg().sec
        current_angle = 0

        while current_angle < relative_angle:
            self.move(0.0, angular_speed)
            t1 = self.get_clock().now().to_msg().sec
            current_angle = abs(angular_speed)*(t1-t0)
        return
    def collision(self, distance = 0.08):
        return  (abs(self.proximity_center) < distance) or (abs(self.proximity_left) < distance) or (abs(self.proximity_right) < distance)

    def start(self):
        self.create_timer(0.1, self.update_callback)

def main(args=None):
    rclpy.init()
    node = ThymioController()
    done = node.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.stop()


if __name__ == '__main__':
    main()
