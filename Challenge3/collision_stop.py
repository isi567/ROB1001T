#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from time import time  # Import time module for rotation timing


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving_publisher = self.create_publisher(Bool, '/moving', 10)
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.button_subscription = self.create_subscription(Bool, '/button', self.button_callback, 10)
        self.safe_distance = 0.5  # Stop if an obstacle is closer than 0.5 meters
        self.moving_forward = False  # Initially not moving
        self.button_pressed = False  # Button state
        self.robot_state = "stopped"  # Initial state
        self.timer = self.create_timer(0.1, self.move_robot)
        self.rotation_start_time = None  # Track the start time of rotation

    def lidar_callback(self, msg):
        if self.robot_state == "rotate":
            # Ignore obstacle detection while rotating
            return

        if not msg.ranges:
            return

        closest_distance = min([r for r in msg.ranges if r > 0])
        if closest_distance <= self.safe_distance:
            self.moving_forward = False  # Stop if an obstacle is too close
            self.robot_state = "rotate"  # Set state to rotate
            self.rotation_start_time = None  # Reset rotation timer
            self.get_logger().info(f"Obstacle detected at {closest_distance:.2f} m, rotating robot.")
        else:
            self.moving_forward = True  # Safe to move

    def button_callback(self, msg):
        # Update button state
        self.button_pressed = msg.data
        self.get_logger().info(f"Button pressed: {self.button_pressed}")
        if self.button_pressed:
            self.robot_state = "moving"  # Set robot state to moving
            self.get_logger().info("Button pressed, robot will move until an obstacle is detected.")

    def move_robot(self):
        twist_msg = Twist()

        if self.robot_state == "moving" and self.moving_forward:
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0
            self.get_logger().info("Moving forward at speed X: 0.2 m/s")
        elif self.robot_state == "rotate":
            if self.rotation_start_time is None:
                self.rotation_start_time = time()  # Record the start time of rotation

            elapsed_time = time() - self.rotation_start_time
            if elapsed_time < 2.5:  # Rotate for 2 seconds (adjust as needed for 180 degrees)
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 1.57  # Rotate in place
                self.get_logger().info(f"Rotating robot, elapsed time: {elapsed_time:.2f} seconds.")
            else:
                self.robot_state = "stopped"  # Stop rotation after 2 seconds
                self.rotation_start_time = None  # Reset rotation timer
                self.get_logger().info("Rotation complete, stopping robot.")
        else:
            twist_msg.angular.z = 0.0
            twist_msg.linear.x = 0.0  # Stop if an obstacle is too close or no button press
            self.get_logger().info("Stopping robot.")

        self.publisher.publish(twist_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()