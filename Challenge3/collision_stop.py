#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool  # Import Bool message type


class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving_publisher = self.create_publisher(Bool, '/moving', 10)  # New moving topic publisher
        self.lidar_subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.button_subscription = self.create_subscription(Bool, '/button', self.button_callback, 10)  # Button subscription
        self.safe_distance = 0.5  # Stop if an obstacle is closer than 0.5 meters
        self.moving_forward = False  # Initially not moving
        self.button_pressed = False  # Button state
        self.timer = self.create_timer(0.1, self.move_robot)

    def lidar_callback(self, msg):
        if not msg.ranges:
            return

        closest_distance = min([r for r in msg.ranges if r > 0])
        self.moving_forward = closest_distance > self.safe_distance

    def button_callback(self, msg):
        # Update button state
        self.button_pressed = msg.data

    def move_robot(self):
        # Only execute if the button is pressed
        if not self.button_pressed:
            self.get_logger().info("Button not pressed, not moving.")
            return

        twist_msg = Twist()
        moving_status = Bool()

        if self.moving_forward:
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0
            moving_status.data = True  # Robot is moving
            self.get_logger().info("Moving forward at speed X: 0.2 m/s")
        else:
            twist_msg.angular.z = 0.0 
            twist_msg.linear.x = 0.0  # Stop if an obstacle is too close
            moving_status.data = False  # Robot is stopped
            self.get_logger().info("Obstacle detected, stopping robot.")

        self.publisher.publish(twist_msg)
        self.moving_publisher.publish(moving_status)  # Publish moving status


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