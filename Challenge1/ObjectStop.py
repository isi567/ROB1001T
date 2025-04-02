#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.safe_distance = 0.5  # Stop if an obstacle is closer than 0.5 meters
        self.moving_forward = True
        self.timer = self.create_timer(0.1, self.move_robot)


    def lidar_callback(self, msg):
        if not msg.ranges:
            return

        closest_distance = min([r for r in msg.ranges if r > 0])
        self.moving_forward = closest_distance > self.safe_distance

        if not self.moving_forward:
            self.root.after(0, self.show_warning)

    def show_warning(self):
        messagebox.showwarning("Warning", "Obstacle detected! Stopping.")

    def move_robot(self):
        twist_msg = Twist()
        if self.moving_forward:
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0
        else:
            twist_msg.linear.x = 0.0  # Stop if an obstacle is too close
            twist_msg.angular.z = 0.0


        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
