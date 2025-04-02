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
        self.safe_distance = 0.3  # Avoid obstacles closer than 0.5 meters
        self.moving_forward = True
        self.turning = False
        self.timer = self.create_timer(0.1, self.move_robot)
        self.closest_distance = float('inf')
        self.closest_angle = 0.0

    def lidar_callback(self, msg):
        if not msg.ranges:
            return

        # Find the closest obstacle and its angle
        self.closest_distance = float('inf')
        self.closest_angle = 0.0
        for i, distance in enumerate(msg.ranges):
            if distance > 0 and distance < self.closest_distance:
                self.closest_distance = distance
                self.closest_angle = i

        # Determine if the robot should turn or move forward
        self.moving_forward = self.closest_distance > self.safe_distance
        self.turning = not self.moving_forward

    def move_robot(self):
        twist_msg = Twist()
    
        if self.moving_forward:
            twist_msg.linear.x = 0.2  # Move forward
            twist_msg.angular.z = 0.0
        elif self.turning:
            # Check if obstacles are directly ahead
            if self.closest_distance < self.safe_distance and (self.closest_angle < 30 or self.closest_angle > 330):
                twist_msg.linear.x = -0.2  # Reverse slightly
                twist_msg.angular.z = 0.0
                self.get_logger().info(f'Reversing! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')
                print('Reversing!')
            else:
                # Turn away from the obstacle
                if self.closest_angle < 180:
                    twist_msg.angular.z = 0.5  # Turn left
                    self.get_logger().info(f'Turning left! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')
                else:
                    twist_msg.angular.z = -0.5  # Turn right
                    self.get_logger().info(f'Turning right! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')
    
        # # Check if the robot is stuck (e.g., oscillating between left and right turns)
        if self.closest_distance < self.safe_distance and self.closest_angle > 120 and self.closest_angle < 240:
            twist_msg.linear.x = -0.2  # Reverse slightly
            twist_msg.angular.z = 0.5
            self.get_logger().info('Fallback: Reversing to avoid being stuck.')
    
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LidarAvoidance()
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
