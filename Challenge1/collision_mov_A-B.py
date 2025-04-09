#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class LidarAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.safe_distance = 0.3  # Avoid obstacles closer than 0.3 meters
        self.moving_forward = True
        self.turning = False
        self.timer = self.create_timer(0.1, self.move_robot)
        self.closest_distance = float('inf')
        self.closest_angle = 0.0

        # Goal-based navigation variables
        self.goal_position = (2.0, 0.0)  # Target position (x, y) in meters
        self.current_position = (0.0, 0.0)  # Current position (x, y)
        self.current_angle = 0.0  # Current orientation in radians

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

        # Calculate distance and angle to the goal
        goal_x, goal_y = self.goal_position
        current_x, current_y = self.current_position
        distance_to_goal = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)

        # Normalize the angle difference to [-pi, pi]
        angle_diff = (angle_to_goal - self.current_angle + math.pi) % (2 * math.pi) - math.pi

        # Check if the robot has reached the goal
        if distance_to_goal < 0.1:  # Goal tolerance
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)
            self.get_logger().info('Goal reached! Stopping navigation.')
            return

        if self.moving_forward:
            # Adjust orientation to face the goal
            if abs(angle_diff) > 0.1:  # Angle tolerance
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Move forward toward the goal
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0
        elif self.turning:
            # Obstacle avoidance logic
            if self.closest_distance < self.safe_distance and (self.closest_angle < 30 or self.closest_angle > 330):
                twist_msg.linear.x = -0.2  # Reverse slightly
                twist_msg.angular.z = 0.0
                self.get_logger().info(f'Reversing! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')
            else:
                if self.closest_angle < 180:
                    twist_msg.angular.z = 0.5  # Turn left
                    self.get_logger().info(f'Turning left! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')
                else:
                    twist_msg.angular.z = -0.5  # Turn right
                    self.get_logger().info(f'Turning right! Closest obstacle at {self.closest_distance:.2f} meters, angle: {self.closest_angle:.2f}')

        self.current_position = (
            self.current_position[0] + twist_msg.linear.x * math.cos(self.current_angle) * 0.1,
            self.current_position[1] + twist_msg.linear.x * math.sin(self.current_angle) * 0.1
        )
        self.current_angle += twist_msg.angular.z * 0.1
        self.current_angle = (self.current_angle + math.pi) % (2 * math.pi) - math.pi  # Normalise angle

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