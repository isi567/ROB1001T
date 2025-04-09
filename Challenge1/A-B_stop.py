#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class LidarNavigation(Node):
    def __init__(self):
        super().__init__('lidar_navigation')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.safe_distance = 0.2  # Stop if an obstacle is closer than 0.2 meters
        self.goal_position = (2.0, 0.0)  # Target position (x, y) in meters
        self.current_position = (0.0, 0.0)  # Current position (x, y)
        self.current_angle = 0.0  # Current orientation in radians
        self.moving_forward = True
        self.timer = self.create_timer(0.1, self.navigate_to_goal)


    def lidar_callback(self, msg):


        closest_distance = min([r for r in msg.ranges if r > 0])
        self.moving_forward = closest_distance > self.safe_distance




    def navigate_to_goal(self):
        # Create a Twist message
        twist_msg = Twist()

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
            self.destroy_node()  # Stop the ROS node
            rclpy.shutdown()  # Shutdown ROS
            return
        
        elif self.moving_forward:
            # Adjust orientation to face the goal
            if abs(angle_diff) > 0.1:  # Angle tolerance
                twist_msg.linear.x = 0.0
                twist_msg.angular.z = 0.5 if angle_diff > 0 else -0.5
            else:
                # Move forward toward the goal
                twist_msg.linear.x = 0.2
                twist_msg.angular.z = 0.0
        else:
            # Stop if an obstacle is too close
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Obstacle detected! Stopping.')

        self.publisher.publish(twist_msg)

        # Update the robot's position (this is a placeholder; in a real robot, you'd use odometry data)
        self.current_position = (
            self.current_position[0] + twist_msg.linear.x * math.cos(self.current_angle) * 0.1,
            self.current_position[1] + twist_msg.linear.x * math.sin(self.current_angle) * 0.1
        )
        self.current_angle += twist_msg.angular.z * 0.1
        self.current_angle = (self.current_angle + math.pi) % (2 * math.pi) - math.pi  # Normalize angle

def main(args=None):
    rclpy.init(args=args)
    lidar_navigation = LidarNavigation()



    try:
        rclpy.spin(lidar_navigation)
    except KeyboardInterrupt:
        lidar_navigation.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        lidar_navigation.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()