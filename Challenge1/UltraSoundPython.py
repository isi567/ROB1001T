# Importing Libraries
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
# Change for your com port, e.g.: "COM8:", "/dev/ttyUSB0", etc.
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

class UltraSoundAvoidance(Node):

    def __init__(self):
        super().__init__('lidar_avoidance')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.safe_distance = 0.5  # Stop if an obstacle is closer than 0.5 meters
        self.moving_forward = True
        self.timer = self.create_timer(0.1, self.my_move_robot)
        self.value_str = ""  # Initialize as an instance variable

    def my_move_robot(self):
        twist_msg = Twist()
        if self.value_str != "ultrasound":  # Use self.value_str here
            twist_msg.linear.x = 0.2  # Stop if an obstacle is too close
            twist_msg.angular.z = 0.0
            self.publisher.publish(twist_msg)

        else:  # If value_str equals "ultrasound"
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            self.get_logger().info('Obstacle detected! Stopping.')
            print("Stopping")
            self.publisher.publish(twist_msg)

    def whileloop(self):
        while True:   
            value_byte = self.Rread()  # read bytes from the Arduino
            try:
                value_str = value_byte.decode("utf-8", errors="ignore").strip()  # Decode and strip whitespace/newlines
                
                if len(value_str) > 0:  # If a message is received
                    self.value_str = value_str  # Update the instance variable
                    if self.value_str == 'RS':
                        print("Arduino was Reset")
                    else:
                        print(self.value_str)
                        self.my_move_robot()  # Adjust robot's movement based on the value
                
            except UnicodeDecodeError as e:
                print(f"Decoding error: {e}")

        
#UltraSoundAvoidance.whileloop()

def main(args=None):
    rclpy.init(args=args)
    node = UltraSoundAvoidance()
    node.whileloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
