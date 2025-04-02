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
            self.timer = self.create_timer(0.1, self.move_robot)
    
    def lidar_callback(self, msg):
        if not msg.ranges:
            return
    
        closest_distance = min([r for r in msg.ranges if r > 0])
        self.moving_forward = closest_distance > self.safe_distance

    def my_move_robot(self):
        twist_msg = Twist()
                while value_str != "ultrasound":
                    twist_msg.linear.x = 0.2  # Stop if an obstacle is too close
                    twist_msg.angular.z = 0.0
                    self.publisher.publish(twist_msg)

                #if it does equal ultrasound
                twist_msg.linear.x = 0.0  # Stop if an obstacle is too close
                twist_msg.angular.z = 0.0
                self.get_logger().info('Obstacle detected! Stopping.')
                print("Stopping")
                self.publisher.publish(twist_msg)
    
        #if not self.moving_forward:
            #self.root.after(0, self.show_warning)
    
    #################################################################################################################################
    def read():
        data = arduino.readline()
        return data
    
    while ( True ):   
        value_byte = read()  # read bytes from the Arduino
        try:
            value_str = value_byte.decode("utf-8", errors="ignore")  # from byte to string, ignoring errors
            if len(value_str) != 0:  # is there any message received?
                if value_str.strip() == 'RS':  # strip whitespace/newlines
                    print("Arduino was Reset")
                else:
                    print(value_str) 
                    my_move_robot()
    
        except UnicodeDecodeError as e:
            print(f"Decoding error: {e}")
    
    arduino.close()


def main(args=None):
    rclpy.init(args=args)
    node = UltraSoundAvoidance()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
