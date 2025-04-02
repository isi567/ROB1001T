# Importing Libraries
import serial
import tkinter as tk
from tkinter import messagebox
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import threading
# Change for your com port, e.g.: "COM8:", "/dev/ttyUSB0", etc.
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

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
                if value_str == "ultrasound":
                    twist_msg.linear.x = 0.0  # Stop if an obstacle is too close
                    twist_msg.angular.z = 0.0
                    self.get_logger().info('Obstacle detected! Stopping.')
                    print("Stopping")
                    self.publisher.publish(twist_msg)

    except UnicodeDecodeError as e:
        print(f"Decoding error: {e}")

arduino.close()
