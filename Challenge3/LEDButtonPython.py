# Importing Libraries
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

# Change for your com port, e.g.: "COM8:", "/dev/ttyUSB0", etc.
arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

class LEDButton(Node):
    
    def __init__(self):
            super().__init__('LEDButton')
            self.publisher = self.create_publisher(Bool, '/button', 10)
            #self.subscription = self.create_subscription(Bool, '/moving', 10)
            #self.timer = self.create_timer(0.1, self.my_move_robot)
            self.value_str = ""
            self.value_byte = ""
    
    #################################################################################################################################
    def Rread(self):
        data = arduino.readline()
        return data
        
    def whileloop(self):
        while ( True ):   
            self.value_byte = self.Rread()  # read bytes from the Arduino
            try:
                self.value_str = self.value_byte.decode("utf-8", errors="ignore")  # from byte to string, ignoring errors
                if len(self.value_str) != 0:  # is there any message received?
                    if self.value_str.strip() == 'RS':  # strip whitespace/newlines
                        print("Arduino was Reset")
                    else:
                        print(self.value_str) 
                        #publish to button topic
                        buttonStatus = Bool()
                        buttonStatus.data = True 
                        self.publisher_.publish(buttonStatus)
                        self.get_logger().info(f'Publishing: {buttonStatus.data}')
                        # Toggle the value for the next message
                        buttonStatus.data = not buttonStatus.data
        
            except UnicodeDecodeError as e:
                print(f"Decoding error: {e}")
        
        arduino.close()


def main(args=None):
    rclpy.init(args=args)
    node = LEDButton()
    node.whileloop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
