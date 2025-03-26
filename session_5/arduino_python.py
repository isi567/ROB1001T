import serial
import time

# Set up the serial connection (adjust the port and baud rate as needed)
arduino_port = '/dev/ttyACM0'  # Replace with your Arduino's port
baud_rate = 115200
timeout = 1

try:
    ser = serial.Serial(arduino_port, baud_rate, timeout=timeout)
    print(f"Connected to Arduino on port {arduino_port}")
    time.sleep(2)  # Wait for the connection to initialize
except serial.SerialException as e:
    print(f"Error connecting to Arduino: {e}")
    exit()

def send_command(command):
    """Send a command to the Arduino."""
    ser.write(command.encode())
    print(f"Sent: {command}")

def read_data():
    """Read data from the Arduino."""
    if ser.in_waiting > 0:
        try:
            data = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"Received: {data}")
            return data
        except UnicodeDecodeError as e:
            print(f"Decoding error: {e}")
            return None
    return None

try:
    while True:
        # Read data from Arduino
        data = read_data()
        if data:
            if data == "ultrasound":
                print("Ultrasound sensor detected an object within 10 cm!")

        # Example: Send a command to Arduino (e.g., toggle LED)
        user_input = input("Enter command to send to Arduino (or 'exit' to quit): ")
        if user_input.lower() == 'exit':
            break
        send_command(user_input)

except KeyboardInterrupt:
    print("\nExiting program.")

finally:
    ser.close()
    print("Serial connection closed.")