
# Importing Libraries
import serial
# Change for your com port, e.g.: "COM8:", "/dev/ttyUSB0", etc.
arduino = serial.Serial(port='COM3', baudrate=115200, timeout=.1)

def read():
    data = arduino.readline()
    return data

while ( True ):
    value_byte = read()  # read bytes from the Arduino
    value_str= value_byte.decode("utf-8") #from byte to string
    if ( len( value_str ) != 0 ): # is there any message received?
        if ( value_str == 'RS' ):
            print( "Arduino was Reset" )
        else:
            print( value_str )

arduino.close()
