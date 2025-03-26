import serial

arduino = serial.Serial(port = 'COM3', baudrate=115200, timeout=.1)

def read(): 
    data = arduino.readline() 
    return data 

while ( True ):
    value_byte = read()  # read bytes from the Arduino
    value_str= value_byte.decode("utf-8") #from byte to string
    if ( len( value_str ) != 0 ): # is there any message received?
        print( "Number of times pressed: ", value_str )

arduino.close()