const int trigPin = 12;
const int echoPin = 11;
const int buttonPin = 5;  // the pin that the pushbutton is attached to
const int ledPin = 4;    // the pin that the LED is attached to
// Variables will change:
int buttonState = 0;        // current state of the button
int lastButtonState = 0;    // previous state of the button
bool buttonIsOn = false;
bool ultraIsOn = false;


float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buttonPin, INPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin( 115200 );  // fast
  Serial.println( "RS" );    
}

void loop() {
  digitalWrite(ledPin, LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;
  if (distance<10){ultraIsOn = true;
  Serial.write("ultrasound");}
  else {ultraIsOn = false;
  }
  delay(100);

  // read the pushbutton input pin:
  buttonState = digitalRead(buttonPin);

  // compare the buttonState to its previous state
  if (buttonState != lastButtonState) {
    // if the state has changed, increment the counter
    if (buttonState == HIGH) {
      buttonIsOn = true;
      Serial.write("button");
      // if the current state is HIGH then the button went from off to on:
    } else {
      // if the current state is LOW then the button went from on to off:
      buttonIsOn = false;
    }
    // Delay a little bit to avoid bouncing
    delay(50);
  }
  // save the current state as the last state, for next time through the loop
  lastButtonState = buttonState;

  if ((ultraIsOn) && (buttonIsOn)){
    Serial.write("Both");
    digitalWrite(ledPin, HIGH);
  }

}
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
