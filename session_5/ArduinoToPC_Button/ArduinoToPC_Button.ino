// Send a message from Arduino to the PC (read by a python script). The message is 
//sent every time a push button is pressed. The message (str type) contains the 
//number of times the button has been pressed and must be printed on a terminal.
int count = 0;
int button = 6;
void setup() {
  // put your setup code here, to run once:
  Serial.begin( 115200 );
  pinMode(button, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  //if button pressed, send message to pc
  if (digitalRead(button) == HIGH){
    count = count + 1;
  }
  Serial.print(count);
}
