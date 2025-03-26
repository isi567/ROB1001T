/*used:
 * HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */

const int trigPin = 9;
const int echoPin = 10;
int ultra_count = 0;
int count = 0;
int button = 6;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(button, INPUT);
  Serial.begin(115200);
}

void loop() {
  //button
  if (digitalRead(button) == HIGH){
    count = count + 1;
  }
  Serial.print(count);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  duration = pulseIn(echoPin, HIGH);
  distance = (duration*.0343)/2;

//ultrasounds
  if (distance > 10){
    ultra_count = ultra_count+1;
  }
  Serial.print(ultra_count)


  if (distance > 10) & (digitalRead(button) == HIGH){
    Serial.print("Both")
  }
  delay(100);
}