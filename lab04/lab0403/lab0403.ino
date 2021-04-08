#include <Servo.h> 
int lastSensorState = LOW;   // sensor's previous state
int esimene_peak = 100;
int threshold = 50;
int currentMillis = 0;
int previousMillis = 0;
int kiirus = 1500;
bool mingi_muutuja = 1;
long rpm = 0;
// an arbitrary threshold value

Servo myservo;


int sensorPin = A1;    // select the input pin for the potentiometer aitah internet
int sensorValue = 0;
  // variable to store the value coming from the sensor
 
void setup() {
  // begin the serial monitor @ 9600 baud
  Serial.begin(9600);
  myservo.attach(5);
  myservo.writeMicroseconds(kiirus);

  
}
 
void loop() {

  
  if (Serial.available() > 0) {
    int a = Serial.parseInt();
    if (a >= 1400 and a <=1600) {
      kiirus  = a;
      Serial.println(kiirus);
    }}


    
  myservo.writeMicroseconds(kiirus);
  currentMillis = millis(); 
  int sensorValue = analogRead(A1);
  if (sensorValue >= threshold and sensorValue < esimene_peak) {
      long revTimeMillis = currentMillis - previousMillis;
      previousMillis = currentMillis;
      rpm= 60*revTimeMillis/1000;
      Serial.print("RPM is ");
      Serial.println(rpm);
      
    }

    delay(20);
  }


 


 
