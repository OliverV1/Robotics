#include <Servo.h>

Servo myservo;

int pos = 0;  

void setup() {
  myservo.attach(9);
  Serial.begin(9600);
  pinMode(2,INPUT);
  pinMode(3,OUTPUT);
}


int echoPin = 2;
int trigPin = 3;
int delay_us = 10; // <--- YOU HAVE TO FIND THE CORRECT VALUE FROM THE DATASHEET
long distance_mm = 0;
long duration_us;

void check() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(delay_us);
  digitalWrite(trigPin, LOW);
  duration_us = pulseIn(echoPin, HIGH);
  distance_mm = duration_us / 5.8;
  Serial.println(distance_mm);
}
void loop() {
  for (pos = 0; pos <= 180;) {
    check();
    if (distance_mm < 300) {
      Serial.println(distance_mm);
    }
    else {
      myservo.write(pos+=1);
      delay(3);
    }
  }
  for (pos = 180; pos >= 0;) {
    check();
    if (distance_mm < 300) {
      Serial.println(distance_mm);
    }
    else {
      myservo.write(pos-=1);
      delay(3);
    }
  }
}
