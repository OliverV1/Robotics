#include <Servo.h> 

Servo myservo;

const int pirPin = 6;         // passive infrared sensor pin
int state = 0;
int previous_work = 0;
int ledState = LOW;
int kiirus = 1500;
const int ledPin =  LED_BUILTIN;
unsigned long previousMillis = 0;
const long interval = 1000; 
// 0 - no motion, 1 - motion detected

void setup(){
  Serial.begin(9600);         // initialize serial with 9600 baud rate
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);
  myservo.attach(5);
  myservo.writeMicroseconds(1500);// set pin #5 as an input from PIR
  
  
}

void loop(){
  if (Serial.available() > 0) {
    int a = Serial.parseInt();
    if (a >= 1400 and a <=1600) {
      kiirus  = a;
      Serial.println(kiirus);
    }}

    
  if(digitalRead(pirPin) == HIGH  &&  state == 0 &&  previous_work == 0){
    myservo.writeMicroseconds(kiirus);
    Serial.println("Turning on!");
    state = 1;
    previous_work = 1;
    
    
    }
   
    
  else if(digitalRead(pirPin) == HIGH  &&  state == 0 &&  previous_work == 1){
  Serial.println("Turning off!");
    myservo.writeMicroseconds(1500);
    state = 1;
    previous_work = 0;
  }

  
  if(digitalRead(pirPin) == LOW  &&  state == 1 ){
    Serial.println("No movement any more");
    state = 0;
  }


}


  
