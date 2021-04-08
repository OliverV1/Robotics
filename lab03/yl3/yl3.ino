#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards

int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
}

void loop() {
    bool clockwise = false;
    myservo.write(0);              // tell servo to go to position in variable 'pos'
    delay(1000);
    myservo.write(30);              // tell servo to go to position in variable 'pos'
    delay(1000);
    myservo.write(80);              // tell servo to go to position in variable 'pos'
    delay(1000);
    myservo.write(150);              // tell servo to go to position in variable 'pos'
    delay(1000); 
    myservo.write(180);              // tell servo to go to position in variable 'pos'
    delay(1000);
  for (pos = 180; pos >= 0; pos -= 1) {
    bool clockwise = true;// goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
