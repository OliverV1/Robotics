const int pirPin = 5;         // passive infrared sensor pin
int state = 0;
int is_first_movement = 0;
int ledState = LOW;
const int ledPin =  LED_BUILTIN;
unsigned long previousMillis = 0;
const long interval = 1000; 
// 0 - no motion, 1 - motion detected

void setup(){
  Serial.begin(9600);         // initialize serial with 9600 baud rate
  pinMode(pirPin, INPUT);
  pinMode(ledPin, OUTPUT);    // set pin #5 as an input from PIR
}

void loop(){
  digitalWrite(ledPin, ledState);
  if(digitalRead(pirPin) == HIGH  &&  state == 0 &&  is_first_movement == 0){
    Serial.println("Motion detected!");
    state = 1;
    is_first_movement = 1;

  }
    else if(digitalRead(pirPin) == HIGH  &&  is_first_movement == 1 &&  state == 0 ){
    ledState = LOW;
    Serial.println("MOTION DEDECTED AGAIN");
    state = 1;
    is_first_movement = 0;    
   }
    
  if(is_first_movement == 1){
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        if (ledState == LOW) {
          ledState = HIGH;
        } else {
        ledState = LOW;
      }
     }
  }

  if(digitalRead(pirPin) == LOW  &&  state == 1 ){
   //ledState = LOW;
    Serial.println("No movement any more");
    state = 0;
  }

}
