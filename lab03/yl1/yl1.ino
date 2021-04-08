void setup()  {
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    int incomingNumber = Serial.parseInt();
    if (incomingNumber > 300) {
      Serial.println("Go");
    }
    else{
      Serial.println("Stop");
    }
  }
}
