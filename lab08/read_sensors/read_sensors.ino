// Ultrasonic sensor
int echoPin = A4;
int trigPin = A5;

// Line sensor
int ls1 = 2;
int ls2 = 3;
int ls3 = 4;
int ls4 = 5;
int ls5 = 6;
int clp = 7;
int near = 8;

// Anything over 400 cm (23200 us pulse) is "out of range"
const unsigned int MAX_ECHO_US = 23200;
const unsigned int MAX_DIST_MM = 4000;

const int SONAR_DELAY_US = 20;
int us1 = MAX_DIST_MM;

void setup() {
  Serial.begin(115200);
  // Set pin directions for the ultrasonic sensor
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  // Set pin directions for the line sensor
  pinMode(ls1, INPUT);
  pinMode(ls2, INPUT);
  pinMode(ls3, INPUT);
  pinMode(ls4, INPUT);
  pinMode(ls5, INPUT);
  pinMode(clp, INPUT);
  pinMode(near, INPUT);
}

void loop()
{
  // Wait until it is signalled that new data is needed
  while(!Serial.available()); 

  // Read everything from serial
  while(Serial.available())   
  {
    Serial.read(); 
  }

  // Get distance from wall with ultrasonic sensor
  us1 = getUS1();   

  // Print data to serial.
  printJSON(us1);   
}

// Gets distance in mm from the ultrasonic sensor
long getUS1(){
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(SONAR_DELAY_US);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, MAX_ECHO_US);
  return (duration/2) / 2.91; // get distance in mm
}

// Print all the sensor data to serial as JSON
void printJSON(int us1) 
{ 
  Serial.print("{\"us1\":");
  Serial.print(us1);
  Serial.print(", \"ls1\":");
  Serial.print(digitalRead(ls1));
  Serial.print(", \"ls2\":");
  Serial.print(digitalRead(ls2));
  Serial.print(", \"ls3\":");
  Serial.print(digitalRead(ls3));
  Serial.print(", \"ls4\":");
  Serial.print(digitalRead(ls4));
  Serial.print(", \"ls5\":");
  Serial.print(digitalRead(ls5));
  Serial.print(", \"clp\":");
  Serial.print(digitalRead(clp));
  Serial.print(", \"near\":");
  Serial.print(digitalRead(near));
  Serial.println("}");
}
