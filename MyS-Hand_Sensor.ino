// Ultrasonic distance sensor used to control several lights with hand without touch
// Lights are controlled based on the distance between sensor module and use hand
// Ultrasonic sensor: HC-SR04

//#define MY_DEBUG
#define MY_RADIO_NRF24
#define MY_NODE_ID 19
#include <SPI.h>
#include <MySensor.h>  

#define CHILD_ID 1
MyMessage msg(CHILD_ID, V_DISTANCE); // 13
byte trig = 7; // TRIG pin of sensor
byte echo = 8; // ECHO pin of sensor

int dist, lastDist; // keep track of actual and previous distance measure
unsigned long tupdate = 1000; // interval between measurements in ms

void setup()  
{ 
  pinMode(trig, OUTPUT);
  pinMode(echo,INPUT);
  dist=0;
  lastDist=0;
}

void presentation() {
  sendSketchInfo("Distance Sensor", "1.0");
  present(CHILD_ID, S_DISTANCE);
}

void loop()      
{     
  dist=distance();
  // Enable while testing, if needed:
  //Serial.println(dist);
  if (dist != lastDist) { // if measured distance is modified, send it to the controller
      send(msg.set(dist));
      lastDist = dist;
  }

  delay(tupdate);
}

int distance() {
  // distance measurement in cm
  long duration;
  int cm;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(5);
  digitalWrite(trig, LOW);
  duration = pulseIn(echo, HIGH);
  cm = microsecondsToCentimeters(duration);
  if (cm>300) cm=300;
  return cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}
