// This MQTT gateway works on Arduino UNO

// Enable debug prints to serial monitor
// #define MY_DEBUG 

// Enables and select radio type (if attached)
#define MY_RADIO_NRF24

#define MY_GATEWAY_MQTT_CLIENT

// Set this nodes subscripe and publish topic prefix, keep it as short as possible
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "P"
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "S"

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "M"

#define MY_GATEWAY_W5100
#define MY_MAC_ADDRESS 0xDE, 0xED, 0xBE, 0xEF, 0xFE, 0xED

// W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4  

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with 
// radio on the same spi bus.
// The PINs mentioned here are for Arduino Mega2560 ! Color code is from MySensor site
#if !defined(MY_W5100_SPI_EN)
  #define MY_SOFTSPI
  #define MY_SOFT_SPI_SCK_PIN 14 // A0, Green
  #define MY_SOFT_SPI_MISO_PIN 16  // A2, Violet
  #define MY_SOFT_SPI_MOSI_PIN 15  // A1, Blue
#endif  

// When W5100 is connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN 5  // Orange
#define MY_RF24_CS_PIN 6  // Yellow

// Enable these if your MQTT broker requires usenrame/password
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"

// If you use static IP, uncomment the following lines
// If you define the Gateway IP in the router and use DHCP, no modification is needed
// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
//#define MY_IP_ADDRESS 192,168,1,101

// If using static ip you need to define Gateway and Subnet address as well
//#define MY_IP_GATEWAY_ADDRESS 192,168,1,1
//#define MY_IP_SUBNET_ADDRESS 255,255,255,0


// MQTT broker ip address, usually the IP of the server where the controller is running
#define MY_CONTROLLER_IP_ADDRESS 192,168,1,100

// The MQTT broker port to to open 
#define MY_PORT 1883      

#include <Ethernet.h>
#include <SPI.h>
#include <MySensor.h>

#define CHILD_ID_REL1 100 // Relay1
#define CHILD_ID_REL2 200 // Relay2 - used for WebCam
#define CHILD_ID_PIR 112  // Motion Sensor PIR
#define CHILD_ID_GAS 113  // Gas Sensor MQ-2

/*
Ethernet use the following PINs, do not use them:
UNO:  4, 10, 11, 12, 13
*/
#define PIR_PIN 2   // Motion sensor
#define MQ_PIN A5   // MQ-2 gas sensor
#define REL1_PIN 7  // Relay1
#define REL2_PIN 8  // Relay2 - WebCam

MyMessage msgPir(CHILD_ID_PIR, V_TRIPPED); // 16
MyMessage msgGas(CHILD_ID_GAS, V_VAR1); //24
MyMessage msgREL1(CHILD_ID_REL1, V_LIGHT);  // 2
MyMessage msgREL2(CHILD_ID_REL2, V_LIGHT);  // 2

const unsigned long tUpdate=60000; //update interval
unsigned long t0;
byte StatePIR=0;
byte oldStatePIR=0;
int InAnalog1;
int valMQ=0;
int refMQ=1023; // refference value will be the lowest value read from the gas sensor
bool StateREL=0, StateREL1=0, StateREL2=0;

void setup() { 
  pinMode(PIR_PIN, INPUT_PULLUP);
  pinMode(REL1_PIN, OUTPUT);
  digitalWrite(REL1_PIN, StateREL1);
  pinMode(REL2_PIN, OUTPUT);
  digitalWrite(REL2_PIN, StateREL2);
  t0=millis();
}

void presentation() {
  present(CHILD_ID_PIR, S_MOTION);
  present(CHILD_ID_GAS, S_CUSTOM);
  present(CHILD_ID_REL1, S_LIGHT);
  present(CHILD_ID_REL2, S_LIGHT);
}

void loop() {
  StatePIR=digitalRead(PIR_PIN);
  if (StatePIR != oldStatePIR) {
    oldStatePIR=StatePIR;
    send(msgPir.set(StatePIR ? "ON" : "OFF"));
  }
  if ((millis()-t0) > tUpdate) ServerUpdate();
}

void ServerUpdate() {
  InAnalog1=analogRead(MQ_PIN);
  if (InAnalog1<refMQ) refMQ=InAnalog1; // if read value is lower that actual, change the 0 refference with the actual one
  valMQ=InAnalog1-refMQ; // calculate the sensor measurement from the refference
  send(msgGas.set(valMQ));
  
  t0=millis();
}

void receive(const MyMessage &message) {
  // We only expect one type of message from controller. But we better check anyway.
  Serial.print("Node:");Serial.println(message.destination);
  if (message.type==V_LIGHT) {
    StateREL=strcmp(message.data,"ON")==0;
    switch(message.sensor) {
      case 100: {
        StateREL1=StateREL;
        send(msgREL1.set(StateREL1 ? "ON":"OFF"));
        if (StateREL) {
          digitalWrite(REL1_PIN, HIGH);
        }
        else {
          digitalWrite(REL1_PIN, LOW);
        }
      }
      break;
      case 200: {
        StateREL2=StateREL;
        send(msgREL2.set(StateREL2 ? "ON":"OFF"));
        if (StateREL) {
          digitalWrite(REL2_PIN, HIGH);
        }
        else {
          digitalWrite(REL2_PIN, LOW);
        }
      }
      break;
    }

   } 
}

