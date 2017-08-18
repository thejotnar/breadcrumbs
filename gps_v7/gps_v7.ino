#include <SD.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include "Adafruit_Thermal.h"

#define TX_PIN 2 
#define RX_PIN 10 

SoftwareSerial mySerial (3, 2);
SoftwareSerial myPrinter(RX_PIN, TX_PIN); 
Adafruit_Thermal printer(&myPrinter);


Adafruit_GPS GPS(&mySerial);
#define GPSECHO true
boolean usingInterrupt = false;
void useInterrupt(boolean);

const int chipSelect = 5;
int writeButton = 7;

void setup() {
  pinMode(writeButton, INPUT);

 // SD.begin(chipSelect);
  Serial.begin(9600);
  analogReference(EXTERNAL);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);

  Serial.print("Initializing SD Card...");
  pinMode(chipSelect, OUTPUT);

  if (!SD.begin(chipSelect)) {
    Serial.println("Card initializing failed.");
    return;
  }
    Serial.println("Card initialized.");
}

SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();

void loop() {
  int buttonState = digitalRead(writeButton);

  String dataString = "";
  File dataFile = SD.open("GPSdata.txt", FILE_WRITE);

    if (! usingInterrupt) {
    char c = GPS.read();
    if (GPSECHO);
     // if (c) Serial.print(c);
  }
  
  if (GPS.newNMEAreceived()) {
  
    if (!GPS.parse(GPS.lastNMEA()))   
      return;  
  }

  if (timer > millis())  timer = millis();

  if (millis() - timer > 2000) { 
    timer = millis(); 

  if (GPS.fix == true) {
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
  } else {
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);

  }

   if (GPS.fix == true && buttonState == 1){
    printer.println("greetings from...");
printer.println("");
printer.println(GPS.latitudeDegrees, 4);
printer.print("");
printer.println(GPS.longitudeDegrees, 4);
printer.print("");
printer.println("4/5/17");
   }
  printer.feed(4);  

  printer.sleep();      
  delay(3000L);         
  printer.wake();       
  printer.setDefault();
}
}

