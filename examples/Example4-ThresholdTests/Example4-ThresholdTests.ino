#include <Wire.h>

#include "SparkFun_AK975X_Arduino_Library.h" //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_AK975X_Arduino_Library

AK975X movementSensor; //Hook object to the library

int ir1, ir2, ir3, ir4, temperature;
int s;
int v;
String sensor;
String value;
bool reading = false;
unsigned long last;
void setup()
{
  Serial.begin(115200);
  Serial.println("AK975X Set Threshold in EEPROM & Read Example");
  pinMode(A3, INPUT);
  
  Wire.begin();

  //Turn on sensor
  if (movementSensor.begin() == false)
  {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }

  Serial.println("AK975X Human Presence Sensor online");
  Serial.println();
  Serial.println("Type : bytenumber ,value ");
  Serial.println("ETH24H : 1 , [-2048,2048]");
  Serial.println("ETH24L : 2 , [-2048,2048]");
  Serial.println("EHY24 : 3 , [0,31]");
  Serial.println("EINTEN : 4 , [0,31]");
  Serial.println("Read INTST : 5");
  Serial.println("Begin readings : 6");
  Serial.println("Stop readings : 7");

  Serial.println();
  
  delay(10);
}

void loop()
{

  if(Serial.available() > 0)  {
    sensor = Serial.readStringUntil(',');
    s= sensor.toInt();
    value = Serial.readStringUntil('\n');
    v = value.toInt();

    switch (s) {
      
      case 1:
      movementSensor.setThresholdIr2Ir4(HIGH,v);
      break;

      case 2:
      movementSensor.setThresholdIr2Ir4(LOW,v);
      break;
      
      case 3:
      movementSensor.setHysteresisIr2Ir4(v);
      break;

      case 4:
      movementSensor.setInterrupts(bitRead(v,4),bitRead(v,3),bitRead(v,2),bitRead(v,1),bitRead(v,0));
      break;

      case 5:
      Serial.print("Interrupt Status: ");
      Serial.println(movementSensor.readInterruptStatus());
      break;

      case 6:
      reading= true;
      break;
      
      case 7:
      reading= false;
      break;    

      default:
      break;
    }
  }

  if (reading) readSensors();

  delay(1);
}

void readSensors()
{
    int interrupt = analogRead(A3);
    if (interrupt<600) {

      if (movementSensor.available())  {
      ir2 = movementSensor.getIR2();
      ir4 = movementSensor.getIR4();
       movementSensor.refresh(); //Read dummy register after new data is read
        
        int i = movementSensor.readInterruptStatus();
        Serial.print("ir2-ir4 : ");
        Serial.println(ir2-ir4);
        Serial.print("time : ");
        Serial.println(millis()-last);
        last=millis();
        Serial.println();
      }
      // else error checking
    }

  }

