/*
  AK975X Human Presence and Movement Sensor Example Code
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 10th, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Demonstrates how to configure advanced aspects of the sensor such as I2C address and speed,
  and different Wire streams.

  Data overrun is printed. Increase serial speed to 115200 to read fast enough.

  IR readings increase as warm bodies enter into the observable areas.

  Hardware Connections:
  Attach a Qwiic shield to your RedBoard or Uno.
  Plug two Qwiic GPS module to any port.
  Serial.print it out at 115200 baud to serial monitor.

  Available functions:
  int getIR1,2,3,4()
  void refresh() - Must be called to trigger next read
  void setMode(byte) - See defines
  void setCutoffFrequency(byte) - See defines
  bool available()
  bool overrun()
  float getTemperature()
  float getTemperatureF()
  void softReset()
*/

#include <Wire.h>

#include "SparkFun_AK975X_Arduino_Library.h" //Use Library Manager or download here: https://github.com/sparkfun/SparkFun_AK975X_Arduino_Library

AK975X movementSensor; //Hook object to the library

int ir1, ir2, ir3, ir4, temperature;

void setup()
{ 
  long baud = 115200;
  Serial.begin(baud);
  Serial.println("AK975X Read Example");

  Wire.begin();

  //Turn on sensor
  //Here we can pass it different Wire streams such as Wire1 or Wire2 used on Teensys
  //We can select FAST or STANDARD I2C speed
  //If you set the address jumpers, this is where you pass the address of the sensor you want to talk to
  if (movementSensor.begin(Wire, I2C_SPEED_FAST, 0x64) == false)
  {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }

  movementSensor.enableDebugging(); //Print extra debug messages if needed. Defaults to Serial.
  //movementSensor.enableDebugging(SerialUSB); //Use can pass other serial ports if desired.
  //There is also a disableDebugging();
}

void loop()
{
  if (movementSensor.available())
  {    
    ir1 = movementSensor.getIR1();
    ir2 = movementSensor.getIR2();
    ir3 = movementSensor.getIR3();
    ir4 = movementSensor.getIR4();
    float tempF = movementSensor.getTemperatureF();

    movementSensor.refresh(); //Read dummy register after new data is read

    //Note: The observable area is shown in the silkscreen.
    //If sensor 2 increases first, the human is on the left
    Serial.print("1:DWN[");
    Serial.print(ir1);
    Serial.print("]\t2:LFT[");
    Serial.print(ir2);
    Serial.print("]\t3:UP[");
    Serial.print(ir3);
    Serial.print("]\t4:RGH[");
    Serial.print(ir4);
    Serial.print("]\ttempF[");
    Serial.print(tempF);
    Serial.print("]\tmillis[");
    Serial.print(millis());
    Serial.print("]");
    Serial.println();
  }
  if(movementSensor.overrun() == true)
    {
      //At 9600bps printing takes longer than the IC can read. Increase serial to 115200bps to increase
      //read rate fast enough.
      Serial.println("Data overrun!");
    }
  delay(1);
}
