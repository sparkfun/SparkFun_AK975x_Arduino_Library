/*
  AK9752 Human Presence and Movement Sensor Example Code
  By: Nathan Seidle
  SparkFun Electronics
  Date: March 10th, 2017
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  This code attempts to calculate derivatives on a single IR channel to see if something has entered
  or exited the view of the sensor. 

  Open the Serial plotter at 115200 to see the output. Requires Arduino v1.8.1 or above.
  
  There are probably better mathematical approaches to filtering and detecting motion events. This is
  just an example of how you might approach it.

  The basis for this code is based on the code from Robotic Materials and their Robotic Finger Sensor.
  Check it out, it's pretty neat: https://www.sparkfun.com/products/14200

  Hardware Connections:
  Attach a Qwiic shield to your RedBoard or Uno.
  Plug the Qwiic sensor into any port.
  Serial.print or plotter at 115200bps

*/

#include <Wire.h>

#include "SparkFun_AK9750_Arduino_Library.h"

AK9750 movementSensor; //Hook object to the library

unsigned int upValue; // current proximity reading
unsigned int averageValue;   // low-pass filtered proximity reading
signed int fa2;              // FA-II value;
signed int fa2Derivative;     // Derivative of the FA-II value;
signed int fa2DerivativeLast;     // Last value of the derivative (for zero-crossing detection)
signed int sensitivity = 50;  // Sensitivity of touch/release detection, values closer to zero increase sensitivity

#define LOOP_TIME 30  // Loop duration in ms. 30ms works well.

//Exponential average weight parameter / cut-off frequency for high-pass filter
//#define EA 0.3  //Very steep
//#define EA 0.1  //Less steep
#define EA 0.05  //Less steep

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  //Turn on sensor
  if (movementSensor.begin() == false)
  {
    Serial.println("Device not found. Check wiring.");
    while (1);
  }

  upValue = movementSensor.getIR3(); //Get one of the latest IR values
  averageValue = upValue;
  fa2 = 0;
  movementSensor.refresh(); //Read dummy register after new data is read

  Serial.println("AK9750 Human Presence Sensor online");
}

void loop()
{
  unsigned long startTime = millis();

  while (movementSensor.available() == false) delay(1); //Wait for new data

  upValue = movementSensor.getIR3(); //Get one of the latest IR values
  movementSensor.refresh(); //Read dummy register after new data is read

  fa2DerivativeLast = fa2Derivative;
  fa2Derivative = (signed int) averageValue - upValue - fa2;
  fa2 = (signed int) averageValue - upValue;

  //Turn on various variables to see how they respond on the graph
  //Serial.print(upValue);
  //Serial.print(",");
  //Serial.print(fa2);
  //Serial.print(",");
  Serial.print(fa2Derivative);
  Serial.print(",");

  //Look to see if the previous fa2Der was below threshold AND current fa2Der is above threshold
  //Basically, if the sign of the fa2Ders has switched since last reading then we have an event
  if ((fa2DerivativeLast < -sensitivity && fa2Derivative > sensitivity) || (fa2DerivativeLast > sensitivity && fa2Derivative < -sensitivity)) // zero crossing detected
  {
    if (fa2 < -sensitivity) // minimum
    {
      Serial.print(-1000); //Cause red line to indicate entering presence
      Serial.print(",");
      //Serial.print("Entered view");
    }
    else if (fa2 > sensitivity) // maximum
    {
      Serial.print(1000); //Cause red line to indicate exiting presence
      Serial.print(",");
      //Serial.print("Exited view");
    }
    else
    {
      Serial.print(0); //Cause red line to indicate no movement
      Serial.print(",");
      //Serial.print("No Movement");
    }
  }
  else
  {
    Serial.print(0);
    Serial.print(",");
  }

  Serial.println();

  // Do this last
  averageValue = EA * upValue + (1 - EA) * averageValue;
  while (millis() < startTime + LOOP_TIME); // enforce constant loop time
}
