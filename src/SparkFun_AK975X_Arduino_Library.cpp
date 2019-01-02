/*
  This is a library written for the AK975X Human Presence Sensor.

  Written by Nathan Seidle @ SparkFun Electronics, March 10th, 2017

  The sensor uses I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  https://github.com/sparkfun/SparkFun_AK975X_Arduino_Library

  Do you like this library? Help support SparkFun. Buy a board!

  Development environment specifics:
  Arduino IDE 1.8.1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_AK975X_Arduino_Library.h"

//Sets up the sensor for constant read
//Returns false if sensor does not respond
boolean AK975X::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{
  //Bring in the user's choices
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  uint8_t deviceID = readRegister(AK975X_WIA2);
  if (deviceID != 0x13) //Device ID should be 0x13
    return (false);

  setMode(AK975X_MODE_0); //Set to continuous read

  setCutoffFrequency(AK975X_FREQ_8_8HZ); //Set output to fastest, with least filtering

  refresh(); //Read dummy register after new data is read

  return (true); //Success!
}

boolean AK975X::reboot()
{

  setMode(AK975X_MODE_0); //Set to continuous read

  setCutoffFrequency(AK975X_FREQ_8_8HZ); //Set output to fastest, with least filtering

  refresh(); //Read dummy register after new data is read

  return (true); //Success!
}

//Returns the decimal value of sensor channel
int16_t AK975X::getIR1()
{
  return (readRegister16(AK975X_IR1));
}
int16_t AK975X::getIR2()
{
  return (readRegister16(AK975X_IR2));
}
int16_t AK975X::getIR3()
{
  return (readRegister16(AK975X_IR3));
}
int16_t AK975X::getIR4()
{
  return (readRegister16(AK975X_IR4));
}

//This reads the dummy register ST2. Required after
//reading out IR data.
void AK975X::refresh()
{
  uint8_t dummy = readRegister(AK975X_ST2); //Do nothing but read the register
}

//Returns the temperature in C
float AK975X::getTemperature()
{
  int16_t value = readRegister16(AK975X_TMP);

  value >>= 6; //Temp is 10-bit. TMPL0:5 fixed at 0

  float temperature = 26.75 + (value * 0.125);
  return (temperature);
}

//Returns the temperature in F
float AK975X::getTemperatureF()
{
  float temperature = getTemperature();
  temperature = temperature * 1.8 + 32.0;
  return (temperature);
}

//Set the mode. Continuous mode 0 is favored
void AK975X::setMode(uint8_t mode)
{
  if (mode > AK975X_MODE_3) mode = AK975X_MODE_0; //Default to mode 0
  if (mode == 0b011) mode = AK975X_MODE_0; //0x03 is prohibited

  //Read, mask set, write
  byte currentSettings = readRegister(AK975X_ECNTL1);

  currentSettings &= 0b11111000; //Clear Mode bits
  currentSettings |= mode;
  writeRegister(AK975X_ECNTL1, currentSettings);
}

//Set the cutoff frequency. See datasheet for allowable settings.
void AK975X::setCutoffFrequency(uint8_t frequency)
{
  if (frequency > 0b101) frequency = 0; //Default to 0.3Hz

  //Read, mask set, write
  byte currentSettings = readRegister(AK975X_ECNTL1);

  currentSettings &= 0b11000111; //Clear EFC bits
  currentSettings |= (frequency << 3); //Mask in
  writeRegister(AK975X_ECNTL1, currentSettings); //Write
}

void AK975X::setThresholdIr2Ir4(bool weight, int v) {

  int k=v >>5; //high bits [11;5]
  int b=v <<3; // low bits
  b &= 0b11111000; // low bits
  k &= 0b01111111; // high bits
  byte currentSettings;

  if (weight) {

    writeRegister(AK975X_ETH24H_LOW, b);
    writeRegister(AK975X_ETH24H_HIGH, k);

    currentSettings=readRegister(AK975X_ETH24H_LOW);
    //Serial.print("AK975X_ETH24H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_ETH24H_HIGH);
    //Serial.print("AK975X_ETH24H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    writeRegister(AK975X_ETH24L_LOW, b);
    writeRegister(AK975X_ETH24L_HIGH, k);

    currentSettings=readRegister(AK975X_ETH24L_LOW);
    //Serial.print("AK975X_ETH24L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_ETH24L_HIGH);
    //Serial.print("AK975X_ETH24L_HIGH : ");
    //Serial.println(currentSettings,BIN);
  }
}

void AK975X::setThresholdEepromIr2Ir4(bool weight, int v) {

  int h=v >>6; //high bits [11;6]
  v &= 0b00111111; // low bits
  h &= 0b00111111; // high bits
  v |= 0b11000000; // mask
  h |= 0b11000000; // mask

  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  

  if (weight) {

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low byte is at adresse 0x55 OR 0X57 11+[6:0] for ETH24H OR ETH24L
    writeRegister(AK975X_EETH24H_LOW, v); // ETH24H low
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high byte is at adresse 0x56 OR 0X58 11+[6:0] for ETH24H OR ETH24L
    writeRegister(AK975X_EETH24H_HIGH, h); // ETH24H High
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=readRegister(AK975X_EETH24H_LOW);
    //Serial.print("AK975X_EETH24H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH24H_HIGH);
    //Serial.print("AK975X_EETH24H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low byte is at adresse 0x55 OR 0X57 11+[6:0] for ETH24H OR ETH24L
    writeRegister(AK975X_EETH24L_LOW, v); // ETH24L low
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high byte is at adresse 0x56 OR 0X58 11+[6:0] for ETH24H OR ETH24L
    writeRegister(AK975X_EETH24L_HIGH, h); // ETH24L High
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=readRegister(AK975X_EETH24L_LOW);
    //Serial.print("AK975X_EETH24L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH24L_HIGH);
    //Serial.print("AK975X_EETH24L_HIGH : ");
    //Serial.println(currentSettings,BIN);

  }

  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setThresholdIr1Ir3(bool weight, int v) {

  int k=v >>5; //high bits [11;5]
  int b=v <<3; // low bits
  b &= 0b11111000; // low bits
  k &= 0b01111111; // high bits
  byte currentSettings;

  if (weight) {

    writeRegister(AK975X_ETH13H_LOW, b);
    writeRegister(AK975X_ETH13H_HIGH, k);

    currentSettings=readRegister(AK975X_ETH13H_LOW);
    //Serial.print("AK975X_ETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_ETH13H_HIGH);
    //Serial.print("AK975X_ETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    writeRegister(AK975X_ETH13L_LOW, b);
    writeRegister(AK975X_ETH13L_HIGH, k);

    currentSettings=readRegister(AK975X_ETH13L_LOW);
    //Serial.print("AK975X_ETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_ETH13L_HIGH);
    //Serial.print("AK975X_ETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);
  } 
}

void AK975X::setThresholdEepromIr1Ir3(bool weight, int v) {

  int h=v >>6; //high bits [11;6]
  v &= 0b00111111; // low bits
  h &= 0b00111111; // high bits
  v |= 0b11000000; // mask
  h |= 0b11000000; // mask

  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  

  if (weight) {

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low byte is at adresse 0x52 OR 0X54 11+[6:0] for ETH13H OR ETH13L
    writeRegister(AK975X_EETH13H_LOW, v); // ETH13H low
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms
    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high byte is at adresse 0x51 OR 0X53 11+[6:0] for ETH13H OR ETH13L
    writeRegister(AK975X_EETH13H_HIGH, h); // ETH13H High
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=readRegister(AK975X_EETH13H_LOW);
    //Serial.print("AK975X_EETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13H_HIGH);
    //Serial.print("AK975X_EETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

  } else {

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // low byte is at adresse 0x52 OR 0X54 11+[6:0] for ETH13H OR ETH13L
    writeRegister(AK975X_EETH13L_LOW, v); // ETH13L low
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

    // high byte is at adresse 0x51 OR 0X53 11+[6:0] for ETH13H OR ETH13L
    writeRegister(AK975X_EETH13L_HIGH, h); // ETH13L High
    delay(15); //  EEPROM write time (tWR) should be longer than 10ms

    currentSettings=readRegister(AK975X_EETH13L_LOW);
    //Serial.print("AK975X_EETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13L_HIGH);
    //Serial.print("AK975X_EETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);

  }

  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::readThreshold() {
  byte currentSettings;

  currentSettings=readRegister(AK975X_ETH24H_LOW);
  //Serial.print("AK975X_ETH24H_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH24H_HIGH);
  //Serial.print("AK975X_ETH24H_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH24L_LOW);
  //Serial.print("AK975X_ETH24L_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH24L_HIGH);
  //Serial.print("AK975X_ETH24L_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH13H_LOW);
  //Serial.print("AK975X_ETH13H_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH13H_HIGH);
  //Serial.print("AK975X_ETH13H_HIGH : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH13L_LOW);
  //Serial.print("AK975X_ETH13L_LOW : ");
  //Serial.println(currentSettings,BIN);

  currentSettings=readRegister(AK975X_ETH13L_HIGH);
  //Serial.print("AK975X_ETH13L_HIGH : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::readThresholdEeprom() {
  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  

    currentSettings=readRegister(AK975X_EETH24H_LOW);
    //Serial.print("AK975X_EETH24H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH24H_HIGH);
    //Serial.print("AK975X_EETH24H_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH24L_LOW);
    //Serial.print("AK975X_EETH24L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH24L_HIGH);
    //Serial.print("AK975X_EETH24L_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13H_LOW);
    //Serial.print("AK975X_EETH13H_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13H_HIGH);
    //Serial.print("AK975X_EETH13H_HIGH : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13L_LOW);
    //Serial.print("AK975X_EETH13L_LOW : ");
    //Serial.println(currentSettings,BIN);

    currentSettings=readRegister(AK975X_EETH13L_HIGH);
    //Serial.print("AK975X_EETH13L_HIGH : ");
    //Serial.println(currentSettings,BIN);    

  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setHysteresisIr2Ir4(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  byte currentSettings;

  writeRegister(AK975X_EHYS24, v); // EHYS24 hysteresis level
  currentSettings=readRegister(AK975X_EHYS24);
  //Serial.print("AK975X_EHYS24 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::setHysteresisEepromIr2Ir4(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  
  writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

  writeRegister(AK975X_EEHYS24, v); // EHY24 hysteresis level
  delay(15);

  currentSettings=readRegister(AK975X_EEHYS24);
  //Serial.print("AK975X_EEHYS24 : ");
  //Serial.println(currentSettings,BIN);

  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  

}

void AK975X::setHysteresisIr1Ir3(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  byte currentSettings;

  writeRegister(AK975X_EHYS13, v); // EHYS13 hysteresis level
  currentSettings=readRegister(AK975X_EHYS13);
  //Serial.print("AK975X_EHYS13 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::setHysteresisEepromIr1Ir3(int v) {

  v &= 0b00011111; // we keep the 5 last bits
  v |= 0b11100000; // mask

  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  
  writeRegister(AK975X_EKEY, EKEY_ON); // allow EEPROM writing

  writeRegister(AK975X_EEHYS13, v); // EHY13 hysteresis level
  delay(15); 

  currentSettings=readRegister(AK975X_EEHYS13);
  //Serial.print("AK975X_EEHYS13 : ");
  //Serial.println(currentSettings,BIN);

  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::readHysteresis() {

  byte currentSettings;

  currentSettings=readRegister(AK975X_EHYS24);
  //Serial.print("AK975X_EHYS24 : ");
  //Serial.println(currentSettings,BIN);
  currentSettings=readRegister(AK975X_EHYS13);
  //Serial.print("AK975X_EHYS13 : ");
  //Serial.println(currentSettings,BIN);
}

void AK975X::readHysteresisEeprom() {

  byte currentSettings;
  byte currentMode = readRegister(AK975X_ECNTL1);
  byte temp = EEPROM_MODE | (currentMode & 00111000);

  writeRegister(AK975X_ECNTL1, temp); // open EEPROM mode  

  currentSettings=readRegister(AK975X_EEHYS24);
  //Serial.print("AK975X_EEHYS24 : ");
  //Serial.println(currentSettings,BIN);
  currentSettings=readRegister(AK975X_EEHYS13);
  //Serial.print("AK975X_EEHYS13 : ");
  //Serial.println(currentSettings,BIN);


  writeRegister(AK975X_ECNTL1, currentMode); // close EEPROM mode  
}

void AK975X::setInterrupts(bool ir13h,bool ir13l,bool ir24h,bool ir24l,bool dr) {
  // mask , assembly of AK975X_EINTEN bits
  int v = 0b11000000 | ( ir13h<<4 | ir13l<<3 | ir24h<<2 | ir24l<<1 | dr);
  writeRegister(AK975X_EINTEN, v); 

  byte currentSettings=readRegister(AK975X_EINTEN);
  //Serial.print("AK975X_EINTEN : ");
  //Serial.println(currentSettings,BIN);  
}

int AK975X::readInterruptStatus() {
  byte currentSettings=readRegister(AK975X_INTST);
  //Serial.print("AK975X_INTST : ");
  //Serial.println(currentSettings,BIN);
  if bitRead(currentSettings,0) return 5;
  if bitRead(currentSettings,1) return 1;
  if bitRead(currentSettings,2) return 2;
  if bitRead(currentSettings,3) return 3;
  if bitRead(currentSettings,4) return 4;
  return 0;
}

//Checks to see if DRDY flag is set in the status register
boolean AK975X::available()
{
  uint8_t value = readRegister(AK975X_ST1);
  return (value & (1 << 0)); //Bit 0 is DRDY
}

//Checks to see if Data overrun flag is set in the status register
boolean AK975X::overrun()
{
  uint8_t value = readRegister(AK975X_ST1);

  return (value & 1 << 1); //Bit 1 is DOR
}

//Does a soft reset
void AK975X::softReset()
{
  writeRegister(AK975X_CNTL2, 0xFF);
}

//Turn on/off Serial.print statements for debugging
void AK975X::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging

  _printDebug = true; //Should we print the commands we send? Good for debugging
}
void AK975X::disableDebugging()
{
  _printDebug = false; //Turn off extra print statements
}

//Reads from a give location
uint8_t AK975X::readRegister(uint8_t location)
{
  uint8_t result; //Return value

  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);

  result = _i2cPort->endTransmission();

  if ( result != 0 )
  {
    printI2CError(result);
    return (255); //Error
  }

  _i2cPort->requestFrom((int)_i2caddr, 1); //Ask for one byte
  while ( _i2cPort->available() ) // slave may send more than requested
  {
    result = _i2cPort->read();
  }

  return result;
}

//Write a value to a spot
void AK975X::writeRegister(uint8_t location, uint8_t val)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);
  _i2cPort->write(val);
  _i2cPort->endTransmission();
}

//Reads a two byte value from a consecutive registers
uint16_t AK975X::readRegister16(byte location)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);

  uint8_t result = _i2cPort->endTransmission();

  if ( result != 0 )
  {
    printI2CError(result);
    return (255); //Error
  }

  _i2cPort->requestFrom((int)_i2caddr, 2);

  uint16_t data = _i2cPort->read();
  data |= (_i2cPort->read() << 8);

  return (data);
}

/** Private functions ***********************/

//If I2C communication fails this function will tell us which error occured
//Originally from Robotic Materials: https://github.com/RoboticMaterials/FA-I-sensor/blob/master/force_proximity_eval/force_proximity_eval.ino
uint8_t AK975X::printI2CError(uint8_t errorCode)
{
  if (_printDebug == true)
  {
    switch (errorCode)
    {
      //From: https://www.arduino.cc/en/Reference/WireEndTransmission
      case 0:
        _debugSerial->println(F("Success"));
        break;
      case 1:
        _debugSerial->println(F("Data too long to fit in transmit buffer"));
        break;
      case 2:
        _debugSerial->println(F("Received NACK on transmit of address"));
        break;
      case 3:
        _debugSerial->println(F("Received NACK on transmit of data"));
        break;
      case 4:
        _debugSerial->println(F("Unknown error"));
        break;
    }
  }
  return (errorCode); //No matter what pass the code back out
}

