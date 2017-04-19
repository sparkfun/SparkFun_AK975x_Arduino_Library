/*
  This is a library written for the AK9750 Human Presence Sensor.

  Written by Nathan Seidle @ SparkFun Electronics, March 10th, 2017

  The sensor uses I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  https://github.com/sparkfun/SparkFun_AK9750_Arduino_Library

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

#include "SparkFun_AK9750_Arduino_Library.h"

//Sets up the sensor for constant read
//Returns false if sensor does not respond
boolean AK9750::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{
  //Bring in the user's choices
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  uint8_t deviceID = readRegister(AK9750_WIA2);
  if (deviceID != 0x13) //Device ID should be 0x13
    return (false);

  setMode(MODE_0); //Set to continuous read

  setCutoffFrequency(FREQ_8_8HZ); //Set output to fastest, with least filtering

  refresh(); //Read dummy register after new data is read

  return (true); //Success!
}

//Returns the decimal value of sensor channel
int16_t AK9750::getIR1()
{
  return (readRegister16(AK9750_IR1));
}
int16_t AK9750::getIR2()
{
  return (readRegister16(AK9750_IR2));
}
int16_t AK9750::getIR3()
{
  return (readRegister16(AK9750_IR3));
}
int16_t AK9750::getIR4()
{
  return (readRegister16(AK9750_IR4));
}

//This reads the dummy register ST2. Required after
//reading out IR data.
void AK9750::refresh()
{
  uint8_t dummy = readRegister(AK9750_ST2); //Do nothing but read the register
}

//Returns the temperature in C
float AK9750::getTemperature()
{
  int value = readRegister16(AK9750_TMP);

  value >>= 6; //Temp is 10-bit. TMPL0:5 fixed at 0

  float temperature = 26.75 + (value * 0.125);
  return (temperature);
}

//Returns the temperature in F
float AK9750::getTemperatureF()
{
  float temperature = getTemperature();
  temperature = temperature * 1.8 + 32.0;
  return (temperature);
}

//Set the mode. Continuous mode 0 is favored
void AK9750::setMode(uint8_t mode)
{
  if (mode > MODE_3) mode = MODE_0; //Default to mode 0
  if (mode == 0b011) mode = MODE_0; //0x03 is prohibited

  //Read, mask set, write
  byte currentSettings = readRegister(AK9750_ECNTL1);

  currentSettings &= 0b11111000; //Clear Mode bits
  currentSettings |= mode;
  writeRegister(AK9750_ECNTL1, currentSettings);
}

//Set the cutoff frequency. See datasheet for allowable settings.
void AK9750::setCutoffFrequency(uint8_t frequency)
{
  if (frequency > 0b101) frequency = 0; //Default to 0.3Hz

  //Read, mask set, write
  byte currentSettings = readRegister(AK9750_ECNTL1);

  currentSettings &= 0b11000111; //Clear EFC bits
  currentSettings |= (frequency << 3); //Mask in
  writeRegister(AK9750_ECNTL1, currentSettings); //Write
}

//Checks to see if DRDY flag is set in the status register
boolean AK9750::available()
{
  uint8_t value = readRegister(AK9750_ST1);
  return (value & (1 << 0)); //Bit 0 is DRDY
}

//Checks to see if Data overrun flag is set in the status register
boolean AK9750::overrun()
{
  uint8_t value = readRegister(AK9750_ST1);

  return (value & 1 << 1); //Bit 1 is DOR
}

//Does a soft reset
void AK9750::softReset()
{
  writeRegister(AK9750_CNTL2, 0xFF);
}

//Turn on/off Serial.print statements for debugging
void AK9750::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging

  _printDebug = true; //Should we print the commands we send? Good for debugging
}
void AK9750::disableDebugging()
{
  _printDebug = false; //Turn off extra print statements
}

//Reads from a give location
uint8_t AK9750::readRegister(uint8_t location)
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
void AK9750::writeRegister(uint8_t location, uint8_t val)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);
  _i2cPort->write(val);
  _i2cPort->endTransmission();
}

//Reads a two byte value from a consecutive registers
uint16_t AK9750::readRegister16(byte location)
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
uint8_t AK9750::printI2CError(uint8_t errorCode)
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

