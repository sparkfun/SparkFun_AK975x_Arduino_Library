/*
  This is a library written for the AK975X Human Presence Sensor.

  Written by Nathan Seidle @ SparkFun Electronics, March 10th, 2017
  Revised by Andy England @ SparkFun Electronics, October 17th, 2017

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

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define AK975X_DEFAULT_ADDRESS 0x64 //7-bit unshifted default I2C Address
//Address is changeable via two jumpers on the rear of the PCB.
//Allowed settings are:
//00 (0x64 default)
//01 (0x65)
//10 (0x66)
//11 Not allowed - used for switch mode

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Register addresses
#define AK975X_WIA2 0x01 //Device ID
#define AK975X_INFO1 0x02
#define SENSORVERSION_AK9750 0x00
#define SENSORVERSION_AK9753 0x01
#define AK975X_ST1 0x05
#define AK975X_IR1 0x06
#define AK975X_IR2 0x08
#define AK975X_IR3 0x0A
#define AK975X_IR4 0x0C
#define AK975X_TMP 0x0E
#define AK975X_ST2 0x10 //Dummy register
#define AK975X_ETH13H 0x11
#define AK975X_ETH13L 0x13
#define AK975X_ETH24H 0x15
#define AK975X_ETH24L 0x17
#define AK975X_EHYS13 0x19
#define AK975X_EHYS24 0x1A
#define AK975X_EINTEN 0x1B
#define AK975X_ECNTL1 0x1C
#define AK975X_CNTL2 0x19

//Valid sensor modes - Register ECNTL1
#define AK975X_MODE_STANDBY 0b000
#define AK975X_MODE_EEPROM_ACCESS 0b001
#define AK975X_MODE_SINGLE_SHOT 0b010
#define AK975X_MODE_0 0b100
#define AK975X_MODE_1 0b101
#define AK975X_MODE_2 0b110
#define AK975X_MODE_3 0b111

//Valid digital filter cutoff frequencies
#define AK975X_FREQ_0_3HZ 0b000
#define AK975X_FREQ_0_6HZ 0b001
#define AK975X_FREQ_1_1HZ 0b010
#define AK975X_FREQ_2_2HZ 0b011
#define AK975X_FREQ_4_4HZ 0b100
#define AK975X_FREQ_8_8HZ 0b101

class AK975X {
  public:
    //By default use Wire, standard I2C speed, and the defaul AK975X address
    boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = AK975X_DEFAULT_ADDRESS, long baud = 9600);

    int16_t getIR1(); //Get the IR values for various sensors
    int16_t getIR2(); //It's a low or negative number that increases
    int16_t getIR3(); //as heat/humans are detected
    int16_t getIR4();

    void refresh(); //Reads the dummy register telling sensor to calculate next reading
    boolean available(); //True if DRDY bit is set
    boolean overrun(); //True if OVR bit is set
    void softReset(); //Resets the IC via software

    void setMode(uint8_t mode = AK975X_MODE_0); //Set mode of the sensor. Mode 0 is continuous read mode
    void setCutoffFrequency(uint8_t frequency = AK975X_FREQ_8_8HZ); //Filtering freq. 8Hz is fastest, least filtered

    float getTemperature(); //Returns sensor temp in C
    float getTemperatureF(); //Returns sensor temp in F for silly non-metric people (me)

    void enableDebugging(Stream &debugPort = Serial); //Output various extra messages to help with debug
    void disableDebugging();

    uint8_t readRegister(uint8_t location); //Basic read of a register
    void writeRegister(uint8_t location, uint8_t val); //Writes to a location
    uint16_t readRegister16(byte location); //Reads a 16bit value

    //Variables

  private:
    uint8_t printI2CError(uint8_t errorCode); //Prints endTransmission statuses

    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _i2caddr;

    boolean _printDebug = false; //Flag to print the serial commands we are sending to the Serial port for debug

    Stream *_debugSerial; //The stream to send debug messages to if enabled
};


