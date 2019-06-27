/*
   AD9954.h - AD9954 DDS communication library
   Created by Neal Pisenti, 2013.
   JQI - Strontium - UMD

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   aunsigned long with this program.  If not, see <http://www.gnu.org/licenses/>.


*/

#ifndef AD9954_h
#define AD9954_h

#include "Arduino.h"

// Enable / disable test 65 .. 120 MHz single tone mode.
//const bool test = true;
const bool test = false;

// Enable / disable print debug info in 65 .. 120 MHz single tone test mode.
const bool debug1 = false;
//const bool debug1 = false;


// Enable / disable test single tone mode.
const bool test_St = false;
//const bool test_St = false;

// Enable / disable linear sweep test mode.
//const bool test_Ls = true;
const bool test_Ls = false;


// Enable / disable print debug info.
//const bool debug = true;
const bool debug = false;





class AD9954
{
  public:
    // Constructor function.
    AD9954( byte, byte, byte, byte, byte, byte, byte, bool);

    // Initialize with refClk frequency
    void initialize(unsigned long);

    // Initialize with refIn frequency, and clock multiplier value
    void initialize(unsigned long, byte);

    // Initialize with refIn frequency, and clock multiplier value, and chargepump value
    //void initialize(unsigned long, byte, byte);

    // Set the Amplitude Scale Factor
    void setASF(float);

    // Set de value of the charge pump 0 = 75uA, 1 = 100uA, 2 = 125uA, 3 = 150uA
    void setChargepump(byte);

    // Reset the DDS
    void reset();

    // Update the new frequency tuning word
    void update();

    void oskEnable();

    // Set the DDS power on & off
    void setPower(int);

    // Gets current frequency
    unsigned long getFreq();

    // Sets frequency
    void setFreq(unsigned long);

    // Gets current frequency tuning word
    unsigned long getFTW();

    // Sets frequency tuning word
    void setFTW(unsigned long);

    // places DDS in linear sweep mode
    //void linearSweep(unsigned long, unsigned long, unsigned long, byte, unsigned long, byte);
    void linearSweep(unsigned long freq0, unsigned long freq1, float posTimeMicros, float negTimeMicros, unsigned int waitingTimeMicros, byte mode, bool noDwell = false);





  private:
    // Instance variables that hold pinout mapping
    // from arduino to DDS pins.
    byte _ssPin, _resetPin, _updatePin, _ps0, _ps1, _osk, _pwrContr;

    // Instance variables for frequency _freq, frequency tuning word _ftw,
    // reference clock frequency _refClk, etc.
    unsigned long _freq, _ftw, _refClk, _refIn;

    // Instance variables set the DDS output amplitude
    uint16_t  _asf;

    // Instance variables set the DDS output amplitude
    float _setAmplitude;

    // Instance variables for the DDS9954 chargePump value
    byte _chargePump;

    // Instance variables for controll the DDS power on & off function
    //int _power;


    // function to write data to register.
    void writeRegister(byte[2], byte[1024]);


    // DDS frequency resolution
    double RESOLUTION;// = 4294967296; // sets resolution to 2^32 = 32 bits. Using type double to avoid confusion with integer division...

    bool _externalUpdate;
};


#endif
