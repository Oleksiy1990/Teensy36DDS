/*
AD9954.cpp - AD9954 DDS communication library
Created by Neal Pisenti, 2013
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
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/


#include "Arduino.h"
#include "SPI.h"
#include "AD9954.h"
#include "math.h"

//const unsigned int PS0_BIT_SET = (1 << 21); // This is output 7 on Arduino Zero
byte chargePumpvalue = 0;
int power = 1;


/* CONSTRUCTOR */

// Constructor function; initializes communication pinouts
AD9954::AD9954(byte ssPin, byte resetPin, byte updatePin, byte ps0, byte ps1, byte osk, byte pwrContr, bool externalUpdate)
{
	RESOLUTION  = 4294967296.0;
	_ssPin = ssPin;
	_resetPin = resetPin;
	_updatePin = updatePin;
	_ps0 = ps0;
	_ps1 = ps1;
	_osk = osk;
	_pwrContr = pwrContr;
	_externalUpdate = externalUpdate;


	// sets up the pinmodes for output
	pinMode(_ssPin, OUTPUT);
	pinMode(_resetPin, OUTPUT);
	pinMode(_updatePin, OUTPUT);
	pinMode(_ps0, OUTPUT);
	pinMode(_ps1, OUTPUT);
	pinMode(_osk, OUTPUT);
	pinMode(_pwrContr, OUTPUT);

	// defaults for pin logic levels
	digitalWrite(_ssPin, HIGH);
	digitalWrite(_resetPin, LOW);
	digitalWrite(_updatePin, LOW);
	digitalWrite(_ps0, LOW);
	digitalWrite(_ps1, LOW);
	digitalWrite(_osk, LOW);
	digitalWrite(_pwrContr, LOW);
}


/* PUBLIC CLASS FUNCTIONS */


// initialize(refClk) - initializes DDS with reference clock frequency refClk
//write 4 * 0x00 to Control Function Register No.1(CFR1 = (0x00))

void AD9954::initialize(unsigned long refClk) {
	_refIn = refClk;
	_refClk = refClk;

	byte registerInfo[] = {0x00, 4};
	byte data[] = {0x02, 0x00, 0x00, 0x00};	// 0x02 sets the OSK bit enables the ASF functionality.
	if (debug) {
		Serial.println("AD9954::initialize DDS (CFR1)(0x00)");
	}
	AD9954::writeRegister(registerInfo, data);

}

// initialize(refClk, clkMult) -- initializes DDS with input refClk, and activates the
// onboard PLL multiplier clkMult.
// clkMult: must be integer between 4 and 20 inclusive.
// write 3 * bloks of data to (Control Function Register No. 2(CFR2)= (0x01))

void AD9954::initialize(unsigned long refClk, byte clkMult) {
	_refIn = refClk;
	_refClk = _refIn * clkMult;

	byte multValue = clkMult;

	byte registerInfo[] = {0x01, 3};
	byte data[] = {0x18, 0x00, 0x00};

	// writes value for clock multiplier
	if (_refClk < 25000000) {
		multValue <<= 1;
		//multValue = multValue << 1;
		} else {
		(multValue <<= 1)++;
		//multValue = (multValue << 1) + 1;

		multValue <<= 2;
		//multValue = multValue << 2;
		multValue = multValue + chargePumpvalue;	// Add the chargePumpvalue bits to register ( CFR2 0x01 ).

		data[2] = lowByte(multValue);

		if (debug) {
			Serial.println("AD9954::(CFR2)(0x01)");
		}
		AD9954::writeRegister(registerInfo, data);
	}
}


// Amplitude is set on the scale 0-100
void AD9954::setASF(float setAmplitude) {

	_asf = (uint16_t) (setAmplitude / 100.0 * 16383);

	byte registerInfo[] = {0x02, 2};
	byte asf[] = { lowByte(_asf >> 8), lowByte(_asf)};

	if (debug) {
		Serial.print("AD9954::setAmplitude 0x02 = "); Serial.print(",  ");  Serial.print(_asf);
		Serial.print(" asf[0] = 0x"); Serial.print(asf[0], HEX);  Serial.print(",  ");
		Serial.print(" asf[1] = 0x"); Serial.println(asf[1], HEX);
	}

	AD9954::writeRegister(registerInfo, asf);
	if (!_externalUpdate) {
		AD9954::update();
		}
	}



void AD9954::setChargepump(byte chargePump) {

	//_chargePump = chargePump;
	chargePumpvalue =  chargePump;

	if (debug) {
		Serial.print("AD9954::chargePumpvalue = ");  Serial.println(chargePumpvalue, BIN);
	}
}






// reset() - takes no arguments; resets DDS
void AD9954::reset() {
	if (debug) {
		Serial.println("reset DDS");  Serial.println("");
	}
	digitalWrite(_resetPin, HIGH);
	delay(1);
	digitalWrite(_resetPin, LOW);
}

// oskEnable() - takes no arguments; To controll the DDS OSK pin.
void AD9954::oskEnable () {
	if (debug) {
		Serial.println("DDS oksEnable");  Serial.println("");
	}
	digitalWrite(_osk, HIGH);
}



// setPower() - set DDS power on & off
void AD9954::setPower( int power) {
	if (power == 1) {
		digitalWrite(_pwrContr, LOW);
		if (debug) {
			Serial.println("DDS AD9954 switch = ON"); Serial.println("");
		}
	}

	if (power == 0) {
		digitalWrite(_pwrContr, HIGH);
		if (debug) {
			Serial.println("DDS AD9954 switch = OFF"); Serial.println("");
		}
	}
}

// update() - sends a logic pulse to the DDS update pin, shift the DDS register value.
// newly set frequency (FTW0) etc.
void AD9954::update() {
	if (debug) {
		Serial.println("AD9954::update");
	}
	digitalWrite(_updatePin, HIGH);
	digitalWrite(_updatePin, LOW);
}


// setFreq(freq) -- writes freq to DDS board, in FTW0
// write 4 * bloks of data default (0x00)  to (Frequency Tuning Word register(FTW0)= (0x04))

void AD9954::setFreq(unsigned long freq) {

	// set _freq and _ftw variables
	_freq = freq;
	_ftw = freq * RESOLUTION / _refClk ;

	// divide up ftw into four bytes
	byte ftw[] = { lowByte(_ftw >> 24), lowByte(_ftw >> 16), lowByte(_ftw >> 8), lowByte(_ftw)};
	// register info -- writing four bytes to register 0x04,
	byte registerInfo[] = {0x04, 4};

	if (debug) {
		Serial.println("AD9954::(FTW0)(0x04) in setFreq");
	}
	AD9954::writeRegister(registerInfo, ftw);

	// issues update command
	if (!_externalUpdate) {
		AD9954::update();
		}
}


// getFreq() - returns current frequency work in in progress not implemented yet.
unsigned long AD9954::getFreq() {
	return _freq;
}

// getFTW() -- returns current FTW work in progress not implemented yet.
unsigned long AD9954::getFTW() {
	return _ftw;
}

// Function setFTW -- accepts 32-bit frequency tuning word ftw;
//      updates instance variables for FTW and Frequency, and writes ftw to DDS.
void AD9954::setFTW(unsigned long ftw) {

	// set freqency and ftw variables
	_ftw = ftw;
	_freq = ftw * _refClk / RESOLUTION;

	// divide up ftw into four bytes
	byte data[] = { lowByte(_ftw >> 24), lowByte(_ftw >> 16), lowByte(_ftw >> 8), lowByte(_ftw)};
	// register info -- writing four bytes to register 0x04,
	byte registerInfo[] = {0x04, 4};

	if (debug) {
		Serial.println("AD9954::(FTW0)(0x04) in setFTW");
	}
	AD9954::writeRegister(registerInfo, data);
	if (!_externalUpdate) {
		AD9954::update();
		}

}

// Function linearSweep -- places DDS in linear sweep mode.
//      Behavior is determined by two frequency tuning words, freq0 and freq1 (freq0 < freq1).
//      The PS0 pin HIGH will ramp towards freq1, PS0 LOW will ramp towards freq0.
//      The rate of the ramp is dictated by pos/negDF (positive/negative DeltaFreq), and pos/negRR (positive/negative RampRate).
//
//      freq0: lower frequency bound (Hz)
//      freq1: upper frequency bound (Hz)
//      posDF: delta frequency for positive ramp (Hz)
//      negDF: delta frequency for negative ramp (Hz)
//      posRR: number between 0 and 255, indicating number of SYNC_CLK cycles spent at each
//			frequency value in the ramp. SYNC_CLK operates at 1/4 of the SYSCLK clock value. Typically SYNC_CLK = 100MHz.
//          Thus, the true "ramp rate" is, eg, posDF/(posRR*10 ns)
//      negRR: same as above, but for negative ramp.
//
//      As a general rule, round up (not down) in calculating the delta frequency steps.

void AD9954::linearSweep(float freq0, float freq1, float posTimeMicros, float negTimeMicros, float waitingTimeMicros, float powerPercentage, byte mode, bool noDwell) {
	// Modes: 0 -> only sweep up and wait
	// 1 -> sweep up, wait, sweep down
	//digitalWrite(_ps1,LOW); // not sure if this is needed, but OK

	/*
	In the command the hardware only requires frequency tuning word for the bottom frequency,
	frequency tuning word for the top frequency, the Delta frequency values for the way
	up and down, and the number of cycles to sit at each frequency
	*/

	// At the beginning these two pins have to be set low
	digitalWrite(_ps0,LOW);
	digitalWrite(_ps1,LOW);


	if (freq1 <= freq0) {
		//Serial.println("Make sure that upper freq is above lower freq!");
		return;
	}
	float frequencySpan = (freq1 - freq0);
	byte RampRatePos = 2;
	byte RampRateNeg = 2;
	long sequenceDurationUpwards;
	long sequenceDurationDownwards;

	unsigned long posDF = 0;
	unsigned long negDF = 0;
	unsigned long numStepsPositive;
	unsigned long numStepsNegative;
	//Serial.println("Here in linear_ramp");

/*
	the next two while-loops are there to find the finest possible sweep
	but still without getting Delta freq = 0, in which case it doesn't sweep
	So I artificially set the min delta freq to 3 Hz
*/
	while (posDF < 3) { // Just say that the Delta freq cannot be less than 3 Hz
		numStepsPositive = (unsigned long) posTimeMicros/(RampRatePos*10.*0.001); // 10 ns is the time step for 400 MHz clock case!
		posDF = frequencySpan/numStepsPositive;
		//Serial.println("We will get posDF");
		//Serial.println(posDF);
		if (posDF < 3) {
			RampRatePos += 1;
		}
		if (RampRatePos > 253) {
			//Serial.println("Too long ramp time, too small frequency span on way up");
			break;
		}
		//Serial.println("We will get RampRatePos");
		//Serial.println(RampRatePos);
	}

	while (negDF < 3) { // Just say that the Delta freq cannot be less than 3 Hz
		numStepsNegative = (unsigned long) negTimeMicros/(RampRateNeg*10.*0.001); // 10 ns is the time step for 400 MHz clock case!
		negDF = frequencySpan/numStepsNegative;
		if (negDF < 3) {
			RampRateNeg += 1;
		}
		if (RampRateNeg > 253) {
			//Serial.println("Too long ramp time, too small frequency span on way down");
			break;
		}
	}
  //REG_PORT_OUT0 |= PS0_BIT_SET; // this is to make sure that there is no output
	//Serial.println("Start of linear ramp function, writing high");
	//digitalWrite(_updatePin,LOW);

	switch (mode) {
		case 0:
	 		negDF = frequencySpan/2; // this is basically to immediately go back
			break;
		case 1:
			break;
		default:
			break;
	}

	//Serial.println("We will get freq span, numStepsPositive,posDF");
	//Serial.println(frequencySpan);
	//Serial.println(numStepsPositive);
	//Serial.println(posDF);

	unsigned long ftw0 = (unsigned long) freq0 * RESOLUTION / _refClk;
	unsigned long ftw1 = (unsigned long) freq1 * RESOLUTION / _refClk;
	unsigned long posDFW = (unsigned long) posDF * RESOLUTION / _refClk;
	unsigned long negDFW = (unsigned long) negDF * RESOLUTION / _refClk;


	// construct register values
	byte CFR1[] = { 0x02, 0x20, 0x00, 0x00}; // 0x02 sets the OSK bit, enables the ASF functionality, 0x20 (linear sweep bit)enables the linear sweep mode.

	if (noDwell) {
		CFR1[3] = 0x04;
	}	// This sets the no-dwell mode, see AD9954 manual
	byte CFR1Info[] = {0x00, 4};

	byte FTW0[] = {lowByte(ftw0 >> 24), lowByte(ftw0 >> 16), lowByte(ftw0 >> 8), lowByte(ftw0) };
	byte FTW0Info[] = {0x04, 4};

	byte FTW1[] = {lowByte(ftw1 >> 24), lowByte(ftw1 >> 16), lowByte(ftw1 >> 8), lowByte(ftw1) };
	byte FTW1Info[] = {0x06, 4};

	byte NLSCW[] = { RampRateNeg, lowByte(negDFW >> 24), lowByte(negDFW >> 16), lowByte(negDFW >> 8), lowByte(negDFW) };
	byte NLSCWInfo[] = {0x07, 5};

	byte PLSCW[] = { RampRatePos, lowByte(posDFW >> 24), lowByte(posDFW >> 16), lowByte(posDFW >> 8), lowByte(posDFW) };
	byte PLSCWInfo[] = {0x08, 5};

	// writing information to all the registers in the device
	AD9954::writeRegister(CFR1Info, CFR1);
	if (debug) {
		Serial.println("AD9954::writeRegister(CFR1 - 0x00)");
	}

	AD9954::writeRegister(FTW0Info, FTW0);
	if (debug) {
		Serial.println("AD9954::writeRegister(FTW0 - 0x04");
	}

	AD9954::writeRegister(FTW1Info, FTW1);
	if (debug) {
		Serial.println("AD9954::writeRegister(FTW1 - 0x06");
	}

	AD9954::writeRegister(NLSCWInfo, NLSCW);
	if (debug) {
		Serial.println("AD9954::writeRegister(NLSCW - 0x07");
	}

	AD9954::writeRegister(PLSCWInfo, PLSCW);
	if (debug) {
		Serial.println("AD9954::writeRegister(PLSCW - 0x08");
	}

	switch (mode) {
		case 0:
			AD9954::setASF(powerPercentage);
			AD9954::update();
			sequenceDurationUpwards = (posTimeMicros + waitingTimeMicros);
			digitalWrite(_ps0,HIGH);
			//REG_PORT_OUT0 &= ~PS0_BIT_SET;
			//REG_PORT_OUT0 |= PS0_BIT_SET; // This procedure initiates the up-sweep

			while (sequenceDurationUpwards > 10000) {
				delayMicroseconds(10000);
				sequenceDurationUpwards -= 10000;
			}
			delayMicroseconds(sequenceDurationUpwards);
			//AD9954::setASF(0);
			//AD9954::update();
			//digitalWrite(_ps0,LOW);
			//delayMicroseconds(100);
			//REG_PORT_OUT0 &= ~PS0_BIT_SET;
			//delay(10);
			break;
		case 1:
			AD9954::setASF(powerPercentage);
			AD9954::update();
			digitalWrite(_ps0,HIGH);
			sequenceDurationUpwards = (posTimeMicros + waitingTimeMicros);
			sequenceDurationDownwards = negTimeMicros;
			//REG_PORT_OUT0 |= PS0_BIT_SET; // This procedure initiates the up-sweep
			while (sequenceDurationUpwards > 10000) {
				delayMicroseconds(10000);
				sequenceDurationUpwards -= 10000;
			}
			delayMicroseconds(sequenceDurationUpwards);
			digitalWrite(_ps0,LOW);
			//REG_PORT_OUT0 &= ~PS0_BIT_SET;
			while (sequenceDurationDownwards > 10000) {
				delayMicroseconds(10000);
				sequenceDurationDownwards -= 10000;
			}
			delayMicroseconds(sequenceDurationDownwards);
			//AD9954::setASF(0);
			//AD9954::update();
			//delayMicroseconds(100);
			break;
		default:
			//Serial.println("Inside the default case");
			digitalWrite(_ps0, LOW);
			break;
    //digitalWrite(_ps0, LOW);
	}
}



/* PRIVATE CLASS FUNCTIONS */

// Writes SPI to particular register.
//      registerInfo is a 2-element array which contains [register, number of bytes]
void AD9954::writeRegister(byte registerInfo[], byte data[]) {

	digitalWrite(_ssPin, LOW); // This is required for approprite SPI communication
	// SPI library itself does not control the CS pin

	// Writes the register value
	SPI.transfer(registerInfo[0]);

	// Writes the data
	for (int i = 0; i < registerInfo[1]; i++) {
		if (debug) {
			Serial.print("Writing data block # ");  Serial.println(i);
		}
		SPI.transfer(data[i]);
	}
	digitalWrite(_ssPin, HIGH); // For us to be able to initiate another SPI transaction
}

//
