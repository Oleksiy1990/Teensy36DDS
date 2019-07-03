/*
AD9954 Example Code
By:
Company:
Date: 5-15-2019

Uses the AD9954 library to control the DDS board support the AD9954.
Only a few basic functions are implemented.
Single tone mode
Lineair sweep mode.

IOSYS		=	GND	(Active high).

MISO		=	(pin1 SPI bus).
MOSI		=	(pin4 SPI bus).
SCK			=	(pin3 SPI bus).

DDS output DC level 1,8Volt DC RF output -10dBm.


DDS.reset()						=	DDS hardware reset.
DDS.setPower(x)					=	Hardware pin (pwr), Function to witch ON and OFF the DDS9954  (1) = DDS-ON , (0) = DDS-OFF Default = OFF
DDS.initialize(xx)				=	initialize DDS with 400 MHz clock and set register (CFR10x00  bit <25> = (1) = OSK enable.
DDS.setASF(100)					=	Set the output power value (  0 .. 100%), set register(ASF 0x02)  <0 .. 13> = (value).
DDS.setChargepump				=	set DDS charge pump to 0 = 75uA, 1 = 100uA, 2 = 125uA, 3 = 150uA	set register (CFR2 0x01)  bits <0, 1> = (value).
DDS.initialize(20000000, 20)	+	initialize DDS with 400 MHz clock, set register (CFR2 0x01) bit <2> = VCO range & bits <3 .. 7 > = (value) = REFCLK multiplier.

*/


/*
Serial communication protocol :

This is written from the point of view of the Arduino, hence "send" means that Arduino sends it, and "receive" is the same way

Always receive and send ascii characters

1) Handshake: Receive 'h' and if all good respond with 'y', else respond with 'n'
2) Receive a character to define operation mode. Options:
	'f' -> "frequency", so single tone operation (given in Hz)
	'p' -> "power", so changing the DDS output power (given on the scale 0-100)
	'd' -> "down", so turn off the DDS
	'u' -> "up", so turn on the DDS
	's' -> "sequence", so execute a triggered sequence

	If the option chosen is 'f' or 'p', immediately send the value in the same command, like "f80000000" will set the frequency to 80 MHz
	If the option chosen is 'd' or 'u', do not send anything else immedaitely
	If the option chosen is 's', send a number of steps in the sequence immediately, so command like "s5" will create a sequence with 5 steps
3) Send the response. If everything has been done correctly in the step before, the response should be 'c', otherwise it should be 'n', 'x' signified that a wrong symbol
has been sent, 't' meant "timeout" so too much time has passed since the handshake

Sequence programming:

1) After having received 'sN' in the previous steps, where N is an integer denoting the number of steps in the sequence, and responded with 'c',
we have to receive the information as to what the sequence will be. The rule is to receive a set of 4 letters of the form 'fplw'
where 'f' is "frequency", 'p' is "power" and 'l' is "linear sweep", or 'w' which is "softWare sweep"
Any of those letters can be substituted with 'v' which stands for "void".
Notice the positions: positions 0 and 2 are for frequency single point data and sweet respectively
positions 1 and 3 are for power single point data and sweep respectively;

the frequency sweep can be 'l' or 'w', while the power sweep can only be 'w' because this DDS
at least immediately doesn't have a utility for hardware power sweeps

We define it so that a linear sweep is programmed in hardware onto the DDS itself, while a software sweep
means that Arduino calculates the points in time and updates the DDS every point (of course this is much slower,
but the function can essentially be arbitary)

If it's all good, Arduino will send response 'r' (meaning "received"). If there is any error, the response will be 'n'.

2) After each quadtuplet of letters we receive the corresponding numerical data, for each
entry in the quarduplet, as a string. Each string of numerical data must be finished with '\n'
If everything is received correctly in the format, the response will be 'r', any error
will lead to response 'n'
Arduino assumes that if any entry was 'v', there will be no data at all sent for that entry.

Furthermore, Arduino will check that one of the entries among 0 and 2 , and 1 and 3, must be 'v'
This is just because at any given time slot we can either set one value or do a sweep, but
not two at the same time

*/



#include <arduino.h>
#include <SPI.h> // SPI library, Arduino style
#include "AD9954.h"
//#include "sam.h"
#include "math.h"
#include <stdint.h>

// this is the pin numbering on the
const byte ssPin = 10;
const byte resetPin = 24;
const byte updatePin = 5;
const byte ps0Pin = 25;
const byte ps1Pin = 26;
const byte oskPin = 27;
const byte pwrContrPin = 28;
const byte interruptPin = 2;

const unsigned long SPIclockspeed = 1000000;

AD9954 DDS(ssPin, resetPin, updatePin, ps0Pin, ps1Pin, oskPin, pwrContrPin, true/*externalUpdate*/);
IntervalTimer SWramp_timer; // The timer to produce schedules interrups that define a software ramp
//AD9954 DDS(10, 24, 5, 25, 26, 27, 28, true);
const unsigned long lower_freq_lim = 1000000;
const unsigned long upper_freq_lim = 160000000;
const float lower_power_lim = 0;
const float upper_power_lim = 100;
const float software_ramp_time_limit = 15000; // 15000 ms = 15 s for now
const float software_ramp_min_timestep = 0.1; // 0.1 ms = 200 micros , given in ms
const float ramp_time_fraction_from_requested = 0.9; // this is to make sure that the ramp ends before the requested time so
// that the next trigger does not appear before the ramp is done and the timer is stopped


bool handshakeState = false;
char incomingChar;
unsigned long timeStart;
unsigned long timeNow;
unsigned long timeoutms = 3000;
bool isSequenceGood;
const byte numBlocksSeqStep = 4; // this is how many letters there are in the
// instruction sequence, like "fplw" or like "fvpv"

// This is only to make sure that if there are no interrups coming, at some point it stops waiting for them
// and comes back to the start of the loop, waiting for handshakes
unsigned long time_now_interrupts;
unsigned long start_time_interrupts;
unsigned long max_time_interrupts = 10000; // let's set it for 10 s for now


volatile bool interruptTriggered = false;
volatile unsigned int interruptCount = 0; // this counts the number of interrupts in any particular sequence
volatile unsigned int IntervalTimerCounter = 0; // This is how many times the interval timer sent a tick
volatile bool isTimerStarted = false; // This is to check if the timer for software ramps has started
volatile unsigned int current_loop_number; // This is to globally have access to the loop number in the sequence

String inStringFreq = ""; // this will hold the transmitted value for frequency (in Hz)
String inStringPower = ""; // this will hold the transmitted value for power (scale 0-100)
String inStringStepsSequence = ""; // This will hold the total number of steps in a given sequence
float inStringFreqFloat; // this and the next two variable will hold the previous three variables, but converted to floats and an integer respectively
float inStringPowerFloat;
int num_steps_total_sequence; // how many steps the total sequence will last (this must be the number of triggers)
const int max_possible_sequence_steps = 20; // the max possible steps for any sequence, this will be the max memory chunk for data
unsigned int num_steps_in_ramp[max_possible_sequence_steps]; // this will hold the number of steps in each software ramp

//const float freqDefault = 80000000; // default frequency
//const float powerDefault = 0; // default powers

// these arrays hold the frequencies and powers to output
// the booleans hold whether at that particular step output is active or not
float sequenceFreqs[max_possible_sequence_steps];
float sequencePowers[max_possible_sequence_steps];
bool sequenceFreqsIn[max_possible_sequence_steps];
bool sequencePowersIn[max_possible_sequence_steps];


// this is to hold all the data for the ramps
typedef struct {
	byte type_ramp; // is it a linear ramp or an exponential ramp
	float start_ramp;
	float stop_ramp;
	float time_ramp;
	float exponential_constant_ramp;
	float time_step_ramp;
} SW_ramp;

bool doSWrampsExist = false;
const unsigned int max_time_steps_SW_ramp = 1000;
unsigned int num_time_steps_SW_ramp[max_possible_sequence_steps];

// These two are to actually hold the numerical values to output during ramps
float SWrampdata_frequency[max_possible_sequence_steps][max_time_steps_SW_ramp];
float SWrampdata_power[max_possible_sequence_steps][max_time_steps_SW_ramp];

// This is to read in the transmitted data for software ramps (before processing it on Teensy to become the actual output arrays)
SW_ramp software_frequency_ramps[max_possible_sequence_steps]; // save the software ramps
bool software_frequency_rampsIn[max_possible_sequence_steps];
SW_ramp software_power_ramps[max_possible_sequence_steps]; // save the software ramps
bool software_power_rampsIn[max_possible_sequence_steps];


bool getHandshake();
void handleSerial(bool *handshakeState);
bool processSequence(int numSteps); // numSteps defines the number of active steps in the sequence, so the number of triggers actually
float receiveNumericalData();
void doSequenceOutput(int numSteps);
bool generateRampArrays(SW_ramp * my_sw_ramp, float * ramp_array, unsigned int * num_steps_this_ramp);
void process_SW_ramps(int numSteps);
void output_SW_Freq_ramp();
void output_SW_Power_ramp();
void myISR();
void fake_timer_function();
void fake_timer_function2();


void setup() {
	//GPIOD_PDDR |= (1<<3);
	pinMode(interruptPin,INPUT);
	pinMode(8,OUTPUT);


	SWramp_timer.priority(1);
	//attachInterrupt(digitalPinToInterrupt(interruptPin), myISR, LOW);
	digitalWrite(ssPin,HIGH); // This is to prepare the system for SPI communication later
	Serial.begin(57600);
	//while (!Serial);
	delay(500);
	//Serial.println("START");
	//Serial.println("");
	SPI.begin();
	SPI.beginTransaction(SPISettings(SPIclockspeed, MSBFIRST, SPI_MODE0));
/*
	DDS.setPower(1);	//(1) = DDS-ON , (0) = DDS-OFF Startup Default DDS = OFF
	delay(100);// let things get set up...

	DDS.reset();
	delay(100); // Give some time to let things get set up...
*/
	//digitalWrite(10, HIGH);  //Fix LOGIC analyzer Slave Select time out error.
	DDS.reset();
	DDS.initialize(400000000);	//initialize DDS

	DDS.setChargepump(1);	//0 = 75uA, 1 = 100uA, 2 = 125uA, 3 = 150uA
	DDS.initialize(20000000, 20);	// REFCLK (20Mhz) and the REFCLK multiplier (4 .. 20).
	DDS.setPower(1);
	delay(50);
	//DDS.setASF(100);	// Value between 0 .. 100


	/*
	Linear sweep mode
	You can also do a bi-directional linear sweep, using `DDS.linearSweep()`. The direction of the ramp is controlled by
	the `PS0` pin, so `PS0 HIGH` will cause it to do a positive ramp (limited by a final frequency), and `PS0 LOW` will make it
	ramp down (limited by a lower-bound frequency). Eg,
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
	//              frequency value in the ramp. SYNC_CLK operates at 1/4 of the SYSCLK clock value. Typically SYNC_CLK = 100MHz.
	//              Thus, the true "ramp rate" is, eg, posDF/(posRR*10 ns)
	//      negRR: same as above, but for negative ramp.
	//
	//      As a general rule, round up (not down) in calculating the delta frequency steps.
	//
	//      AD9954::linearSweep(freq0, freq1, posDF, posRR, negDF, negRR);

	*/
	//noInterrupts();

}

//void SysTick_Handler(void){}

void loop() {
	// The next stuff is for testing SystemCoreClock
	/*
	delay(1000);
	uint32_t ssc = SystemCoreClock;// this works on Arduino Zero, 48 MHz clock freq
	Serial.println(ssc);
	//Serial.println(SYST_CSR);
	uint32_t returnCode;

  returnCode = SysTick_Config(SystemCoreClock / 1000);
	Serial.println(returnCode);
	*/

		//digitalWrite(triggerPin,HIGH);

		//Serial.println("Inside while loop");
		//DDS.reset();
		/*
		DDS.setFreq(5000000);
		DDS.setASF(100);
		DDS.update();
		DDS.setFreq(1500000);
		DDS.setASF(50);
		DDS.update();
		DDS.setASF(0);
		DDS.update();
		*/
/*
		delay(2000);
		DDS.setFreq(85000000);
		DDS.setASF(100);
		DDS.update();
		delay(2000);
*/
/*
digitalWrite(pwrContrPin,HIGH);
delay(1000);
digitalWrite(pwrContrPin,LOW);
delay(100);
*/
		//Serial.println("inside the while loop");
		//digitalWrite(ssPin,HIGH);
		//GPIOD_PDOR |= (1<<3);
		//digitalWrite(8,LOW);
		//delay(1000);
		//GPIOD_PDOR &= ~(1<<3);
		//digitalWrite(ssPin,LOW);
		//digitalWrite(8,HIGH);




  /* // This stuff checks if the update bit has been set or not
  if (REG_PORT_OUT0 & ~(1 << 20)) {
      Serial.println("Bit is 0");
  }
  delay(1000);
  digitalWrite(6,HIGH);
  Serial.println("Written high");
  if (REG_PORT_OUT0 & (1 << 20)) {
      Serial.println("Bit is 1");
  }
  delay(1000);
  digitalWrite(6,LOW);
  Serial.println("Written low");
  */

  interruptCount = 0;
  handshakeState = getHandshake();
	handleSerial(&handshakeState);
	handshakeState = false;
	delay(10);

}

// getHandshake() simply establishes the first communication with Teensy
bool getHandshake(){
	if (Serial.available() > 0 && handshakeState == false) {
		incomingChar = Serial.read();
		if (incomingChar == 'h') {
			Serial.print('y');
			interruptTriggered = false;
			return true;
		}
		else {
			delay(20); // just to make sure that whatever needed to enter the buffer is there
			while (Serial.read() >= 0);
			Serial.print('n'); // only send it after the buffer has been cleared, otherwise information will be lost and there will be annoying bugs
			return false;
			}
	}
	else {
		return false;
	}
}


// This is essentially the main function of the whole program
void handleSerial(bool *handshakeState) {
	timeStart = millis(); // this is necessary to set the max time between the handshake and all information
	while (*handshakeState) {// only enter this loop if the handshake state is correct
		if (Serial.available() > 0) {
			incomingChar = Serial.read();
			switch (incomingChar) {

				case 'f': // 'f' -> frequency: single tone operation
					delay(1);
					//Serial.println("Trying to set frequency");
					while (Serial.available() > 0) {
						incomingChar = Serial.read(); // it returns -1 if nothing is available
							if (isDigit(incomingChar) || incomingChar == '.') {
								inStringFreq += incomingChar;
							}
							else { // meaning that wrong characters are being sent
								while (Serial.read() >= 0);
								inStringFreq = "";
								inStringFreqFloat = 0;
								Serial.print('n');
								*handshakeState = false;
								break;
							}
					}
					if (inStringFreq.length() > 0) {
						inStringFreqFloat = inStringFreq.toFloat();
						if (inStringFreqFloat >= lower_freq_lim && inStringFreqFloat <= upper_freq_lim) {

							DDS.setFreq(inStringFreqFloat);
							DDS.update();
							Serial.print('c'); // 'c' stands for "correct"
						}
						else { // meaning that the requested frequency is out of range
							Serial.print('f');
						}
						inStringFreq = "";
						inStringFreqFloat = 0;
					}
					else { // if there was nothing send after 'f'
						while (Serial.read() >= 0);
						Serial.print('n');
						inStringFreq = "";
						inStringFreqFloat = 0;
					}
					*handshakeState = false; // make sure to send handshake to false after an operation
					break;
				case 'p': // 'p' -> power: change power in this command
					delay(1);
					while (Serial.available() > 0) {
						incomingChar = Serial.read();
							if (isDigit(incomingChar) || incomingChar == '.') {
								inStringPower += incomingChar;
							}
							else {
								while (Serial.read() >= 0);
								inStringPower = "";
								inStringPowerFloat = 0;
								Serial.print('n');
								*handshakeState = false;
								break;
							}
					}
					if (inStringPower.length() > 0) {
						inStringPowerFloat = inStringPower.toFloat();
						if (inStringPowerFloat >= lower_power_lim && inStringPowerFloat <= upper_power_lim) {
							DDS.setASF(inStringPowerFloat);
							DDS.update();
							Serial.print('c');
						}
						else { // meaning that power is out of range
							Serial.print('p');
						}
						inStringPower = "";
						inStringPowerFloat = 0;
					}
					else { // if there was nothing sent after 'p'
						while (Serial.read() >= 0);
						Serial.print('n');
						inStringPower = "";
						inStringPowerFloat = 0;
					}
					*handshakeState = false;
					break;
				case 'd': // Not sure if this is necessary, but this is DDS power down
					DDS.setPower(0);
					*handshakeState = false;
					Serial.print('c');
					break;
				case 'u': // Not sure if this is necessary but this is DDS power up
					DDS.setPower(1);
					*handshakeState = false;
					Serial.print('c');
					break;
				case 's': // 's' stands for "sequence"
					delay(1);
					while (Serial.available() > 0) {
						incomingChar = Serial.read();
							if (isDigit(incomingChar)) {
								inStringStepsSequence += incomingChar; // so first get the number of steps in the sequence
							}
							else {
								while (Serial.read() >= 0);
								*handshakeState = false;
								Serial.print('n');
								break;
							}
					}
					if (inStringStepsSequence.length() > 0) {
						num_steps_total_sequence = inStringStepsSequence.toInt();
						if (num_steps_total_sequence > 0 && num_steps_total_sequence <= max_possible_sequence_steps) {
							Serial.print('c');
							inStringStepsSequence = "";
							isSequenceGood = processSequence(num_steps_total_sequence); // Make sure that this is correct, maybe let it output a bool
							process_SW_ramps(num_steps_total_sequence);
							if (isSequenceGood){
								doSequenceOutput(num_steps_total_sequence);
							}
							else {
								inStringStepsSequence = "";
								num_steps_total_sequence = 0;
								Serial.print('n');
								*handshakeState = false;
								Serial.print('n');
								break;
							}
						}
						else {
							while (Serial.read() >= 0);
							inStringStepsSequence = "";
							num_steps_total_sequence = 0;
							Serial.print('n');
							*handshakeState = false;
							break;
						}
					}
					else {
							while (Serial.read() >= 0);
							inStringStepsSequence = "";
							num_steps_total_sequence = 0;
							Serial.print('n');
							*handshakeState = false;
							break;
					}
					*handshakeState = false;
					inStringStepsSequence = "";
					num_steps_total_sequence = 0;
					break;
				default:
					delay(20);
					while (Serial.read() >= 0);
					Serial.println("x"); // output "x" signifies that wrong request was send in terms of mode
					*handshakeState = false;
					break;
			}
		}



		delay(5); // just to let it cool down a bit :)

		timeNow = millis();
		if (*handshakeState == true && timeNow - timeStart > timeoutms) {
			Serial.print('t'); //stands for "timeout"
			while (Serial.read() >= 0);
			*handshakeState = false;
			// now reinitialize all variables to defaults
		  inStringFreq = "";
		  inStringPower = "";
		  inStringStepsSequence = "";
		  inStringFreqFloat = 0;
			inStringPowerFloat = 0;
		  num_steps_total_sequence = 0;
			break;
		}
	}
}


bool processSequence(int numSteps) {
	float runningData; // This is the variable into which
	bool errorCondition = false;
	doSWrampsExist = false;
	char status[numBlocksSeqStep]; // this is basically what type of commands are in each step (void, simple step, ramp)
	delay(100); // Apparently this waiting time is really important

	for (byte i = 0; i < numSteps; ++i) { // num steps is in the total sequence, so the number of triggers
		delay(10);
		// first we receive the commands, of the form 'fplw', and respond with 'r' if all good
		for (byte j = 0; j < numBlocksSeqStep; ++j ) {
			if (Serial.available() > 0) { // it must be exactly the expected number of characters
				incomingChar = Serial.read();
			}
			else {
				incomingChar = 'q';
			}
			if (incomingChar == 'f' || incomingChar == 'p' || incomingChar == 'l' || incomingChar == 'w' || incomingChar == 'v') {
				status[j] = incomingChar;
				//Serial.print(status[j]);
				}
			else {
				while (Serial.read() >= 0);
				errorCondition = true;
				break;
			}
		}
		if (errorCondition) {
			Serial.print('n');
			return false;
		}
		else {
			Serial.print('r'); // stands for "received"
		}

		// Now we start filling the saved data buffers with the results of the commands
		// Do not send "vvvv"

		// NOTE!!! This is only good for 4 character status messages
		if (status[0] != 'v' && status[2] != 'v'){ // meaning, one cannot update a single freq and send a ramp at the same time
			errorCondition = true;
		}
		if (status[1] != 'v' && status[3] != 'v'){ // one cannot update the power and send a ramp at the same time
			errorCondition = true;
		}
		if (status[0] =='v' && status[1] == 'v' && status[2] =='v' && status[3] == 'v'){ // one cannot send "vvvv"
			errorCondition = true;
		}
		if (errorCondition) {
			Serial.print('n');
			return false;
		}


		if (status[0] == 'v' && status[2] == 'v' && i == 0) { // that is, either single freq or freq ramp must not be void
			errorCondition = true;
		}
		if (status[1] == 'v' && status[3] == 'v' && i == 0) { // either single power or power ramp must not be void
			errorCondition = true;
		}
		if (errorCondition) {
			Serial.print('n');
			return false;
		}

		if (status[0] == 'v') {
			sequenceFreqs[i] = 0;
			sequenceFreqsIn[i] = false;
		}
		else if (status[0] == 'f') {
			runningData = receiveNumericalData();
			if (runningData >= lower_freq_lim && runningData <= upper_freq_lim){
				sequenceFreqs[i] = runningData;
				sequenceFreqsIn[i] = true;
			}
			else {
				runningData = 0;
				Serial.print('f');
				return false;
			}
		}
		else {
			errorCondition = true;
		}

		if (status[1] == 'v') {
			sequencePowers[i] = 0;
			sequencePowersIn[i] = false;
		}
		else if (status[1] == 'p') {
			runningData = receiveNumericalData();
			if (runningData >= lower_power_lim && runningData <= upper_power_lim){
				sequencePowers[i] = runningData;
			}
			else {
				runningData = 0;
				Serial.print('p');
				return false;
			}
			sequencePowersIn[i] = true;
		}
		else {
			errorCondition = true;
		}
		if (errorCondition) {
			Serial.print('n');
			return false;
		}

	// process and save the frequency ramp
		if (status[2] == 'v') {
			software_frequency_ramps[i] = {2,0,0,0,0}; // this initializes it to a wrong value so that a correct value must be written before it can proceed
			software_frequency_rampsIn[i] = false;
		}
		else if (status[2] == 'w') {
			doSWrampsExist = true;
			software_frequency_rampsIn[i] = true;
			for (unsigned short q = 0; q < 5; q++) { // it's because there are 5 entries in the SW_ramp struct
				switch (q) {
					case 0: // this is type of ramp: 0 -> linear, 1 -> exponential
						runningData = (unsigned int) receiveNumericalData();
						if (runningData > 1) {
							Serial.print('n');
							return false; // this is an error, it should not be anything but 0 or 1
						}
						else {
							software_frequency_ramps[i].type_ramp = runningData;
							break;
						}
					case 1: // start value for ramp
						runningData = receiveNumericalData();
						if (runningData >= lower_freq_lim && runningData <= upper_freq_lim) {
							software_frequency_ramps[i].start_ramp = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 2: // stop value for ramp
						runningData = receiveNumericalData();
						if (runningData >= lower_freq_lim && runningData <= upper_freq_lim) {
							software_frequency_ramps[i].stop_ramp = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 3: // time duration of ramp
						runningData = receiveNumericalData();
						if (runningData > 0 && runningData <= software_ramp_time_limit) {
							software_frequency_ramps[i].time_ramp = runningData*ramp_time_fraction_from_requested;
							break;
						}
						else {
							Serial.print('n');
							return false;
						}
					case 4:
						runningData = receiveNumericalData(); // exponential constant, in case ramp is exponential
						software_frequency_ramps[i].exponential_constant_ramp = runningData;
						break;
					default:
						Serial.print('n');
						return false;
				}
			}
		}
		else {
			errorCondition = true;
		}

// Process and save the power ramp
		if (status[3] == 'v') {
			software_power_ramps[i] = {2,0,0,0,0};
			software_power_rampsIn[i] = false;
		}
		else if (status[3] == 'w') {
			doSWrampsExist = true;
			software_power_rampsIn[i] = true;
			for (unsigned short q = 0; q < 5; q++) { // it's because there are 5 entries in the SW_ramp struct
				switch (q) {
					case 0:
						runningData = (unsigned int) receiveNumericalData();
						if (runningData > 1) {
							Serial.print('n');
							return false; // this is an error, it should not be anything but 0 or 1
						}
						else {
							software_power_ramps[i].type_ramp = runningData;
							break;
						}
					case 1:
						runningData = receiveNumericalData();
						if (runningData >= lower_power_lim && runningData <= upper_power_lim) {
							software_power_ramps[i].start_ramp = runningData;
							break;
						}
						else {
							Serial.print('p');
							return false;
						}
					case 2:
						runningData = receiveNumericalData();
						if (runningData >= lower_power_lim && runningData <= upper_power_lim) {
							software_power_ramps[i].stop_ramp = runningData;
							break;
						}
						else {
							Serial.print('p');
							return false;
						}
					case 3:
						runningData = receiveNumericalData();
						if (runningData > 0 && runningData <= software_ramp_time_limit) {
							software_power_ramps[i].time_ramp = runningData*ramp_time_fraction_from_requested;
							break;
						}
						else {
							Serial.print('n');
							return false;
						}
					case 4:
						runningData = receiveNumericalData();
						software_power_ramps[i].exponential_constant_ramp = runningData;
						break;
					default:
						Serial.print('n');
						return false;
				}
			}
		}
		else {
			errorCondition = true;
		}

		if (errorCondition) {
			Serial.print('n');
			return false;
		}
	}
	return true;
}

// this function receives numerical data for the contents of steps and ramps in the sequence
float receiveNumericalData() {
	String inString = "";
	float inStringFloat = 0;
	delay(10);

	while (Serial.available() > 0) {
	incomingChar = Serial.read();
		if (isDigit(incomingChar) || incomingChar == '.') {
			inString += incomingChar;
		}
		else if (isSpace(incomingChar)) { // this must be the newline character that is sent
			break;
		}
		else {
			delay(10);
			while (Serial.read() >= 0);
			inString = "";
			inStringFloat = 0;
			break;
		}
	}
	if (inString.length() > 0) {
		inStringFloat = inString.toFloat();
		Serial.print('r');
		return inStringFloat;
	}
	else {
		while (Serial.read() >= 0);
		Serial.print('n');
		return -1;
	}

}


// this function generates the actual data arrays that produce software ramps.
// in other words, it evaluates the particular linear or exponential function that is given
// called from process_SW_ramps
bool generateRampArrays(SW_ramp * my_sw_ramp, float * ramp_array, unsigned int * num_steps_this_ramp) {
	unsigned int num_steps_arr;
	if (my_sw_ramp->type_ramp == 0) { // this is a linear ramp
		if (my_sw_ramp->time_ramp/(max_time_steps_SW_ramp - 1.) < software_ramp_min_timestep) {
			my_sw_ramp->time_step_ramp = software_ramp_min_timestep;
			num_steps_arr = (unsigned int) my_sw_ramp->time_ramp/software_ramp_min_timestep;
		}
		else {
			num_steps_arr = max_time_steps_SW_ramp - 1;
			my_sw_ramp->time_step_ramp = my_sw_ramp->time_ramp/(max_time_steps_SW_ramp - 1.);
		}
		*num_steps_this_ramp = num_steps_arr+1;
		float data_step = (my_sw_ramp->stop_ramp - my_sw_ramp->start_ramp)/(num_steps_arr);
		for (unsigned int i = 0; i < (num_steps_arr + 1); i++) {
			*(ramp_array + i) = (my_sw_ramp->start_ramp + i*data_step);
		}
		return true;
	}
	else if (my_sw_ramp->type_ramp == 1) { // this is an exponential ramp
		float data_value;
		if (my_sw_ramp->time_ramp/(max_time_steps_SW_ramp - 1.) < software_ramp_min_timestep) {
			my_sw_ramp->time_step_ramp = software_ramp_min_timestep;
			num_steps_arr = (unsigned int) my_sw_ramp->time_ramp/software_ramp_min_timestep;
		}
		else {
			num_steps_arr = max_time_steps_SW_ramp - 1;
			my_sw_ramp->time_step_ramp = my_sw_ramp->time_ramp/(max_time_steps_SW_ramp - 1.);
		}
		*num_steps_this_ramp = num_steps_arr+1;
		float prefactor = my_sw_ramp->start_ramp - my_sw_ramp->stop_ramp;
		for (unsigned int i = 0; i < (num_steps_arr + 1); i++) {
			data_value = prefactor*exp(-(0+i*my_sw_ramp->time_step_ramp)/my_sw_ramp->exponential_constant_ramp) + my_sw_ramp->stop_ramp;
			*(ramp_array + i) = data_value;
		}
		return true;
	}
	else {
		return false;
	}
}



void process_SW_ramps(int numSteps){
	//bool ramp_generated_well = true;
	float temporary_data_arr[max_time_steps_SW_ramp];

	if (doSWrampsExist) {
		doSWrampsExist = false; // it has to be reset so that next time it doesn't automatically appear
		for (int i = 0; i < numSteps; i++) {
			if (software_frequency_rampsIn[i]) {
				generateRampArrays(&software_frequency_ramps[i],&temporary_data_arr[0],&num_steps_in_ramp[i]);
				for (unsigned int q = 0; q < num_steps_in_ramp[i]; q++) {
					SWrampdata_frequency[i][q] = temporary_data_arr[q];
				}
			}
			else if (software_power_rampsIn[i]) {
				generateRampArrays(&software_power_ramps[i],&temporary_data_arr[0],&num_steps_in_ramp[i]);
				for (unsigned int q = 0; q< num_steps_in_ramp[i]; q++) {
					SWrampdata_power[i][q] = temporary_data_arr[q];
				}
			}
		}
	}
}



void doSequenceOutput(int numSteps) {
	attachInterrupt(digitalPinToInterrupt(interruptPin), myISR, RISING);
	delay(50);
	//SWramp_timer.begin(fake_timer_function,1000000.); // this fake function works, so the timer must be working fine
	for (byte i = 0; i < numSteps; ++i) {
		current_loop_number = i;
		if (sequenceFreqsIn[i]) {
			DDS.setFreq(sequenceFreqs[i]);
			//delay(100);
		}
		if (sequencePowersIn[i]) {
			DDS.setASF(sequencePowers[i]);
			//delay(100);
		}
		start_time_interrupts = millis();

		while (true) {
			time_now_interrupts = millis();
			if (abs(time_now_interrupts - start_time_interrupts) > max_time_interrupts) {
				i = numSteps;
				break;
			}
			//delay(1);
			if (interruptTriggered) {
				//SWramp_timer.end();
				if (software_frequency_rampsIn[i]) {
					//DDS.setFreq(SWrampdata_frequency[i][0]); // the first data point
					//DDS.update();
					IntervalTimerCounter = 0;
					SWramp_timer.begin(output_SW_Freq_ramp,software_frequency_ramps[i].time_step_ramp); // it's because it's in microseconds
					//output_SW_Freq_ramp();
				}
				else if (software_power_rampsIn[i]) {
					//DDS.setASF(SWrampdata_power[i][0]); // the first data point
					//DDS.update();
					IntervalTimerCounter = 0;
					//output_SW_Power_ramp();
					//IntervalTimerCounter = 1;
					//SWramp_timer.begin(output_SW_Power_ramp,software_ramp_timestep*1000.);
					isTimerStarted = SWramp_timer.begin(fake_timer_function,software_ramp_min_timestep*1000.);
					// so this does start the timer
					if (isTimerStarted) {
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
					}
					if (!isTimerStarted){
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
						digitalWrite(8,HIGH);
						delay(500);
						digitalWrite(8,LOW);
						delay(500);
					}

				}
				interruptTriggered = false;
				break;
			}
		}
	}
	detachInterrupt(digitalPinToInterrupt(interruptPin));
}


void myISR() {
	DDS.update();
	interruptCount += 1;
	interruptTriggered = true;
}

void output_SW_Freq_ramp() {
	if (IntervalTimerCounter < num_steps_in_ramp[current_loop_number]) {
		DDS.setFreq(SWrampdata_frequency[current_loop_number][IntervalTimerCounter]);
		DDS.update();
		IntervalTimerCounter += 1;
	}
	else {
		SWramp_timer.end();
	}
}

void output_SW_Power_ramp() {
	if (IntervalTimerCounter < num_steps_in_ramp[current_loop_number]) {
		//DDS.setFreq(70000000+10000000*(IntervalTimerCounter%2));
		DDS.setASF(SWrampdata_power[current_loop_number][IntervalTimerCounter]);
		DDS.update();
		IntervalTimerCounter += 1;
	}
	else {
		SWramp_timer.end();
	}
}

void fake_timer_function() {}

void fake_timer_function2() {
	if (IntervalTimerCounter%2 > 0) {
		digitalWrite(8,HIGH);
	}
	else {
		digitalWrite(8,LOW);
	}
	IntervalTimerCounter++;
	if (IntervalTimerCounter>10) {
		SWramp_timer.end();
	}
}

/*
void SysTick_Handler(void) {

}
*/
