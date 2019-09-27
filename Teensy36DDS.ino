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
#include "communicationFunctions.h"

// this is the pin numbering on the
const byte ssPin = 10;
const byte resetPin = 24;
const byte updatePin = 5;
const byte ps0Pin = 25;
const byte ps1Pin = 26;
const byte oskPin = 27;
const byte pwrContrPin = 28;
//const byte interruptPin = 9; // NOTE!!! This is NOT on DDS3 and DDS4
const byte interruptPin = 2; //  This is on DDS3 and DDS4!!!

const unsigned long SPIclockspeed = 1000000;

AD9954 DDS(ssPin, resetPin, updatePin, ps0Pin, ps1Pin, oskPin, pwrContrPin, true/*externalUpdate*/);
IntervalTimer SWramp_timer; // The timer to produce schedules interrups that define a software ramp
//AD9954 DDS(10, 24, 5, 25, 26, 27, 28, true);
const unsigned long lower_freq_lim = 1000000;
const unsigned long upper_freq_lim = 160000000;
const float lower_HW_ramp_timelimit = 1; // shortest HW ramp time, given in microseconds
const float higher_HW_ramp_timelimit = 1e7; // longest HW ramp time, given in microseconds
const float lower_power_lim = 0;
const float upper_power_lim = 100;
const float software_ramp_time_limit = 15000; // 15000 ms = 15 s for now
const float software_ramp_min_timestep = 0.1; // 0.1 ms = 200 micros , given in ms
const float ramp_time_fraction_from_requested = 0.9; // this is to make sure that the ramp ends before the requested time so
// that the next trigger does not appear before the ramp is done and the timer is stopped
// VERY IMPORTANT! Pay attention to this ramp_time_fraction_from_requested, otherwise timing bugs will result!

bool handshakeState = false;
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
unsigned long max_time_interrupts = 20000; // let's set it for 20 s for now


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

// this is to hold all the data for linear ramps (hardware)
typedef struct {
	unsigned long freq0;
	unsigned long freq1;
	float posTimeMicros;
	float negTimeMicros;
	float waitingTimeMicros;
	float power;
	byte mode;
	bool noDwell;
} linear_ramp;

bool doSWrampsExist = false;
bool doLinearRampsExist = false;
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

// This is to read in transmitted data for hardware linear frequency ramps
linear_ramp linear_HW_frequency_ramps[max_possible_sequence_steps];
bool linear_HW_frequency_rampsIn[max_possible_sequence_steps];


void handleSerial(bool *handshakeState);
bool processSequence(int numSteps); // numSteps defines the number of active steps in the sequence, so the number of triggers actually
float receiveNumericalData();
void doSequenceOutput(int numSteps);
bool generateRampArrays(SW_ramp * my_sw_ramp, float * ramp_array, unsigned int * num_steps_this_ramp);
void process_SW_ramps(int numSteps);
void output_SW_Freq_ramp();
void output_SW_Power_ramp();
void myISR();


void setup() {
	//GPIOD_PDDR |= (1<<3);
	pinMode(interruptPin,INPUT);
	//pinMode(8,OUTPUT);


	SWramp_timer.priority(10);
	//attachInterrupt(digitalPinToInterrupt(interruptPin), myISR, RISING);
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



	// The DDS linear sweep directly is only for testing the linear sweeps outside of the control system
	//DDS.linearSweep(5000000, 7000000, 2, 2, 2, 100, 0, false);

  interruptCount = 0;
  handshakeState = getHandshake(&interruptTriggered,handshakeState);
	handleSerial(&handshakeState);
	handshakeState = false;


	// Possibly reset the DDS, if necessary. The problem is that then it refreshes everything
	/*
	DDS.reset();
	DDS.initialize(400000000);	//initialize DDS
	DDS.setChargepump(1);	//0 = 75uA, 1 = 100uA, 2 = 125uA, 3 = 150uA
	DDS.initialize(20000000, 20);	// REFCLK (20Mhz) and the REFCLK multiplier (4 .. 20).
	DDS.setPower(1);
	*/
	delay(100);

}

// This is essentially the main function of the whole program
void handleSerial(bool *handshakeState) {
	char incomingChar;
	timeStart = millis(); // this is necessary to set the max time between the handshake and all information
	while (*handshakeState) {// only enter this loop if the handshake state is correct
		if (Serial.available() > 0) {
			incomingChar = Serial.read();
			switch (incomingChar) {
				// 'f' means get frequency information, single tone operation
				case 'f':
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
				// 'p' means get power information, single tone operation
				case 'p':
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

				// Not sure if this is necessary, but this is DDS power down
				case 'd':
					DDS.setPower(0);
					*handshakeState = false;
					Serial.print('c');
					break;

				// Not sure if this is necessary but this is DDS power up
				case 'u':
					DDS.setPower(1);
					*handshakeState = false;
					Serial.print('c');
					break;

				// Receive a sequence, 's' stands for "sequence"
				case 's':
					delay(1);

					// get the number of sequence steps, return 'c' if it's been requested
					// correctly, and then call processSequence(...) function,
					// then process_SW_ramps(...), and doSequenceOutput(...)
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
							else { // this will run if isSequenceGood is false
								inStringStepsSequence = "";
								num_steps_total_sequence = 0;
								Serial.print('n');
								*handshakeState = false;
								break;
							}
						}
						else { // this will run if num_steps_total_sequence is out of limits
							while (Serial.read() >= 0);
							inStringStepsSequence = "";
							num_steps_total_sequence = 0;
							Serial.print('n');
							*handshakeState = false;
							break;
						}
					}
					else { // This will run if no data is sent over Serial to communicate sequence length
							while (Serial.read() >= 0);
							inStringStepsSequence = "";
							num_steps_total_sequence = 0;
							Serial.print('n');
							*handshakeState = false;
							break;
					}
					// the next 4 lines are still part of case 's'
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

		/*
		This last part of the function checks if the communication has
		not timed out, and if it did, and the handshake is still hanging on true,
		it resets to false, sends 't' on Serial (for "timeout"), clears out the serial
		buffer, resets variables to defaults, and breaks the loop
		*/
		timeNow = millis();
		if (*handshakeState == true && timeNow - timeStart > timeoutms) {
			Serial.print('t'); //stands for "timeout"
			while (Serial.read() >= 0);
			*handshakeState = false;
		  inStringFreq = "";
		  inStringPower = "";
		  inStringFreqFloat = 0;
			inStringPowerFloat = 0;
		  inStringStepsSequence = "";
		  num_steps_total_sequence = 0;
			break;
		}
	} // this is end of while(*handshakeState) loop
} // end of handleSerial(...) function


bool processSequence(int numSteps) {
	char incomingChar;
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

/*
Process and save the frequency ramp
This can be either a software-defined ramp or a hardware-defined ramp, so
the options are 'v', 'w', 'l' ('l' is the linear hardware ramp)
*/
		if (status[2] == 'v') {
			software_frequency_ramps[i] = {2,0,0,0,0}; // this initializes it to a wrong value so that a correct value must be written before it can proceed
			software_frequency_rampsIn[i] = false;
			linear_HW_frequency_rampsIn[i] = false; // initialize to false at that sequence step
			linear_HW_frequency_ramps[i] = {0,0,0,0,0,0,0,false}; // just initializing everything to 0
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
		/*
		The data has to be sent in the form
		1) lower freq (Hz)
		2) upper freq (Hz)
		3) time to go up (micros)
		4) time to go down (micros)
		5) time to wait at higher frequency (micros)
		6) power (float)
		7) mode (integer)
		8) noDwell value (integer): 0 -> false, 1 -> true
		*/


		else if (status[2] == 'l') {
			doLinearRampsExist = true;
			linear_HW_frequency_rampsIn[i] = true;
			for (unsigned short q = 0; q < 8; q++) { // it's because there are 7 entries in the linear_HW_frequency_ramps struct
				switch (q) {
					case 0: // start frequency for HW ramp
						runningData = receiveNumericalData();
						if (runningData >= lower_freq_lim && runningData <= upper_freq_lim) {
							linear_HW_frequency_ramps[i].freq0 = (unsigned long) runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 1: // stop frequency for HW ramp
						runningData = receiveNumericalData();
						if (runningData >= lower_freq_lim && runningData <= upper_freq_lim) {
							linear_HW_frequency_ramps[i].freq1 = (unsigned long) runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 2: // positive edge ramp time, microseconds
						runningData = receiveNumericalData();
						if (runningData >= lower_HW_ramp_timelimit && runningData <= higher_HW_ramp_timelimit) {
							linear_HW_frequency_ramps[i].posTimeMicros = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 3: // negative edge ramp time. microseconds
						runningData = receiveNumericalData();
						if (runningData >= lower_HW_ramp_timelimit && runningData <= higher_HW_ramp_timelimit) {
							linear_HW_frequency_ramps[i].negTimeMicros = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 4: // waiting time between positive and negative ramp, in microseconds
						runningData = receiveNumericalData();
						if (runningData >= lower_HW_ramp_timelimit && runningData <= higher_HW_ramp_timelimit) {
							linear_HW_frequency_ramps[i].waitingTimeMicros = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 5: // waiting time between positive and negative ramp, in microseconds
						runningData = receiveNumericalData();
						if (runningData >= lower_power_lim && runningData <= upper_power_lim) {
							linear_HW_frequency_ramps[i].power = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 6:
						runningData = (unsigned int) receiveNumericalData();
						if (runningData >= 0 && runningData <= 5) { // depends on how many options there are
							linear_HW_frequency_ramps[i].mode = runningData;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					case 7:
						runningData = (unsigned int) receiveNumericalData();
						if (runningData == 0) {
							linear_HW_frequency_ramps[i].noDwell = false;
							break;
						}
						else if (runningData == 1) {
							linear_HW_frequency_ramps[i].noDwell = true;
							break;
						}
						else {
							Serial.print('f');
							return false;
						}
					default:
						Serial.print('n');
						return false;
				}
			}
		}
		else {
			errorCondition = true;
		}

/*
Process and save the power ramp
This can only be a software ramp, hardware power ramps are not supported
So the only options are 'v' and 'w'
*/
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
	char incomingChar;
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


/*
The next function is to generate the numerical values of the frequencies and powers
that will be output in software-defined ramps
*/
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
	//digitalWrite(8,LOW);
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
				//SWramp_timer.end(); // This could possibly be important but for now we don't use it just for faster timing
				if (software_frequency_rampsIn[i]) {
					IntervalTimerCounter = 1;
					// the first point has to be sent immediately, otherwise it waits before the 1st tick of the timer
					DDS.setFreq(SWrampdata_frequency[current_loop_number][0]); // the first data point
					DDS.update();
					SWramp_timer.begin(output_SW_Freq_ramp,software_frequency_ramps[i].time_step_ramp*1000.); // it's because it's in microseconds
					delay(software_frequency_ramps[i].time_ramp + 3*software_frequency_ramps[i].time_step_ramp);
					/*
					while (IntervalTimerCounter < num_steps_in_ramp[current_loop_number]) {
						delay(software_power_ramps[i].time_step_ramp*3.); // this is an arbitrary number
					}
					*/
				}
				else if (software_power_rampsIn[i]) {
					IntervalTimerCounter = 1;
					//digitalWrite(8,HIGH);
					// the first point has to be sent immediately, otherwise it waits before the 1st tick of the timer
					DDS.setASF(SWrampdata_power[current_loop_number][0]);
					DDS.update();
					SWramp_timer.begin(output_SW_Power_ramp,software_power_ramps[i].time_step_ramp*1000.);
					delay(software_power_ramps[i].time_ramp + 3*software_power_ramps[i].time_step_ramp); // adding the 3 time step ramps at the end just to wait that it really finished
					/*
					while (IntervalTimerCounter < num_steps_in_ramp[current_loop_number]) {
						delay(software_power_ramps[i].time_step_ramp*3.); // this is an arbitrary number
					}
					*/
					//delay(software_frequency_ramps[i].time_ramp + 5);
					/*
					if (current_loop_number == (numSteps - 1)) {
						delay(software_frequency_ramps[i].time_ramp + 5); // if the ramp is the last step, wait until it's done
					}
					*/
					// so this does start the timer
					// NOTE! First trigger comes after the first cycle of the timer, NOT immediately at the beginning
					// And it looks like it does start the timer function too!!!

				}
				else if (linear_HW_frequency_rampsIn[i]) {
					//float totaltime = linear_HW_frequency_ramps[i].posTimeMicros+linear_HW_frequency_ramps[i].negTimeMicros+
					//linear_HW_frequency_ramps[i].waitingTimeMicros;
					DDS.linearSweep(linear_HW_frequency_ramps[i].freq0,linear_HW_frequency_ramps[i].freq1,
					linear_HW_frequency_ramps[i].posTimeMicros,linear_HW_frequency_ramps[i].negTimeMicros,
					linear_HW_frequency_ramps[i].waitingTimeMicros,linear_HW_frequency_ramps[i].power,
					linear_HW_frequency_ramps[i].mode,linear_HW_frequency_ramps[i].noDwell);
					//delay(totaltime*1000 + 1); // the delay should already be implemented in the linearSweep function itself
				}

				interruptTriggered = false;
				break;
			}
		}
	}
	detachInterrupt(digitalPinToInterrupt(interruptPin));
}


void myISR() {
	//Serial.println("Interrupt triggered");
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
		DDS.setASF(SWrampdata_power[current_loop_number][IntervalTimerCounter]);
		DDS.update();
		IntervalTimerCounter += 1;
	}
	else {
		SWramp_timer.end();
	}
}
/*
void fake_timer_function() {
	if (IntervalTimerCounter%2 > 0.5) {
		digitalWrite(8,HIGH);
	}
	else {
		digitalWrite(8,LOW);
	}
	IntervalTimerCounter++;
	if (IntervalTimerCounter>50) {
		SWramp_timer.end();
	}

}

void fake_timer_function2() {
	digitalWrite(8,LOW);
}
*/


/*
void SysTick_Handler(void) {

}
*/

// Some debug code in case it's needed
					/*
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
					*/
