#include "communicationFunctions.h"
#include "Arduino.h"


/*
getHandshake(bool *interruptTriggered, bool handshakeState)
establishes the first communication with Teensy
get 'h' and respond with 'y' if it's all good, 'n' if there's a problem
return true if good, false if bad
set interruptTriggered to false if correct handshake
*/
bool getHandshake(volatile bool *interruptTriggered, bool handshakeState){
  char incomingChar;
	if (Serial.available() > 0 && handshakeState == false) {
		incomingChar = Serial.read();
		if (incomingChar == 'h') {
			Serial.print('y');
			*interruptTriggered = false;
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

bool getOneDatapoint(float *result, float lower_lim, float upper_lim) {
          char incomingChar;
          String inStringData = "";
          float inStringDataFloat;
					while (Serial.available() > 0) {
						incomingChar = Serial.read(); // it returns -1 if nothing is available
							if (isDigit(incomingChar) || incomingChar == '.') {
								inStringData += incomingChar;
							}
							else { // meaning that wrong characters are being sent
								while (Serial.read() >= 0);
								inStringData = "";
								*result = 0;
								Serial.print('n');
                return false; // stops the function here and returns false
              }
					}
					if (inStringData.length() > 0) {
						inStringDataFloat = inStringData.toFloat();
						if (inStringDataFloat >= lower_lim && inStringDataFloat <= upper_lim) {
							Serial.print('c'); // 'c' stands for "correct"
              *result = inStringDataFloat;
              return true; // This means that everything went correctly
						}
						else { // meaning that the requested frequency is out of range
							Serial.print('n');
              *result = 0;
              return false;
						}
					}
          else { // basically if inStringData length = 0, so no data
						Serial.print('n');
            *result = 0;
            return false;
          }
}

// this function receives numerical data for the contents of steps and ramps in the sequence
// if the numerical data has been received correctly and a float has been produced, return "r" via Serial
// if there is an error, return -1 and send "n" via Serial
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
		Serial.print('r'); // this means "received" and so the operation went correctly
		return inStringFloat;
	}
	else {
		while (Serial.read() >= 0);
		Serial.print('n'); // this means "no" and so something failed in the operation
		return -1;
	}

}
