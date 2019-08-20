
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
