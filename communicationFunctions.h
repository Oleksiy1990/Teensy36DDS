#ifndef communicationFunctions_h
#define communicationFunctions_h

#include "Arduino.h"

bool getHandshake(volatile bool *interruptTriggered, bool handshakeState);
bool getOneDatapoint(float *result, float lower_lim, float upper_lim);
float receiveNumericalData();

#endif
