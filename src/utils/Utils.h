#pragma once

#include <Arduino.h>

void printOutput(const String& output);
String payloadToString(uint8_t* payload, size_t length);
bool timeReached(unsigned long targetTime);
