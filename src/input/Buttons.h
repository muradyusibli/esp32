#pragma once

#include <Arduino.h>

struct DebouncedInput {
    bool lastReading   = false;
    bool stablePressed = false;
    unsigned long lastChangeAt = 0;
};

bool isButtonPressed(int pin);
bool updateDebouncedInput(int pin, DebouncedInput& state, bool& newStableState);

void requestSocketButtonChange(bool state);
void processSocketButtonRequest();
void printButtonDebug();
