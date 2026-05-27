#pragma once

// LED color mapping:
//   index 0 = green  (GPIO 16)
//   index 1 = yellow (GPIO 17)
//   index 2 = red    (GPIO 25)
//   index 3 = blue   (GPIO 26)
// Backend sends integers 1-4; code subtracts 1 to get the index into ledPins[].

extern const int ledPins[];
extern const int numberOfLeds;
extern const int ledButtonPins[];

extern const int keypadRowPins[];
extern const int keypadColPins[];
// In Pins.h — add with your other pin constants
constexpr int MPU_ADDR = 0x68;

extern const int wakeButtonPins[];
extern const int numberOfWakeButtons;

extern const int randomBlinkCount;
extern const int randomOnMs;
extern const int randomOffMs;
extern const int blinkDelay;

extern const char* request_url;
