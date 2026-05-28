#include "hardware/Pins.h"
#include "app/Settings.h"

const int ledPins[]       = { 16, 17, 25, 26 };
const int numberOfLeds    = 4;
const int keypadRowPins[] = { 18, 19, 32, 33 };
const int keypadColPins[] = { 4, 13, 14, 15 };


const int wakeButtonPins[] = { 13, 4, 14, 15 };
const int numberOfWakeButtons = sizeof(wakeButtonPins) / sizeof(wakeButtonPins[0]);

const int randomBlinkCount = 4;
const int randomOnMs       = 250;
const int randomOffMs      = 250;
const int blinkDelay       = 250;

const char* request_url = "https://websocket.itip-demo.ucll.cloud/demotext.txt";
