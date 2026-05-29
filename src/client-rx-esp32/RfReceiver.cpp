#include "client-rx-esp32/RfReceiver.h"

#include <Arduino.h>
#include <RCSwitch.h>
#include "app/Settings.h"
#include "power/PowerManager.h"


#define PIN_MOTOR             12
#define PIN_LED                2
#define MOTOR_ON_DURATION_MS 500

static RCSwitch mySwitch = RCSwitch();
static unsigned long motorStartedAt = 0;
static bool          motorRunning   = false;

void initRfReceiver() {
    pinMode(PIN_MOTOR, OUTPUT);
    pinMode(PIN_LED,   OUTPUT);

    digitalWrite(PIN_MOTOR, LOW);
    digitalWrite(PIN_LED,   LOW);

    mySwitch.enableReceive(RF_RECEIVER_PIN);

    Serial.println("[RF] Receiver ready on GPIO 35.");
}


bool updateRfReceiver() {
    bool validCodeReceived = false;

    if (mySwitch.available()) {
        unsigned long receivedCode = mySwitch.getReceivedValue();

        if (receivedCode == 46726) {
            Serial.print("[RF] Received code: ");
            Serial.print(receivedCode);
            Serial.print(" / bitlength: ");
            Serial.print(mySwitch.getReceivedBitlength());
            Serial.print(" / protocol: ");
            Serial.println(mySwitch.getReceivedProtocol());

            markControllerActivity();  // reset inactivity timer

            digitalWrite(PIN_MOTOR, HIGH);
            digitalWrite(PIN_LED,   HIGH);
            motorStartedAt     = millis();
            motorRunning       = true;
            validCodeReceived  = true;
        }

        mySwitch.resetAvailable();
    }

    if (motorRunning && (millis() - motorStartedAt >= MOTOR_ON_DURATION_MS)) {
        digitalWrite(PIN_MOTOR, LOW);
        digitalWrite(PIN_LED,   LOW);
        motorRunning = false;
    }

    return validCodeReceived;
}



void enableRfReceive() {
    mySwitch.enableReceive(RF_RECEIVER_PIN);
}

void disableRfReceive() {
    mySwitch.disableReceive();
}