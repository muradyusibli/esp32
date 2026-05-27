#include "input/Buttons.h"

#include "app/Settings.h"
#include "app/State.h"
#include "hardware/OledDisplay.h"
#include "hardware/Pins.h"
#include "power/PowerManager.h"

bool isButtonPressed(int pin) {
    return digitalRead(pin) == LOW;
}

bool updateDebouncedInput(int pin, DebouncedInput& state, bool& newStableState) {
    bool currentReading = isButtonPressed(pin);
    unsigned long now = millis();

    if (currentReading != state.lastReading) {
        state.lastReading  = currentReading;
        state.lastChangeAt = now;
    }

    if ((now - state.lastChangeAt) >= BUTTON_DEBOUNCE_MS) {
        if (currentReading != state.stablePressed) {
            state.stablePressed = currentReading;
            newStableState = currentReading;
            return true;
        }
    }

    return false;
}

void requestSocketButtonChange(bool state) {
    portENTER_CRITICAL(&sharedMux);
    pendingSocketButtonState  = state;
    socketButtonChangeRequest = true;
    portEXIT_CRITICAL(&sharedMux);
}

void processSocketButtonRequest() {
    bool hasChange = false;
    bool newState = false;

    portENTER_CRITICAL(&sharedMux);
    if (socketButtonChangeRequest) {
        newState = pendingSocketButtonState;
        socketButtonChangeRequest = false;
        hasChange = true;
    }
    portEXIT_CRITICAL(&sharedMux);

    if (!hasChange) return;

    socketButtonState = newState;
    socketButtonStateDirty = true;
    markControllerActivity();

    Serial.printf("SOCKET BUTTON %s\n", socketButtonState ? "PRESSED" : "RELEASED");
    displayMessage(socketButtonState ? "Socket pressed" : "Socket released");
}

void printButtonDebug() {
#if DEBUG_BUTTONS
    static unsigned long last = 0;
    if (millis() - last < 1000) return;
    last = millis();

    Serial.printf("[BTN] random=%d socket=%d manual=%d\n",
        digitalRead(RANDOM_BUTTON_PIN), digitalRead(SOCKET_BUTTON_PIN),
        (int)manualControlActive);
#endif
}