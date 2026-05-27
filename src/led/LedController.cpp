#include "led/LedController.h"

#include <Arduino.h>

#include "app/Settings.h"
#include "app/State.h"
#include "hardware/OledDisplay.h"
#include "hardware/Pins.h"
#include "input/Buttons.h"
#include "power/PowerManager.h"
#include "utils/Utils.h"
#include "input/Keypad.h"
#include "network/NetworkManager.h"  // ← for sendKeypressEvent()
static DebouncedInput randomButtonDebounce;
static DebouncedInput socketButtonDebounce;

// Animation state only touched by ledButtonTask.
static bool randomLedIsOn = false;
static int randomBlinkPosition = 0;
static int randomCurrentLed = -1;
static unsigned long randomNextChangeAt = 0;

static bool backendLedIsOn = false;
static int backendSequence[MAX_BACKEND_SEQUENCE_STEPS];
static int backendSequenceLength = 0;
static int backendSequencePosition = 0;
static int backendSequenceDelayMs = blinkDelay;
static unsigned long backendNextChangeAt = 0;

void initControllerPins() {
    for (int i = 0; i < numberOfLeds; i++) {
        pinMode(ledPins[i], OUTPUT);
        digitalWrite(ledPins[i], LOW);
    }



    pinMode(RANDOM_BUTTON_PIN, INPUT_PULLUP);
    pinMode(SOCKET_BUTTON_PIN, INPUT_PULLUP);

    randomSeed(analogRead(34));
}

void turnAllLedsOff() {
    for (int i = 0; i < numberOfLeds; i++) {
        digitalWrite(ledPins[i], LOW);
    }
}

void clearPendingBackendRequest() {
    portENTER_CRITICAL(&sharedMux);
    backendLoadRequest = false;
    pendingBackendSequenceLength = 0;
    portEXIT_CRITICAL(&sharedMux);
}

static bool takeBackendSequenceRequest(int* seqOut, int& lenOut, int& delayOut) {
    bool ok = false;

    portENTER_CRITICAL(&sharedMux);
    if (backendLoadRequest) {
        lenOut = pendingBackendSequenceLength;
        delayOut = pendingBackendSequenceDelayMs;
        for (int i = 0; i < lenOut; i++) seqOut[i] = pendingBackendSequence[i];
        backendLoadRequest = false;
        ok = true;
    }
    portEXIT_CRITICAL(&sharedMux);

    return ok;
}

void stopLedTaskAnimations(bool turnOffLeds) {
    randomSequenceActive = false;
    randomLedIsOn = false;
    randomBlinkPosition = 0;
    randomCurrentLed = -1;

    backendSequenceActive = false;
    backendLedIsOn = false;
    backendSequencePosition = 0;

    // Reset keypad rows to idle state before suspend
    for (int r = 0; r < 4; r++) {
        digitalWrite(keypadRowPins[r], HIGH);
    }

    if (turnOffLeds) turnAllLedsOff();
}

static void startRandomSequenceFromLedTask() {
    stopLedTaskAnimations(true);

    randomSequenceActive = true;
    randomLedIsOn = false;
    randomBlinkPosition = 0;
    randomCurrentLed = -1;
    randomNextChangeAt = millis();

    Serial.println("RANDOM BUTTON PRESSED");
    requestDisplayMessage("Random LEDs");
}

static void updateRandomSequenceFromLedTask() {
    if (!randomSequenceActive) return;

    unsigned long now = millis();
    if (!timeReached(randomNextChangeAt)) return;

    if (!randomLedIsOn) {
        if (randomBlinkPosition >= randomBlinkCount) {
            randomSequenceActive = false;
            randomCurrentLed = -1;
            turnAllLedsOff();
            Serial.println("Random sequence finished");
            requestDisplayMessage("Random done");
            return;
        }

        randomCurrentLed = random(0, numberOfLeds);
        turnAllLedsOff();
        digitalWrite(ledPins[randomCurrentLed], HIGH);
        Serial.printf("Random LED: %d\n", randomCurrentLed + 1);

        randomLedIsOn = true;
        randomNextChangeAt = now + randomOnMs;
    } else {
        digitalWrite(ledPins[randomCurrentLed], LOW);
        randomLedIsOn = false;
        randomBlinkPosition++;
        randomNextChangeAt = now + randomOffMs;
    }
}

static void startBackendSequenceFromLedTask(const int* sequence, int length, int delayMs) {
    stopLedTaskAnimations(true);

    backendSequenceLength = length;
    backendSequencePosition = 0;
    backendSequenceDelayMs = delayMs;
    backendLedIsOn = false;

    for (int i = 0; i < length; i++) backendSequence[i] = sequence[i];

    backendSequenceActive = (length > 0);
    backendNextChangeAt = millis();

    if (backendSequenceActive) {
        Serial.printf("[IOc] backend sequence: %d steps @ %d ms\n", length, delayMs);
        requestDisplayMessage("Watch LEDs!");
    }
}

static void updateBackendSequenceFromLedTask() {
    if (!backendSequenceActive) return;

    unsigned long now = millis();
    if (!timeReached(backendNextChangeAt)) return;

    if (!backendLedIsOn) {
        if (backendSequencePosition >= backendSequenceLength) {
            backendSequenceActive = false;
            turnAllLedsOff();
            Serial.println("[IOc] sequence done — setting espDonePending");

            // Signal main loop to call sendEspDone().
            // Do not call socketIO from this task.
            portENTER_CRITICAL(&sharedMux);
            espDonePending = true;
            portEXIT_CRITICAL(&sharedMux);
            return;
        }

        int ledIndex = backendSequence[backendSequencePosition];
        turnAllLedsOff();
        digitalWrite(ledPins[ledIndex], HIGH);
        Serial.printf("[IOc] LED %d ON\n", ledIndex + 1);

        backendLedIsOn = true;
        backendNextChangeAt = now + backendSequenceDelayMs;
    } else {
        turnAllLedsOff();
        backendLedIsOn = false;
        backendSequencePosition++;
        backendNextChangeAt = now + 80; // gap between LEDs
    }
}

void ledButtonTask(void* parameter) {
    (void)parameter;
    bool previousManual = false;

    while (true) {

       char key = keypadScan();
        bool anyManual = (key >= 'A' && key <= 'D');

        if (anyManual) markControllerActivity();

        // ── Send keypress to backend on key-down edge only ─────────────────
        // `previousKey` tracks the last scan result so we fire exactly once
        // per press, not once per scan tick (~5 ms).
        static char previousKey = 0;
        if (key != 0 && key != previousKey) {
            // Any key on the matrix — digits, A-D, *, # — is forwarded.
            // The frontend useNumpad hook filters to the keys it cares about.
            sendKeypressEvent(key);
        }
        previousKey = key;
        // ──────────────────────────────────────────────────────────────────

        if (anyManual) {
            if (!previousManual) {
                stopLedTaskAnimations(false);
                clearPendingBackendRequest();
                Serial.println("[BTN] Manual LED control active");
                requestDisplayMessage("Manual control");
            }

            manualControlActive = true;
            previousManual = true;

            int ledIndex = key - 'A';
            turnAllLedsOff();
            digitalWrite(ledPins[ledIndex], HIGH);
        } else {
            if (previousManual) {
                turnAllLedsOff();
                Serial.println("[BTN] Manual LED control released");
            }

            manualControlActive = false;
            previousManual = false;
        }

        // Random button
        bool newRandomState = false;
        if (updateDebouncedInput(RANDOM_BUTTON_PIN, randomButtonDebounce, newRandomState)) {
            if (newRandomState) {
                markControllerActivity();
                if (!manualControlActive && !randomSequenceActive) {
                    startRandomSequenceFromLedTask();
                } else {
                    Serial.println("[BTN] Random ignored");
                }
            }
        }

        // Socket button
        bool newSocketState = false;
        if (updateDebouncedInput(SOCKET_BUTTON_PIN, socketButtonDebounce, newSocketState)) {
            markControllerActivity();
            requestSocketButtonChange(newSocketState);
        }

        if (manualControlActive) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_TASK_PERIOD_MS));
            continue;
        }

        // Backend sequence request
        int requestedSequence[MAX_BACKEND_SEQUENCE_STEPS];
        int requestedLength = 0;
        int requestedDelay = blinkDelay;

        if (takeBackendSequenceRequest(requestedSequence, requestedLength, requestedDelay) && requestedLength > 0) {
            startBackendSequenceFromLedTask(requestedSequence, requestedLength, requestedDelay);
        }

        // Animate
        if (randomSequenceActive) {
            updateRandomSequenceFromLedTask();
        } else if (backendSequenceActive) {
            updateBackendSequenceFromLedTask();
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_TASK_PERIOD_MS));
    }
}

void scheduleBackendSequence(JsonArray sequence, int delayMs, long sequenceId) {
    if (manualControlActive) {
        Serial.println("[IOc] Ignored — manual control active");
        return;
    }

    if (delayMs < 50) delayMs = blinkDelay;
    currentSequenceId = sequenceId;

    int localSequence[MAX_BACKEND_SEQUENCE_STEPS];
    int localLength = 0;

    for (JsonVariant step : sequence) {
        if (localLength >= MAX_BACKEND_SEQUENCE_STEPS) break;

        int ledNumber = step.as<int>();
        int ledIndex = ledNumber - 1;

        if (ledIndex < 0 || ledIndex >= numberOfLeds) {
            Serial.printf("[IOc] invalid LED number: %d\n", ledNumber);
            continue;
        }

        localSequence[localLength++] = ledIndex;
    }

    if (localLength == 0) {
        Serial.println("[IOc] empty sequence");
        requestDisplayMessage("Empty sequence");
        return;
    }

    markControllerActivity();

    portENTER_CRITICAL(&sharedMux);
    pendingBackendSequenceLength = localLength;
    pendingBackendSequenceDelayMs = delayMs;
    for (int i = 0; i < localLength; i++) pendingBackendSequence[i] = localSequence[i];
    backendLoadRequest = true;
    portEXIT_CRITICAL(&sharedMux);

    Serial.printf("[IOc] scheduled %d steps @ %d ms (seqId=%ld)\n", localLength, delayMs, sequenceId);
    requestDisplayMessage("Watch LEDs!");
}
