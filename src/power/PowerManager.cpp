#include "power/PowerManager.h"

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "app/Settings.h"
#include "app/State.h"
#include "hardware/OledDisplay.h"
#include "hardware/Pins.h"
#include "input/Buttons.h"
#include "led/LedController.h"
#include "network/NetworkManager.h"
#include "hardware/Gyro.h"
#include "client-rx-esp32/RfReceiver.h"

// Define this only after you have physically wired MPU-6050 INT → GPIO 27.
// While undefined, gyro-wake from sleep is disabled but everything else works.
// #define MPU_INT_PIN 27

void markControllerActivity() {
    lastControllerActivityAt = millis();
}

bool isAnyControllerButtonPressed() {
    for (int i = 0; i < numberOfWakeButtons; i++) {
        if (digitalRead(wakeButtonPins[i]) == LOW) return true;
    }
    return false;
}

bool isControllerBusy() {
    return manualControlActive ||
           randomSequenceActive ||
           backendSequenceActive ||
           backendLoadRequest ||
           socketButtonStateDirty;
}

void enterLightSleepUntilButtonPress() {
    Serial.println("[Power] Entering light sleep (3 min inactive)");
    displayMessage("Sleeping\nPress button");
    delay(150);

    if (ledButtonTaskHandle) vTaskSuspend(ledButtonTaskHandle);
    stopLedTaskAnimations(true);

    socketStarted = false;
    socketConnected = false;
    socketReadyForEvents = false;
    wifiWasConnected = false;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);

    clearDisplay(true);

    // Drive all row pins LOW so any keypress pulls a column LOW during sleep
    for (int r = 0; r < 4; r++) {
        pinMode(keypadRowPins[r], OUTPUT);
        digitalWrite(keypadRowPins[r], LOW);
    }

    for (int i = 0; i < numberOfWakeButtons; i++) {
        Serial.printf("[Power] Wake pin GPIO%d = %d\n", wakeButtonPins[i], digitalRead(wakeButtonPins[i]));
    }
    Serial.flush();

    // Register keypad column pins as GPIO wake sources (LOW = key pressed)
    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_enable((gpio_num_t)wakeButtonPins[i], GPIO_INTR_LOW_LEVEL);
    }

#ifdef MPU_INT_PIN
    // Only register gyro wake source when the wire is physically in place.
    // Without it GPIO 27 floats HIGH and immediately prevents sleep entry.
    clearGyroMotionInterrupt();
    gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_HIGH_LEVEL);
    Serial.println("[Power] Gyro wake source armed on GPIO 27");
#endif

    // Register wakeup sources once, outside the loop.
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(500000ULL); // 500 ms

    while (true) {
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        if (cause == ESP_SLEEP_WAKEUP_TIMER) {
            Serial.println("[Power] Timer wake");
            Serial.flush();
            updateRfReceiver();
            continue;
        }

        if (cause == ESP_SLEEP_WAKEUP_GPIO) {
#ifdef MPU_INT_PIN
            // Check if any keypad column is LOW; if not, it was the gyro
            bool keypadPressed = false;
            for (int i = 0; i < numberOfWakeButtons; i++) {
                if (digitalRead(wakeButtonPins[i]) == LOW) {
                    keypadPressed = true;
                    break;
                }
            }
            if (!keypadPressed) {
                Serial.println("[Power] Gyro motion wake — continuing sleep");
                Serial.flush();
                clearGyroMotionInterrupt();
                continue;
            }
#endif
            Serial.println("[Power] GPIO wake — keypad pressed");
            Serial.flush();
            break;
        }

        Serial.printf("[Power] Unexpected wake cause: %d\n", cause);
        Serial.flush();
        break;
    }

    // Disable all GPIO wake sources
    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_disable((gpio_num_t)wakeButtonPins[i]);
    }
#ifdef MPU_INT_PIN
    gpio_wakeup_disable((gpio_num_t)MPU_INT_PIN);
#endif

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Restore row pins to HIGH for normal keypad scanning
    for (int r = 0; r < 4; r++) {
        digitalWrite(keypadRowPins[r], HIGH);
    }

    lastControllerActivityAt = millis();

    // Re-initialize I²C after light sleep
    Wire.end();
    delay(10);
    Wire.begin(OLED_SDA, OLED_SCL);

#ifdef MPU_INT_PIN
    clearGyroMotionInterrupt();
#endif

    if (ledButtonTaskHandle) vTaskResume(ledButtonTaskHandle);

    displayMessage("Woke up!\nReconnecting...");
    markControllerActivity();
    startWiFiAttempt();
}

void updateLowPowerManager() {
#if ENABLE_LOW_POWER_MODE
    if (!lastControllerActivityAt) {
        markControllerActivity();
        return;
    }

    // Print sleep-block reason every 5 seconds
    static unsigned long lastDebugPrint = 0;
    unsigned long now = millis();
    if (now - lastDebugPrint >= 5000) {
        lastDebugPrint = now;
        unsigned long idleMs = now - lastControllerActivityAt;
        Serial.printf("[Power] Idle: %lums / %lums  buttons:%d  busy:%d"
                      " (manual:%d randSeq:%d backSeq:%d backLoad:%d sockDirty:%d)"
                      "  gyro:%d  gpio27:%d\n",
                      idleMs, (unsigned long)INACTIVITY_SLEEP_MS,
                      (int)isAnyControllerButtonPressed(),
                      (int)isControllerBusy(),
                      (int)manualControlActive,
                      (int)randomSequenceActive,
                      (int)backendSequenceActive,
                      (int)backendLoadRequest,
                      (int)socketButtonStateDirty,
                      (int)isGyroActive(),
                      digitalRead(27));
        Serial.flush();
    }

    if (isAnyControllerButtonPressed() || isControllerBusy()) {
        markControllerActivity();
        return;
    }

    if (isGyroActive()) {
        markControllerActivity();
        return;
    }

    if (millis() - lastControllerActivityAt >= INACTIVITY_SLEEP_MS) {
        enterLightSleepUntilButtonPress();
    }
#endif
}