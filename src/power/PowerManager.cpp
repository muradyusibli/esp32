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
#include "hardware/Gyro.h"
#include "input/Buttons.h"
#include "led/LedController.h"
#include "network/NetworkManager.h"
#include "client-rx-esp32/RfReceiver.h"


#define MPU_INT_PIN 27


#define RF_LISTEN_WINDOW_MS 100

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
    return manualControlActive    ||
           randomSequenceActive   ||
           backendSequenceActive  ||
           backendLoadRequest     ||
           socketButtonStateDirty;
}

void requestImmediateSleep() {
    portENTER_CRITICAL(&sharedMux);
    sleepButtonRequest = true;
    portEXIT_CRITICAL(&sharedMux);
}

static bool takeImmediateSleepRequest() {
    bool requested = false;

    portENTER_CRITICAL(&sharedMux);
    if (sleepButtonRequest) {
        sleepButtonRequest = false;
        requested = true;
    }
    portEXIT_CRITICAL(&sharedMux);

    return requested;
}

static void clearImmediateSleepRequest() {
    portENTER_CRITICAL(&sharedMux);
    sleepButtonRequest = false;
    portEXIT_CRITICAL(&sharedMux);
}

static void waitForSleepButtonRelease(const char* reason) {
    if (digitalRead(SLEEP_BUTTON_PIN) == LOW) {
        Serial.println(reason);
        displayMessage("Release sleep\nbutton");

        while (digitalRead(SLEEP_BUTTON_PIN) == LOW) {
            delay(10);
            yield();
        }

        delay(BUTTON_DEBOUNCE_MS);
    }

    clearImmediateSleepRequest();
}



void enterLightSleepUntilButtonPress() {
    Serial.println("[Power] Entering light sleep");
    displayMessage("Sleeping\nPress button");
    delay(150);

    if (ledButtonTaskHandle) vTaskSuspend(ledButtonTaskHandle);
    stopLedTaskAnimations(true);

    socketStarted        = false;
    socketConnected      = false;
    socketReadyForEvents = false;
    wifiWasConnected     = false;

    socketButtonStateDirty = false;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);

    clearDisplay(true);

    for (int r = 0; r < 4; r++) {
        pinMode(keypadRowPins[r], OUTPUT);
        digitalWrite(keypadRowPins[r], LOW);
    }

    waitForSleepButtonRelease("[Power] Waiting for sleep button release before sleeping");


#if RF_WAKES_FROM_SLEEP
    disableRfReceive();
#endif

    for (int i = 0; i < numberOfWakeButtons; i++) {
        Serial.printf("[Power] Wake pin GPIO%d = %d\n",
                      wakeButtonPins[i], digitalRead(wakeButtonPins[i]));
    }
    Serial.flush();


    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_enable((gpio_num_t)wakeButtonPins[i], GPIO_INTR_LOW_LEVEL);
    }

    // MPU-9265 INT pin: wake when pulled HIGH by motion.
#ifdef MPU_INT_PIN
    initGyroMotionWake();
    delay(20);
    clearGyroMotionInterrupt();
    delay(5);
    gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_HIGH_LEVEL);
    Serial.printf("[Power] Gyro wake source armed on GPIO %d\n", MPU_INT_PIN);
#endif

    // RF is NOT registered as a GPIO wake source — the 433MHz module DATA

    Serial.println("[Power] RF wake via timer poll (500ms)");
    Serial.flush();

    // GPIO and timer wake sources.
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(500000ULL);  // 500 ms — RF poll interval

    //  Sleep loop
    bool wakeConfirmed = false;

    while (!wakeConfirmed) {
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        if (cause == ESP_SLEEP_WAKEUP_TIMER) {
#if RF_WAKES_FROM_SLEEP
            enableRfReceive();

            delay(RF_LISTEN_WINDOW_MS);

            bool rfGot = updateRfReceiver();

            if (rfGot) {
                Serial.println("[Power] RF signal received (timer poll) — waking");
                Serial.flush();
                wakeConfirmed = true;
                break;
            }

            disableRfReceive();
#endif
            continue;
        }

        if (cause == ESP_SLEEP_WAKEUP_GPIO) {

            bool gpioButtonPressed = false;
            for (int i = 0; i < numberOfWakeButtons; i++) {
                if (digitalRead(wakeButtonPins[i]) == LOW) {
                    gpioButtonPressed = true;
                    break;
                }
            }

            if (gpioButtonPressed) {
                if (digitalRead(SLEEP_BUTTON_PIN) == LOW) {
                    Serial.println("[Power] GPIO wake — sleep button pressed");
                } else {
                    Serial.println("[Power] GPIO wake — keypad pressed");
                }
                Serial.flush();
                wakeConfirmed = true;
                break;
            }

#ifdef MPU_INT_PIN
            if (digitalRead(MPU_INT_PIN) == HIGH) {
#if GYRO_WAKES_FROM_SLEEP
                Serial.println("[Power] Gyro motion wake (INT pin) — waking");
                Serial.flush();
                clearGyroMotionInterrupt();
                wakeConfirmed = true;
                break;
#else
                Serial.println("[Power] Gyro INT fired but GYRO_WAKES_FROM_SLEEP=0 — continuing");
                Serial.flush();
                clearGyroMotionInterrupt();
                delay(5);

                gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_HIGH_LEVEL);
                continue;
#endif
            }
#endif

            continue;
        }

        Serial.printf("[Power] Unexpected wake cause: %d — waking\n", cause);
        Serial.flush();
        wakeConfirmed = true;
        break;
    }

    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_disable((gpio_num_t)wakeButtonPins[i]);
    }

#ifdef MPU_INT_PIN
    gpio_wakeup_disable((gpio_num_t)MPU_INT_PIN);
#endif

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Restore keypad row pins to their normal driven-HIGH idle state.
    for (int r = 0; r < 4; r++) {
        digitalWrite(keypadRowPins[r], HIGH);
    }

    lastControllerActivityAt = millis();

    //  Re-initialise I²C
    // Wire is not automatically restored after light sleep.
    Wire.end();
    delay(10);
    Wire.begin(OLED_SDA, OLED_SCL);
    delay(20);  // let bus settle before talking to MPU

    // Re-arm WOM registers after Wire re-init.
    initGyroMotionWake();

    // Re-enable RF receiver for normal awake operation.
#if RF_WAKES_FROM_SLEEP
    enableRfReceive();
#endif

    waitForSleepButtonRelease("[Power] Waiting for sleep button release after wake");

    if (ledButtonTaskHandle) vTaskResume(ledButtonTaskHandle);

    displayMessage("Woke up!\nReconnecting...");
    markControllerActivity();
    startWiFiAttempt();
}


void updateLowPowerManager() {
#if ENABLE_LOW_POWER_MODE

    if (takeImmediateSleepRequest()) {
        Serial.println("[Power] Sleep button pressed — entering light sleep");
        enterLightSleepUntilButtonPress();
        return;
    }

    if (!lastControllerActivityAt) {
        markControllerActivity();
        return;
    }

    static unsigned long lastDebugPrint = 0;
    unsigned long now = millis();

    if (now - lastDebugPrint >= 5000) {
        lastDebugPrint = now;
        unsigned long idleMs = now - lastControllerActivityAt;

        Serial.printf("[Power] Idle: %lums / %lums  buttons:%d  busy:%d"
                      " (manual:%d randSeq:%d backSeq:%d backLoad:%d sockDirty:%d)"
                      "  gyro:%d\n",
                      idleMs, (unsigned long)INACTIVITY_SLEEP_MS,
                      (int)isAnyControllerButtonPressed(),
                      (int)isControllerBusy(),
                      (int)manualControlActive,
                      (int)randomSequenceActive,
                      (int)backendSequenceActive,
                      (int)backendLoadRequest,
                      (int)socketButtonStateDirty,
                      (int)isGyroActive());

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