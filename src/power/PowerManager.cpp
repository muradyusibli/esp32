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

// ─────────────────────────────────────────────────────────────────────────────
// Optional: MPU-6050 hardware INT wire
//
// Uncomment ONLY after physically wiring MPU-6050 INT pin → GPIO 27.
// Without the wire the pin floats HIGH, registers as a permanent wake source,
// and prevents sleep entry entirely.
//
// When commented out, gyro motion is detected via I²C polling instead
// (readGyroIntStatus() every 500 ms timer wake) — no extra wire needed.
// ─────────────────────────────────────────────────────────────────────────────
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
    return manualControlActive    ||
           randomSequenceActive   ||
           backendSequenceActive  ||
           backendLoadRequest     ||
           socketButtonStateDirty;
}

// ─────────────────────────────────────────────────────────────────────────────
// Sleep entry
// ─────────────────────────────────────────────────────────────────────────────

void enterLightSleepUntilButtonPress() {
    Serial.println("[Power] Entering light sleep (3 min inactive)");
    displayMessage("Sleeping\nPress button");
    delay(150);

    // Suspend the LED task — it holds mutexes we don't want locked during sleep.
    if (ledButtonTaskHandle) vTaskSuspend(ledButtonTaskHandle);
    stopLedTaskAnimations(true);

    // Reset network state — we reconnect fresh on wake.
    socketStarted        = false;
    socketConnected      = false;
    socketReadyForEvents = false;
    wifiWasConnected     = false;

    // Clear the dirty flag so it doesn't permanently block sleep on reconnect.
    // (flushSocketButtonState() only clears it when the socket is connected;
    //  if the button was pressed offline the flag stays set forever.)
    socketButtonStateDirty = false;

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);

    clearDisplay(true);

    // Drive all row pins LOW so any keypress pulls a column pin LOW during sleep.
    for (int r = 0; r < 4; r++) {
        pinMode(keypadRowPins[r], OUTPUT);
        digitalWrite(keypadRowPins[r], LOW);
    }

    // Log pin states for debugging before we go silent.
    for (int i = 0; i < numberOfWakeButtons; i++) {
        Serial.printf("[Power] Wake pin GPIO%d = %d\n",
                      wakeButtonPins[i], digitalRead(wakeButtonPins[i]));
    }
    Serial.flush();

    // ── Register wake sources ──────────────────────────────────────────────
    // Keypad column pins: wake when any column is pulled LOW by a keypress.
    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_enable((gpio_num_t)wakeButtonPins[i], GPIO_INTR_LOW_LEVEL);
    }

#ifdef MPU_INT_PIN
    // Hardware INT wire path: arm the GPIO.
    // Only reachable when the wire is physically present.
    clearGyroMotionInterrupt();
    gpio_wakeup_enable((gpio_num_t)MPU_INT_PIN, GPIO_INTR_HIGH_LEVEL);
    Serial.println("[Power] Gyro wake source armed on GPIO " TOSTRING(MPU_INT_PIN));
#endif

    // RF receiver: wake immediately when a signal pulse arrives so RCSwitch's ISR
    // can capture the transitions.  GPIO 35 idles LOW; a received signal goes HIGH.
    gpio_wakeup_enable((gpio_num_t)RF_RECEIVER_PIN, GPIO_INTR_HIGH_LEVEL);

    // Both sources registered once, outside the loop.
    esp_sleep_enable_gpio_wakeup();
    esp_sleep_enable_timer_wakeup(500000ULL);  // 500 ms — RF + gyro poll interval

    // ── Sleep loop ─────────────────────────────────────────────────────────
    bool wakeConfirmed = false;

    while (!wakeConfirmed) {
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

        // ── Timer wake: poll RF and gyro, then go back to sleep ────────────
        if (cause == ESP_SLEEP_WAKEUP_TIMER) {

            // Wire is not automatically restored after light sleep; re-init it
            // here so the I²C calls below (gyro, etc.) actually work.
            Wire.begin(OLED_SDA, OLED_SCL);

#if RF_WAKES_FROM_SLEEP
            if (updateRfReceiver()) {
                Serial.println("[Power] RF signal received — waking");
                Serial.flush();
                wakeConfirmed = true;
                break;
            }
#else
            updateRfReceiver();  // still fires motor/LED, just doesn't wake
#endif

#if GYRO_WAKES_FROM_SLEEP && !defined(MPU_INT_PIN)
            // I²C polling path — no hardware INT wire required.
            // Bit 6 of INT_STATUS (0x3A) = motion interrupt fired.
            uint8_t intStatus = readGyroIntStatus();
            if (intStatus & 0x40) {
                Serial.println("[Power] Gyro motion detected (I2C poll) — waking");
                Serial.flush();
                wakeConfirmed = true;
                break;
            }
#endif

            continue;  // no wake condition — go back to sleep
        }

        // ── GPIO wake: keypad, RF signal, or (optionally) hardware gyro INT ──
        if (cause == ESP_SLEEP_WAKEUP_GPIO) {

            // Check keypad first — any column pulled LOW means a key is pressed.
            bool keypadPressed = false;
            for (int i = 0; i < numberOfWakeButtons; i++) {
                if (digitalRead(wakeButtonPins[i]) == LOW) {
                    keypadPressed = true;
                    break;
                }
            }

            if (keypadPressed) {
                Serial.println("[Power] GPIO wake — keypad pressed");
                Serial.flush();
                wakeConfirmed = true;
                break;
            }

#ifdef MPU_INT_PIN
            // Hardware gyro INT wire path.
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
                continue;
#endif
#endif  // MPU_INT_PIN

            // Not keypad (and no INT wire) — likely the RF pin went HIGH.
            // RCSwitch's ISR ran while the CPU was awake during the signal;
            // check whether a valid code was decoded.
#if RF_WAKES_FROM_SLEEP
            if (updateRfReceiver()) {
                Serial.println("[Power] RF signal via GPIO wake — waking");
                Serial.flush();
                wakeConfirmed = true;
                break;
            }
#else
            updateRfReceiver();
#endif

            continue;  // spurious / noise wake — go back to sleep
        }

        // ── Unexpected cause: bail out rather than hang ────────────────────
        Serial.printf("[Power] Unexpected wake cause: %d — waking\n", cause);
        Serial.flush();
        wakeConfirmed = true;
        break;
    }

    // ── Teardown wake sources ──────────────────────────────────────────────
    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_disable((gpio_num_t)wakeButtonPins[i]);
    }
    gpio_wakeup_disable((gpio_num_t)RF_RECEIVER_PIN);
#ifdef MPU_INT_PIN
    gpio_wakeup_disable((gpio_num_t)MPU_INT_PIN);
#endif
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    // Restore keypad row pins to their normal driven-HIGH idle state.
    for (int r = 0; r < 4; r++) {
        digitalWrite(keypadRowPins[r], HIGH);
    }

    lastControllerActivityAt = millis();

    // ── Re-initialise I²C ─────────────────────────────────────────────────
    // The Wire peripheral is not automatically restored after light sleep.
    Wire.end();
    delay(10);
    Wire.begin(OLED_SDA, OLED_SCL);

    // Re-arm the MPU-6050 motion interrupt registers.
    // They may have been cleared or lost during the Wire re-init sequence.
    initGyroMotionWake();

    if (ledButtonTaskHandle) vTaskResume(ledButtonTaskHandle);

    displayMessage("Woke up!\nReconnecting...");
    markControllerActivity();
    startWiFiAttempt();
}

// ─────────────────────────────────────────────────────────────────────────────
// Called every loop() iteration
// ─────────────────────────────────────────────────────────────────────────────

void updateLowPowerManager() {
#if ENABLE_LOW_POWER_MODE
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