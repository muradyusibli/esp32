#include "power/PowerManager.h"

#include <Arduino.h>
#include <WiFi.h>
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "app/Settings.h"
#include "app/State.h"
#include "hardware/OledDisplay.h"
#include "hardware/Pins.h"
#include "input/Buttons.h"
#include "led/LedController.h"
#include "network/NetworkManager.h"

void markControllerActivity() {
    lastControllerActivityAt = millis();
}

bool isAnyControllerButtonPressed() {
    for (int i = 0; i < numberOfWakeButtons; i++) {
        if (isButtonPressed(wakeButtonPins[i])) return true;
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

    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_enable((gpio_num_t)wakeButtonPins[i], GPIO_INTR_LOW_LEVEL);
    }

    // RF receiver (GPIO 13) idles LOW and pulses HIGH when a signal arrives.
    // Register it as a HIGH-level wake source so any incoming RF packet wakes
    // the device, exactly like a keypad button press does.
    gpio_wakeup_enable((gpio_num_t)RF_RECEIVER_PIN, GPIO_INTR_HIGH_LEVEL);

    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();

    for (int i = 0; i < numberOfWakeButtons; i++) {
        gpio_wakeup_disable((gpio_num_t)wakeButtonPins[i]);
    }
    gpio_wakeup_disable((gpio_num_t)RF_RECEIVER_PIN);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

    lastControllerActivityAt = millis();

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

    if (isAnyControllerButtonPressed() || isControllerBusy()) {
        markControllerActivity();
        return;
    }

    if (millis() - lastControllerActivityAt >= INACTIVITY_SLEEP_MS) {
        enterLightSleepUntilButtonPress();
    }
#endif
}