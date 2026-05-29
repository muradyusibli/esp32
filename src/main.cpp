#include <Arduino.h>

#include "app/State.h"
#include "hardware/OledDisplay.h"
#include "hardware/Gyro.h"
#include "input/Buttons.h"
#include "led/LedController.h"
#include "network/NetworkManager.h"
#include "power/PowerManager.h"
#include "client-rx-esp32/RfReceiver.h"
#include "input/Keypad.h"

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== ESP32 Sequence Game ===");

    initControllerPins();
    initDisplay();
    initGyroMotionWake();
    initRfReceiver();
    initKeypad();

    xTaskCreatePinnedToCore(
        ledButtonTask,
        "ledButtonTask",
        4096,
        nullptr,
        3,
        &ledButtonTaskHandle,
        1
    );

    markControllerActivity();
    startWiFiAttempt();
}

void loop() {
    printButtonDebug();
    processSocketButtonRequest();
    processDisplayRequest();
    updateWiFiAndSocket();
    updateLowPowerManager();
    updateRfReceiver();
    updateGyro();
    yield();
}