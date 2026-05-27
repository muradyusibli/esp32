#include "hardware/OledDisplay.h"

#include <Adafruit_SH1106.h>
#include <Arduino.h>
#include <Wire.h>
#include <cstring>

#include "app/Settings.h"
#include "app/State.h"

static Adafruit_SH1106 display;
static String lastDisplayText = "";

static volatile bool displayRequestPending = false;
static char pendingDisplayMessage[128];

void initDisplay() {
    Wire.begin(OLED_SDA, OLED_SCL);
    display.begin(SH1106_SWITCHCAPVCC, I2C_ADDRESS);
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    displayMessage("Starting...");
}

void displayMessage(const char* message) {
    String text(message);
    if (text == lastDisplayText) return;

    lastDisplayText = text;
    display.clearDisplay();
    display.setCursor(0, 10);
    display.println(message);
    display.display();
}

void requestDisplayMessage(const char* message) {
    portENTER_CRITICAL(&sharedMux);
    strncpy(pendingDisplayMessage, message, sizeof(pendingDisplayMessage) - 1);
    pendingDisplayMessage[sizeof(pendingDisplayMessage) - 1] = '\0';
    displayRequestPending = true;
    portEXIT_CRITICAL(&sharedMux);
}

void processDisplayRequest() {
    bool hasMessage = false;
    char local[128];

    portENTER_CRITICAL(&sharedMux);
    if (displayRequestPending) {
        strncpy(local, pendingDisplayMessage, sizeof(local) - 1);
        local[sizeof(local) - 1] = '\0';
        displayRequestPending = false;
        hasMessage = true;
    }
    portEXIT_CRITICAL(&sharedMux);

    if (hasMessage) displayMessage(local);
}

void clearDisplay(bool resetCachedText) {
    display.clearDisplay();
    display.display();
    if (resetCachedText) lastDisplayText = "";
}
