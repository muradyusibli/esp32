#include "input/Keypad.h"
#include "hardware/Pins.h"
#include <Arduino.h>

void initKeypad() {
    for (int r = 0; r < 4; r++) {
        pinMode(keypadRowPins[r], OUTPUT);
        digitalWrite(keypadRowPins[r], HIGH);
    }
    for (int c = 0; c < 4; c++) {
        pinMode(keypadColPins[c], INPUT_PULLUP);
    }
}

char keypadScan() {
    const char keys[4][4] = {
        {'1','2','3','A'},
        {'4','5','6','B'},
        {'7','8','9','C'},
        {'*','0','#','D'}
    };
    for (int r = 0; r < 4; r++) {
        digitalWrite(keypadRowPins[r], LOW);
        vTaskDelay(pdMS_TO_TICKS(1));
        for (int c = 0; c < 4; c++) {
            if (digitalRead(keypadColPins[c]) == LOW) {
                digitalWrite(keypadRowPins[r], HIGH);
                Serial.printf("[KEYPAD] row=%d (GPIO%d) col=%d (GPIO%d) key=%c\n",
                    r, keypadRowPins[r], c, keypadColPins[c], keys[r][c]);
                return keys[r][c];
            }
        }
        digitalWrite(keypadRowPins[r], HIGH);
    }
    return 0;
}