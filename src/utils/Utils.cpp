#include "utils/Utils.h"

void printOutput(const String& output) {
    Serial.printf("[IOc] send event: %s\n", output.c_str());
}

String payloadToString(uint8_t* payload, size_t length) {
    if (!payload || length == 0) return "";

    String value;
    value.reserve(length);
    for (size_t i = 0; i < length; i++) value += (char)payload[i];
    return value;
}

bool timeReached(unsigned long targetTime) {
    return (long)(millis() - targetTime) >= 0;
}
