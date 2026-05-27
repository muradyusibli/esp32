#pragma once

#include <ArduinoJson.h>

void initControllerPins();
void turnAllLedsOff();

void clearPendingBackendRequest();
void stopLedTaskAnimations(bool turnOffLeds = true);
void ledButtonTask(void* parameter);

void scheduleBackendSequence(JsonArray sequence, int delayMs, long sequenceId);
