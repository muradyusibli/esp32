#pragma once

void markControllerActivity();
bool isAnyControllerButtonPressed();
bool isControllerBusy();
void requestImmediateSleep();
void updateLowPowerManager();
void enterLightSleepUntilButtonPress();
