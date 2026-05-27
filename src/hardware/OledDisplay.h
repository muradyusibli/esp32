#pragma once

void initDisplay();
void displayMessage(const char* message);
void requestDisplayMessage(const char* message);
void processDisplayRequest();
void clearDisplay(bool resetCachedText = true);
