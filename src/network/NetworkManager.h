#pragma once

#include <Arduino.h>
#include <WebSocketsClient.h>
void startWiFiAttempt();
void startWebSocketConnection();     // ← was startSocketConnection()
void updateWiFiAndSocket();

void onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length);  // ← was socketIOEvent(...)
void sendClientHello();
void sendEspDone();
void sendKeypressEvent(char key);   // ← NEW: called by LedController on each keypress
void makeHttpRequest();
void sendGyroData(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  bool tiltForward, bool tiltBackward,
                  bool tiltLeft,    bool tiltRight);