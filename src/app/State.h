#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>        // ← was SocketIOclient.h
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "app/Settings.h"

extern WebSocketsClient webSocket;   // ← was SocketIOclient socketIO
extern HTTPClient http;

// WiFi / Socket state
extern bool socketStarted;
extern bool socketConnected;
extern bool socketReadyForEvents;
extern bool wifiWasConnected;

extern bool socketButtonState;
extern bool socketButtonStateDirty;

extern unsigned long lastWifiRetryAt;
extern unsigned long lastWifiStatusPrintAt;
extern unsigned long lastSocketDisconnectMessageAt;
extern volatile unsigned long lastControllerActivityAt;

extern long currentSequenceId;

extern TaskHandle_t ledButtonTaskHandle;
extern portMUX_TYPE sharedMux;

extern volatile bool manualControlActive;

extern volatile bool socketButtonChangeRequest;
extern volatile bool pendingSocketButtonState;
extern volatile bool sleepButtonRequest;
extern volatile bool backendLoadRequest;
extern int pendingBackendSequence[MAX_BACKEND_SEQUENCE_STEPS];
extern int pendingBackendSequenceLength;
extern int pendingBackendSequenceDelayMs;

extern volatile bool espDonePending;

extern volatile bool randomSequenceActive;
extern volatile bool backendSequenceActive;
