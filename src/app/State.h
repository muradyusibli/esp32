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

// sequenceId received from the backend — sent back in esp:done
extern long currentSequenceId;

// FreeRTOS shared state accessed from both cores / tasks
extern TaskHandle_t ledButtonTaskHandle;
extern portMUX_TYPE sharedMux;

extern volatile bool manualControlActive;

extern volatile bool socketButtonChangeRequest;
extern volatile bool pendingSocketButtonState;

extern volatile bool backendLoadRequest;
extern int pendingBackendSequence[MAX_BACKEND_SEQUENCE_STEPS];
extern int pendingBackendSequenceLength;
extern int pendingBackendSequenceDelayMs;

// Set by the LED task when sequence finishes; consumed on the main loop.
// WebSocket calls must happen on the main loop / core 0.
extern volatile bool espDonePending;

// Animation activity flags used by the low-power manager.
extern volatile bool randomSequenceActive;
extern volatile bool backendSequenceActive;
