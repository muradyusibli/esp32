#include "app/State.h"
#include "hardware/Pins.h"

WebSocketsClient webSocket;
HTTPClient http;

bool socketStarted        = false;
bool socketConnected      = false;
bool socketReadyForEvents = false;
bool wifiWasConnected     = false;

bool socketButtonState      = false;
bool socketButtonStateDirty = false;

unsigned long lastWifiRetryAt               = 0;
unsigned long lastWifiStatusPrintAt         = 0;
unsigned long lastSocketDisconnectMessageAt = 0;
volatile unsigned long lastControllerActivityAt = 0;

long currentSequenceId = -1;

TaskHandle_t ledButtonTaskHandle = nullptr;
portMUX_TYPE sharedMux           = portMUX_INITIALIZER_UNLOCKED;

volatile bool manualControlActive = false;

volatile bool socketButtonChangeRequest = false;
volatile bool pendingSocketButtonState  = false;
volatile bool sleepButtonRequest = false;
volatile bool backendLoadRequest = false;
int pendingBackendSequence[MAX_BACKEND_SEQUENCE_STEPS];
int pendingBackendSequenceLength  = 0;
int pendingBackendSequenceDelayMs = blinkDelay;

volatile bool espDonePending = false;

volatile bool randomSequenceActive  = false;
volatile bool backendSequenceActive = false;
