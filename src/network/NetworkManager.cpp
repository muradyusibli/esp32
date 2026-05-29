#include "network/NetworkManager.h"

#include <ArduinoJson.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <WiFi.h>

#include "app/Settings.h"
#include "app/State.h"
#include "config.h"
#include "hardware/OledDisplay.h"
#include "hardware/Pins.h"
#include "led/LedController.h"
#include "utils/Utils.h"


static void wsSend(const String& json) {
    printOutput(json);
    webSocket.sendTXT(json.c_str());
}

static void sendEvent(const char* eventName, JsonObject data) {
    JsonDocument wrapper;
    wrapper["event"] = eventName;
    wrapper["data"]  = data;
    String out;
    serializeJson(wrapper, out);
    wsSend(out);
}


static void flushSocketButtonState() {
    if (!socketButtonStateDirty) return;
    if (!(socketStarted && socketConnected && socketReadyForEvents &&
          WiFi.status() == WL_CONNECTED)) return;

    JsonDocument doc;
    JsonObject data = doc.to<JsonObject>();
    data["protocolVersion"] = 1;
    data["clientRole"]      = "ESP32";
    data["targetRole"]      = "BACKEND";
    data["gameType"]        = "SEQUENCE";
    data["action"]          = "DEVICE_BUTTON_STATE";
    data["roomId"]          = roomId;
    data.createNestedObject("payload")["buttonState"] = socketButtonState;

    sendEvent("game:message", data);
    socketButtonStateDirty = false;
}

//  Public send

void sendClientHello() {
    JsonDocument doc;
    JsonObject data = doc.to<JsonObject>();
    data["protocolVersion"] = 1;
    data["clientRole"]      = "ESP32";
    data["roomId"]          = roomId;
    data["gameType"]        = "SEQUENCE";
    data["deviceType"]      = "ESP32_LED_CONTROLLER";
    sendEvent("client:hello", data);
}

void sendEspDone() {
    if (!(socketStarted && socketConnected && socketReadyForEvents)) return;

    JsonDocument doc;
    JsonObject data = doc.to<JsonObject>();
    data["roomId"]     = roomId;
    data["sequenceId"] = currentSequenceId;
    sendEvent("esp:done", data);

    Serial.println("[WS] esp:done sent → backend will emit game:inputReady to frontend");
    requestDisplayMessage("Your turn!");
}

//  Keypress event


void sendKeypressEvent(char key, const char* type) {
    if (!(socketStarted && socketConnected && socketReadyForEvents &&
          WiFi.status() == WL_CONNECTED)) return;

    char keyStr[2] = { key, '\0' };

    JsonDocument doc;
    JsonObject data = doc.to<JsonObject>();
    data["protocolVersion"] = 1;
    data["clientRole"]      = "ESP32";
    data["targetRole"]      = "BACKEND";
    data["gameType"]        = "SEQUENCE";
    data["action"]          = "KEYPRESS";
    data["roomId"]          = roomId;

    JsonObject payload = data.createNestedObject("payload");
    payload["key"]  = keyStr;
    payload["type"] = type;


    if (key >= '0' && key <= '9') {
        payload["numpadKey"] = (int)(key - '0');
    } else {
        payload["numpadKey"] = -1;
    }

    sendEvent("game:keypress", data);
    Serial.printf("[WS] game:keypress → key='%c' type=%s\n", key, type);
}

void sendGyroData(float ax, float ay, float az,
                  float gx, float gy, float gz,
                  bool tiltForward, bool tiltBackward,
                  bool tiltLeft,    bool tiltRight)
{
    if (!(socketStarted && socketConnected && socketReadyForEvents
          && WiFi.status() == WL_CONNECTED)) return;

    JsonDocument doc;
    JsonObject data = doc.to<JsonObject>();
    data["protocolVersion"] = 1;
    data["clientRole"]      = "ESP32";
    data["targetRole"]      = "BACKEND";
    data["gameType"]        = "SEQUENCE";
    data["action"]          = "GYRO_DATA";
    data["roomId"]          = roomId;

    JsonObject payload = data.createNestedObject("payload");
    payload["acc_x"]  = serialized(String(ax, 4));
    payload["acc_y"]  = serialized(String(ay, 4));
    payload["acc_z"]  = serialized(String(az, 4));
    payload["gyro_x"] = serialized(String(gx, 2));
    payload["gyro_y"] = serialized(String(gy, 2));
    payload["gyro_z"] = serialized(String(gz, 2));

    payload["tilt_forward"]  = tiltForward;
    payload["tilt_backward"] = tiltBackward;
    payload["tilt_left"]     = tiltLeft;
    payload["tilt_right"]    = tiltRight;

    sendEvent("game:message", data);
}

//  Incoming message handler
static void handleGameMessage(JsonObject msg) {
    if (msg.isNull()) return;

    String msgRoom = msg["roomId"] | "";
    if (msgRoom.length() > 0 && msgRoom != roomId) {
        Serial.printf("[WS] ignored message for room: %s\n", msgRoom.c_str());
        return;
    }

    String gameType = msg["gameType"] | "";
    String action   = msg["action"]   | "";
    if (gameType != "SEQUENCE") return;

    JsonObject payload = msg["payload"].as<JsonObject>();
    if (payload.isNull()) return;

    if (payload.containsKey("message")) {
        String message = payload["message"].as<String>();
        if (message.length() > 0 && message != "null")
            displayMessage(message.c_str());
    }

    if (action == "PLAY_SEQUENCE") {
        if (!payload.containsKey("sequence")) return;
        JsonArray sequence  = payload["sequence"].as<JsonArray>();
        int      delayMs    = payload["delayMs"]    | blinkDelay;
        long     sequenceId = payload["sequenceId"] | (long)-1;
        scheduleBackendSequence(sequence, delayMs, sequenceId);
    }
}

//  WebSocket event callback

void onWebSocketEvent(WStype_t type, uint8_t* payload, size_t length) {
    String text = payloadToString(payload, length);

    switch (type) {

    case WStype_DISCONNECTED:
        socketConnected      = false;
        socketReadyForEvents = false;
        Serial.printf("[WS] Disconnected. payload='%s' length=%u\n",
                      text.c_str(), (unsigned)length);
        if (millis() - lastSocketDisconnectMessageAt >=
            SOCKET_DISCONNECT_MESSAGE_INTERVAL_MS) {
            lastSocketDisconnectMessageAt = millis();
#if SHOW_SOCKET_OFFLINE_ON_OLED
            displayMessage("WS offline\nLEDs still work");
#endif
        }
        break;

    case WStype_CONNECTED:
        socketConnected      = true;
        socketReadyForEvents = false;
        Serial.printf("[WS] Connected: %s\n", text.c_str());

        delay(100);

        {
        JsonDocument joinDoc;
        JsonObject joinData = joinDoc.to<JsonObject>();
        joinData["roomId"] = roomId;
        sendEvent("joinRoom", joinData);
        delay(50);
        }

        sendClientHello();

        displayMessage("WS OK\nRegistering...");
        break;

        case WStype_TEXT: {
            Serial.printf("[WS] text: %s\n", text.c_str());

            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, payload, length);
            if (err) {
                Serial.printf("[WS] JSON err: %s\n", err.c_str());
                return;
            }

            if (!doc.is<JsonObject>()) return;

            String eventName = doc["event"] | "";
            if (eventName.isEmpty()) return;

            Serial.printf("[WS] event: %s\n", eventName.c_str());

            if (eventName == "game:message") {
                handleGameMessage(doc["data"].as<JsonObject>());

            } else if (eventName == "update") {
                // Legacy backward-compat path
                JsonObject attrs = doc["data"].as<JsonObject>();
                if (attrs.isNull()) return;
                if (attrs.containsKey("message")) {
                    String message = attrs["message"].as<String>();
                    if (message.length() > 0 && message != "null")
                        displayMessage(message.c_str());
                }
                if (attrs.containsKey("sequence")) {
                    JsonArray sequence  = attrs["sequence"].as<JsonArray>();
                    int      delayMs    = attrs["delayMs"]    | blinkDelay;
                    long     sequenceId = attrs["sequenceId"] | (long)-1;
                    scheduleBackendSequence(sequence, delayMs, sequenceId);
                }

            } else if (eventName == "client:helloAck") {
                socketReadyForEvents = true;
                Serial.println("[WS] client:helloAck received — ready to send events");

            } else if (eventName == "game:inputReady") {
                Serial.println("[WS] game:inputReady received");

            } else {
                Serial.printf("[WS] ignored event: %s\n", eventName.c_str());
            }
            break;
        }

        case WStype_ERROR:
            socketConnected      = false;
            socketReadyForEvents = false;
            Serial.printf("[WS] Error: %s\n", text.c_str());
            displayMessage("WS error");
            break;

        case WStype_PING:
            Serial.println("[WS] PING");
            break;

        case WStype_PONG:
            Serial.println("[WS] PONG");
            break;

        default:
            break;
    }
}

//WiFi + connection lifecycle
void startWiFiAttempt() {
    Serial.printf("[WiFi] Connecting to: %s\n", ssid);
    displayMessage("Connecting WiFi");

    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(ssid, password);

    lastWifiRetryAt      = millis();
    lastWifiStatusPrintAt = 0;
}

void startWebSocketConnection() {
    Serial.printf("[CFG] host=%s port=%d path=%s ssl=%d demoHttp=%d\n",
                  host, port, WS_PATH, USE_SOCKET_SSL, RUN_DEMO_HTTP_REQUEST);

    webSocket.onEvent(onWebSocketEvent);
    webSocket.setReconnectInterval(SOCKET_RECONNECT_INTERVAL_MS);

    webSocket.enableHeartbeat(100000, 10000, 3);  // ping every 100s

#if USE_SOCKET_SSL
    Serial.printf("[WS] Connecting to wss://%s:%d%s\n", host, port, WS_PATH);
    webSocket.beginSSL(host, port, WS_PATH, "");
#else
    Serial.printf("[WS] Connecting to ws://%s:%d%s\n", host, port, WS_PATH);
    webSocket.begin(host, port, WS_PATH, "");
#endif

    socketStarted        = true;
    socketConnected      = false;
    socketReadyForEvents = false;
}
void updateWiFiAndSocket() {
    unsigned long now        = millis();
    wl_status_t   wifiStatus = WiFi.status();

    if (wifiStatus == WL_CONNECTED) {
        if (!wifiWasConnected) {
            wifiWasConnected = true;

            String ip = WiFi.localIP().toString();
            Serial.printf("[WiFi] Connected! IP: %s\n", ip.c_str());

            char buf[128];
            snprintf(buf, sizeof(buf), "WiFi OK\n%s", ip.c_str());
            displayMessage(buf);

            startWebSocketConnection();

            if (RUN_DEMO_HTTP_REQUEST) makeHttpRequest();
        }

        if (socketStarted) {
            webSocket.loop();
            flushSocketButtonState();

            bool shouldSendDone = false;
            portENTER_CRITICAL(&sharedMux);
            if (espDonePending) {
                espDonePending  = false;
                shouldSendDone = true;
            }
            portEXIT_CRITICAL(&sharedMux);

            if (shouldSendDone) sendEspDone();
        }
        return;
    }

    if (wifiWasConnected) {
        wifiWasConnected     = false;
        socketStarted        = false;
        socketConnected      = false;
        socketReadyForEvents = false;
        Serial.println("[WiFi] Lost connection.");
        displayMessage("WiFi lost\nLEDs still work");
    }

    if (now - lastWifiStatusPrintAt >= WIFI_STATUS_PRINT_INTERVAL_MS) {
        lastWifiStatusPrintAt = now;
        Serial.printf("[WiFi] Not connected. Status=%d\n", wifiStatus);
    }

    if (now - lastWifiRetryAt >= WIFI_RETRY_INTERVAL_MS) {
        Serial.println("[WiFi] Retrying...");
        displayMessage("WiFi retrying...");
        WiFi.disconnect(false);
        WiFi.begin(ssid, password);
        lastWifiRetryAt = now;
    }
}

void makeHttpRequest() {
    http.begin(request_url);
    http.setTimeout(3000);
    int code = http.GET();
    if (code == HTTP_CODE_OK) Serial.println(http.getString());
    else Serial.printf("[HTTP] code: %d\n", code);
    http.end();
}
