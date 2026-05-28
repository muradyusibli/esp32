#include "Gyro.h"
#include "Pins.h"
#include <Wire.h>
#include <Arduino.h>
#include "network/NetworkManager.h"
#include "app/Settings.h"

static Gyro gyro;

static constexpr float ACC_SENSITIVITY  = 16384.0f;
static constexpr float GYRO_SENSITIVITY =   131.0f;
static constexpr float G_TO_MS2         =     9.81f;
static constexpr int   AVG_WINDOW       =    20;

static constexpr float TILT_THRESHOLD_FB  =  0.854f;  // forward/backward (5°)
static constexpr float TILT_THRESHOLD_LR  =  1.703f;  // left/right (10°)

// ── Bias (calibration) ──────────────────────────────────────────────
static float bias_ax = 0, bias_ay = 0, bias_az = 0;
static float bias_gx = 0, bias_gy = 0, bias_gz = 0;

// ── Running-average buffers ──────────────────────────────────────────
static float buf_ax[AVG_WINDOW] = {}, buf_ay[AVG_WINDOW] = {}, buf_az[AVG_WINDOW] = {};
static float buf_gx[AVG_WINDOW] = {}, buf_gy[AVG_WINDOW] = {}, buf_gz[AVG_WINDOW] = {};
static int   avg_idx   = 0;
static int   avg_count = 0;

static void pushAvg(float ax, float ay, float az,
                    float gx, float gy, float gz) {
    buf_ax[avg_idx] = ax;  buf_ay[avg_idx] = ay;  buf_az[avg_idx] = az;
    buf_gx[avg_idx] = gx;  buf_gy[avg_idx] = gy;  buf_gz[avg_idx] = gz;
    avg_idx = (avg_idx + 1) % AVG_WINDOW;
    if (avg_count < AVG_WINDOW) avg_count++;
}

static void calcAvg(float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz) {
    float sax=0,say=0,saz=0,sgx=0,sgy=0,sgz=0;
    for (int i = 0; i < avg_count; i++) {
        sax += buf_ax[i]; say += buf_ay[i]; saz += buf_az[i];
        sgx += buf_gx[i]; sgy += buf_gy[i]; sgz += buf_gz[i];
    }
    ax = sax/avg_count; ay = say/avg_count; az = saz/avg_count;
    gx = sgx/avg_count; gy = sgy/avg_count; gz = sgz/avg_count;
}

static bool prev_tiltForward  = false;
static bool prev_tiltBackward = false;
static bool prev_tiltLeft     = false;
static bool prev_tiltRight    = false;

static volatile bool gyroActivityFlag = false;

// ── Gyro class ───────────────────────────────────────────────────────
bool Gyro::begin() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    return Wire.endTransmission(true) == 0;
}

bool Gyro::read(GyroData& out) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)1);
    if (Wire.available() < 14) return false;

    out.acc_x  = Wire.read() << 8 | Wire.read();
    out.acc_y  = Wire.read() << 8 | Wire.read();
    out.acc_z  = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();  // skip temperature
    out.gyro_x = Wire.read() << 8 | Wire.read();
    out.gyro_y = Wire.read() << 8 | Wire.read();
    out.gyro_z = Wire.read() << 8 | Wire.read();
    return true;
}

// ── Calibration ──────────────────────────────────────────────────────
static void calibrateGyro() {
    Serial.println("[Gyro] Calibrating — keep still...");

    const int SAMPLES = 200;
    long sum_ax=0, sum_ay=0, sum_az=0;
    long sum_gx=0, sum_gy=0, sum_gz=0;

    for (int i = 0; i < SAMPLES; i++) {
        GyroData raw;
        if (gyro.read(raw)) {
            sum_ax += raw.acc_x; sum_ay += raw.acc_y; sum_az += raw.acc_z;
            sum_gx += raw.gyro_x; sum_gy += raw.gyro_y; sum_gz += raw.gyro_z;
        }
        delay(5);
    }

    bias_ax = (sum_ax / SAMPLES) / ACC_SENSITIVITY * G_TO_MS2;
    bias_ay = (sum_ay / SAMPLES) / ACC_SENSITIVITY * G_TO_MS2;
    bias_az = (sum_az / SAMPLES) / ACC_SENSITIVITY * G_TO_MS2;
    bias_gx = (sum_gx / SAMPLES) / GYRO_SENSITIVITY;
    bias_gy = (sum_gy / SAMPLES) / GYRO_SENSITIVITY;
    bias_gz = (sum_gz / SAMPLES) / GYRO_SENSITIVITY;

    Serial.printf("[Gyro] Bias Acc  X:%7.3f  Y:%7.3f  Z:%7.3f m/s²\n", bias_ax, bias_ay, bias_az);
    Serial.printf("[Gyro] Bias Gyro X:%7.2f  Y:%7.2f  Z:%7.2f °/s\n",  bias_gx, bias_gy, bias_gz);
    Serial.println("[Gyro] Calibration done");
}

void initGyro() {
    if (!gyro.begin()) {
        Serial.println("[Gyro] Init failed — check wiring");
    } else {
        Serial.println("[Gyro] OK");
        calibrateGyro();
    }
}

// ── MPU-6050 register helpers ─────────────────────────────────────────
static void mpuWriteReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission(true);
}

static uint8_t mpuReadReg(uint8_t reg) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

// ── Motion-wake interrupt setup ───────────────────────────────────────
// Safe to call both at boot and after light-sleep wake (re-arms the MPU registers).
void initGyroMotionWake() {
    mpuWriteReg(0x37, 0x20);  // INT_PIN_CFG: active-high, latch until read
    mpuWriteReg(0x1F, 0x14);  // MOT_THR: ~40 mg
    mpuWriteReg(0x20, 0x01);  // MOT_DUR: 1 ms
    mpuWriteReg(0x38, 0x40);  // INT_ENABLE: MOT_EN bit
    mpuReadReg(0x3A);          // clear any latched interrupt immediately
    delay(5);
    Serial.println("[Gyro] Motion-wake interrupt configured");
}

// Clears the latched motion interrupt by reading INT_STATUS.
void clearGyroMotionInterrupt() {
    mpuReadReg(0x3A);
}

// Returns the raw INT_STATUS byte (register 0x3A).
// Bit 6 (0x40) = motion interrupt fired.
// Also clears the latch as a side effect — call once and act on the result.
uint8_t readGyroIntStatus() {
    return mpuReadReg(0x3A);
}

// ── Update loop (50 Hz) ───────────────────────────────────────────────
static unsigned long lastGyroSendAt = 0;

void updateGyro() {
    unsigned long now = millis();
    if (now - lastGyroSendAt < 20) return;
    lastGyroSendAt = now;

    GyroData raw;
    if (!gyro.read(raw)) {
        Serial.println("[Gyro] Read failed");
        return;
    }

    float ax = ((raw.acc_x  / ACC_SENSITIVITY) * G_TO_MS2) - bias_ax;
    float ay = ((raw.acc_y  / ACC_SENSITIVITY) * G_TO_MS2) - bias_ay;
    float az = ((raw.acc_z  / ACC_SENSITIVITY) * G_TO_MS2) - bias_az;
    float gx = (raw.gyro_x  / GYRO_SENSITIVITY) - bias_gx;
    float gy = (raw.gyro_y  / GYRO_SENSITIVITY) - bias_gy;
    float gz = (raw.gyro_z  / GYRO_SENSITIVITY) - bias_gz;

    pushAvg(ax, ay, az, gx, gy, gz);

    float sax, say, saz, sgx, sgy, sgz;
    calcAvg(sax, say, saz, sgx, sgy, sgz);

    // Activity detection via angular rate (immune to static tilt bias).
    float gyroMag = sqrtf(sgx*sgx + sgy*sgy + sgz*sgz);
    if (gyroMag > GYRO_ACTIVITY_THRESHOLD_DEGS) {
        gyroActivityFlag = true;
    }

    bool tiltForward  = sax >  TILT_THRESHOLD_FB;
    bool tiltBackward = sax < -TILT_THRESHOLD_FB;
    bool tiltLeft     = say >  TILT_THRESHOLD_LR;
    bool tiltRight    = say < -TILT_THRESHOLD_LR;

    bool stateChanged = (tiltForward  != prev_tiltForward)  ||
                        (tiltBackward != prev_tiltBackward) ||
                        (tiltLeft     != prev_tiltLeft)     ||
                        (tiltRight    != prev_tiltRight);

    if (stateChanged) {
        prev_tiltForward  = tiltForward;
        prev_tiltBackward = tiltBackward;
        prev_tiltLeft     = tiltLeft;
        prev_tiltRight    = tiltRight;
        Serial.printf("[Gyro] Tilt  fwd:%d bwd:%d left:%d right:%d  ax:%.3f  ay:%.3f\n",
                      tiltForward, tiltBackward, tiltLeft, tiltRight, sax, say);
        Serial.printf("[Gyro] Acc  X:%7.3f  Y:%7.3f  Z:%7.3f m/s²\n", sax, say, saz);
        Serial.printf("[Gyro] Gyro X:%7.2f  Y:%7.2f  Z:%7.2f °/s\n",  sgx, sgy, sgz);
        sendGyroData(sax, say, saz, sgx, sgy, sgz,
                     tiltForward, tiltBackward, tiltLeft, tiltRight);
    }
}

bool isGyroActive() {
    if (gyroActivityFlag) {
        gyroActivityFlag = false;
        return true;
    }
    return false;
}