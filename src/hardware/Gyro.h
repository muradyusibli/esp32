#pragma once
#include <stdint.h>

struct GyroData {
    int16_t acc_x, acc_y, acc_z;
    int16_t gyro_x, gyro_y, gyro_z;
};

class Gyro {
public:
    bool begin();
    bool read(GyroData& out);
};

void     initGyro();
void     updateGyro();
bool     isGyroActive();
void     initGyroMotionWake();
void     clearGyroMotionInterrupt();
uint8_t  readGyroIntStatus();