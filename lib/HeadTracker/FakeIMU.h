#ifndef BACKPACK_FAKEIMU_H
#define BACKPACK_FAKEIMU_H

#include "IMUBase.h"

class FakeIMU : public IMUBase {
public:
    FakeIMU() : IMUBase(0x00) {
        sampleRate = 100;
        gyroRange = 2000.0f;
        gRes = gyroRange / 32768.0f;
    }

    bool initialize() override { return true; }
    bool isFake() const override { return true; }

protected:
    bool getDataFromRegisters(FusionVector &accel, FusionVector &gyro) override {
        accel = {.axis = {.x = 0.0f, .y = 0.0f, .z = 1.0f}};
        gyro  = {.axis = {.x = 0.0f, .y = 0.0f, .z = 0.0f}};
        return true;
    }
};

#endif
