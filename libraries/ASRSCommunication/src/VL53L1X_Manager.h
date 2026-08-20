// =====================================================
// VL53L1X_Manager.h
// =====================================================

#ifndef VL53L1X_MANAGER_H
#define VL53L1X_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
// add in the average ( read multiple time and filter the noise) or kalman filter
#if __has_include("SparkFun_VL53L1X.h")
#include "SparkFun_VL53L1X.h"
#define ASRS_HAS_SPARKFUN_VL53L1X 1
#else
#define ASRS_HAS_SPARKFUN_VL53L1X 0
#endif

#if ASRS_HAS_SPARKFUN_VL53L1X

class VL53L1X_Manager
{
public:

    struct SensorConfig
    {
        uint8_t xshutPin;
        uint8_t i2cAddress;
    };

    VL53L1X_Manager(TwoWire &wire = Wire);

    bool begin(SensorConfig *configs,
               uint8_t sensorCount,
               uint8_t sdaPin,
               uint8_t sclPin);

    // Update ALL sensors
    bool updateAll();

    // Update ONLY one sensor
    bool updateSensor(uint8_t index);

    // Keep sensors powered, but range only one sensor during this update.
    bool updateSensorExclusiveRanging(uint8_t index);

    // Power only one sensor, measure it, then shut it down again.
    bool updateSensorExclusivePower(uint8_t index);

    uint16_t getDistance(uint8_t index);

    uint8_t getRangeStatus(uint8_t index);

    uint8_t getSensorCount();

private:

    static const uint8_t MAX_SENSORS = 8;

    TwoWire *_wire;

    SFEVL53L1X _sensors[MAX_SENSORS];

    SensorConfig _configs[MAX_SENSORS];

    uint16_t _distance[MAX_SENSORS];

    uint8_t _rangeStatus[MAX_SENSORS];

    uint8_t _sensorCount;

    bool initSensor(uint8_t index, bool printStatus = true);

    void shutdownAllSensors();

    void stopAllRanging();

    uint16_t readSingleSensor(uint8_t index);
};

#endif

#endif
