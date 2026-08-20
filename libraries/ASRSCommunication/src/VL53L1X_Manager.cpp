// =====================================================
// VL53L1X_Manager.cpp
// =====================================================

#include "VL53L1X_Manager.h"

#if ASRS_HAS_SPARKFUN_VL53L1X

// =====================================================
// Constructor
// =====================================================

VL53L1X_Manager::VL53L1X_Manager(TwoWire &wire)
{
    _wire = &wire;

    _sensorCount = 0;
}

// =====================================================
// Begin
// =====================================================

bool VL53L1X_Manager::begin(SensorConfig *configs,
                            uint8_t sensorCount,
                            uint8_t sdaPin,
                            uint8_t sclPin)
{
    if (sensorCount > MAX_SENSORS)
    {
        return false;
    }

    _sensorCount = sensorCount;

#if defined(ESP32)
    _wire->begin(sdaPin, sclPin);
#else
    (void)sdaPin;
    (void)sclPin;
    _wire->begin();
#endif

    // Lower I2C speed for stability
    _wire->setClock(100000);

    // Configure pins
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        _configs[i] = configs[i];

        pinMode(_configs[i].xshutPin, OUTPUT);

        digitalWrite(_configs[i].xshutPin, LOW);

        _distance[i] = 65535;

        _rangeStatus[i] = 255;
    }

    delay(500);

    // Initialize sensors one-by-one
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        if (!initSensor(i))
        {
            Serial.print("VL53 Init Failed: ");

            Serial.println(i);

            return false;
        }
    }

    Serial.println("VL53L1X Manager Ready");

    return true;
}

// =====================================================
// Initialize Sensor
// =====================================================

bool VL53L1X_Manager::initSensor(uint8_t index, bool printStatus)
{
    // Power ON sensor
    digitalWrite(_configs[index].xshutPin, HIGH);

    delay(300);

    // Initialize sensor
    if (_sensors[index].begin() != 0)
    {
        return false;
    }

    delay(50);

    // Set unique address
    _sensors[index].setI2CAddress(
        _configs[index].i2cAddress);

    delay(50);

    // Recommended settings
    _sensors[index].setDistanceModeLong();

    _sensors[index].setTimingBudgetInMs(20);

    delay(50);

    if (printStatus)
    {
        Serial.print("Sensor ");

        Serial.print(index);

        Serial.println(" Initialized");
    }

    return true;
}

void VL53L1X_Manager::shutdownAllSensors()
{
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
        digitalWrite(_configs[i].xshutPin, LOW);
    }
}

void VL53L1X_Manager::stopAllRanging()
{
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
        _sensors[i].stopRanging();
    }
}

// =====================================================
// Read ONE sensor
// Sequential ranging
// =====================================================

uint16_t VL53L1X_Manager::readSingleSensor(uint8_t index)
{
    // Start ranging
    _sensors[index].startRanging();

    // Allow VCSEL startup
    delay(5);

    uint32_t startTime = millis();

    // Wait for measurement ready
    while (!_sensors[index].checkForDataReady())
    {
        if (millis() - startTime > 100)
        {
            _sensors[index].stopRanging();

            _rangeStatus[index] = 255;

            return 65535;
        }

        delay(1);
    }

    // Read distance
    _rangeStatus[index] =
        _sensors[index].getRangeStatus();

    uint16_t distance =
        _sensors[index].getDistance();

    // Clear interrupt
    _sensors[index].clearInterrupt();

    // Stop ranging
    _sensors[index].stopRanging();

    // Small cooldown
    delay(10);

    return distance;
}

// =====================================================
// Update ONE sensor
// =====================================================

bool VL53L1X_Manager::updateSensor(uint8_t index)
{
    if (index >= _sensorCount)
    {
        return false;
    }

    _distance[index] =
        readSingleSensor(index);

    return true;
}

bool VL53L1X_Manager::updateSensorExclusiveRanging(uint8_t index)
{
    if (index >= _sensorCount)
    {
        return false;
    }

    stopAllRanging();
    delay(2);

    _distance[index] =
        readSingleSensor(index);

    stopAllRanging();

    return true;
}

bool VL53L1X_Manager::updateSensorExclusivePower(uint8_t index)
{
    if (index >= _sensorCount)
    {
        return false;
    }

    shutdownAllSensors();
    delay(10);

    if (!initSensor(index, false))
    {
        _distance[index] = 65535;
        shutdownAllSensors();
        delay(10);
        return false;
    }

    _distance[index] =
        readSingleSensor(index);

    shutdownAllSensors();
    delay(10);

    return true;
}

// =====================================================
// Update ALL sensors
// =====================================================

bool VL53L1X_Manager::updateAll()
{
    for (uint8_t i = 0; i < _sensorCount; i++)
    {
        updateSensor(i);
    }

    return true;
}

// =====================================================
// Get Distance
// =====================================================

uint16_t VL53L1X_Manager::getDistance(uint8_t index)
{
    if (index >= _sensorCount)
    {
        return 65535;
    }

    return _distance[index];
}

uint8_t VL53L1X_Manager::getRangeStatus(uint8_t index)
{
    if (index >= _sensorCount)
    {
        return 255;
    }

    return _rangeStatus[index];
}

// =====================================================
// Get Sensor Count
// =====================================================

uint8_t VL53L1X_Manager::getSensorCount()
{
    return _sensorCount;
}

#endif
