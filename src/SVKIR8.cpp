#include <SVKIR8.h>

#include <Arduino.h>
#include <stdint.h>
#include <stdlib.h>

#define DEBUG_MODE 0

// Default constructor - uses default multiplexer pins
SVKIR8::SVKIR8()
{
    // Initialize with default multiplexer pins
    setMultiplexerPins();
}

// Overloaded constructor with custom multiplexer pins
SVKIR8::SVKIR8(const uint8_t *pins)
{
    // Initialize with custom multiplexer pins
    setMultiplexerPins(pins);
}

/// @brief Sets Default Multiplexer pins for typical SVK Tiger Robot
void SVKIR8::setMultiplexerPins()
{
    // sets up the pinModes for digital signal pins of multiplexer (first 3 pins)
    pinMode(_muxPins[0], OUTPUT);
    pinMode(_muxPins[1], OUTPUT);
    pinMode(_muxPins[2], OUTPUT);

    /// Re-initializes Calibration of robot since Pins have changed
    _calibration.initialized = false;
}

/// @brief Overloaded method for custom multiplexer pins, rarely needed only if someone changed the pins manually on PCB
/// @param pins new multiplexer pins in an array
void SVKIR8::setMultiplexerPins(const uint8_t *pins) {
    // Override the default _muxPins with the provided pins array
    for (uint8_t i = 0; i < 4; i++) {
        _muxPins[i] = pins[i];  // Copy values from the input array to _muxPins
    }

    // Set up the pinModes for the new _muxPins
    pinMode(_muxPins[0], OUTPUT);
    pinMode(_muxPins[1], OUTPUT);
    pinMode(_muxPins[2], OUTPUT);

    // Re-initialize calibration since Pins have changed
    _calibration.initialized = false;
}

// Since log2 doesnt exist in AVR processors, we are using integer representation of log2 using shifting
void SVKIR8::setSamplesPerSensor(uint8_t samples)
{
    if (samples > 64) { samples = 64; }
    _samplesPerSensor = samples;

    // Check if _samplesPerSensor is a power of 2
    if ((_samplesPerSensor & (_samplesPerSensor - 1)) == 0) {
        // Integer version of log2
        uint8_t shift = 0;
        uint8_t temp = _samplesPerSensor;
        while (temp >>= 1) {
            shift++;
        }
        _shiftAmount = shift;
    } else {
        _shiftAmount = 0;
    }
}

void SVKIR8::calibrate()
{
    if(!_calibrateOn) { return; }
    calibratePrivate();
}

void SVKIR8::resetCalibration()
{
  for (uint8_t i = 0; i < _sensorAmount; i++)
  {
    if (_calibration.maximum)   { _calibration.maximum[i] = 0; }
    if (_calibration.minimum)   { _calibration.minimum[i] = _maxValue; }
  }
}

void SVKIR8::read()
{
    readPrivate();
}

void SVKIR8::readCalibrated()
{
    if(!_calibration.initialized)
    {
        Serial.println("Not Calibrated");
        return;
    }

    read();

    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
        uint16_t calmin, calmax;

        calmax = _calibration.maximum[i];
        calmin = _calibration.minimum[i];

        uint16_t denominator = calmax - calmin;
        int16_t value = 0;

        if (denominator != 0)
        {
        value = (((int32_t)_sensorValues[i]) - calmin) * 1000 / denominator;
        }

        if (value < 0) 
        { 
          value = 0; 
        }
        else if (value > 1000)
        {
           value = 1000; 
        }

        _sensorValues[i] = value;
    }
}

uint16_t SVKIR8::readLineBlack()
{
    return readLinesPrivate();
}

void SVKIR8::selectChannel(uint8_t sensorNum)
{
    // Truth table for the multiplexer signal pins
    const uint8_t muxPinLayout[] = { 0b110, 0b111, 0b011, 0b010, 0b001, 0b100, 0b000, 0b101 };

    // Get the channel configuration for the specified sensor
    uint8_t channelBits = muxPinLayout[sensorNum];

    // Write each bit to the corresponding digital pin
    for (uint8_t i = 0; i < 3; i++) {
        digitalWrite(_muxPins[i], (channelBits >> i) & 0x01);
    }
}

void SVKIR8::calibratePrivate()
{
    uint16_t maxSensorValues[_sensorAmount];
    uint16_t minSensorValues[_sensorAmount];

    // (Re)initialize the arrays if necessary.
    if (!_calibration.initialized)
    {
        // Initialize the max and min calibrated values to values that
        // will cause the first reading to update them.
        for (uint8_t i = 0; i < _sensorAmount; i++)
        {
            _calibration.maximum[i] = 0;        // Initialize max to 0
            _calibration.minimum[i] = _maxValue; // Initialize min to _maxValue (highest possible value)
        }

        _calibration.initialized = true; // Mark as initialized
    }

    // Loop to read the sensor values multiple times
    for (uint8_t j = 0; j < 10; j++)
    {
        read();  // Read the sensor values into _sensorValues array

        for (uint8_t i = 0; i < _sensorAmount; i++)
        {
            // Update the max sensor values
            if ((j == 0) || (_sensorValues[i] > maxSensorValues[i]))
            {
                maxSensorValues[i] = _sensorValues[i];
            }

            // Update the min sensor values
            if ((j == 0) || (_sensorValues[i] < minSensorValues[i]))
            {
                minSensorValues[i] = _sensorValues[i];
            }
        }
    }

    // Record the min and max calibration values in the calibration structure
    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
        // Update the maximum if the current min value was higher than the previously recorded max
        if (minSensorValues[i] > _calibration.maximum[i])
        {
            _calibration.maximum[i] = minSensorValues[i];
        }

        // Update the minimum if the current max value was lower than the previously recorded min
        if (maxSensorValues[i] < _calibration.minimum[i])
        {
            _calibration.minimum[i] = maxSensorValues[i];
        }
    }
}


void SVKIR8::readPrivate()
{

    memset(_sensorValues, 0, sizeof(_sensorValues));

    for (uint8_t j = 0; j < _samplesPerSensor; j++)
    {
        for (uint8_t i = 0; i < _sensorAmount; i++)
        {
            // add the conversion result
            selectChannel(i);
            _sensorValues[i] += analogRead(_muxPins[3]);
        }
    }

    for (uint8_t i = 0; i < _sensorAmount; i++) {
        if (_shiftAmount > 0) {
            // Use bit-shift averaging for power-of-2 values
            _sensorValues[i] = (_sensorValues[i] + (1 << (_shiftAmount - 1))) >> _shiftAmount;
        } else {
            // Fall back to division for non-power-of-2 values
            _sensorValues[i] = (_sensorValues[i] + (_samplesPerSensor >> 1)) / _samplesPerSensor;
        }
    }

}

uint16_t SVKIR8::readLinesPrivate()
{
    uint8_t onLineFlag = 0; // Flag to track if any line is detected, 0 = no line, 1 = line detected
    uint32_t avg = 0;       // This is for the weighted total
    uint16_t sum = 0;       // This is for the denominator, which is <= 64000

    readCalibrated();

    for (uint8_t i = 0; i < _sensorAmount; i++)
    {
        uint16_t value = _sensorValues[i];

        // Set flag if line is detected
        if (value > 200) {
            onLineFlag = 1;
        }

        // Only average in values that are above a noise threshold
        if (value > 50)
        {
            avg += (uint32_t)value * (i * 1000);
            sum += value;
        }
    }

    // If no line was detected, determine if it was lost to the left or right
    if (onLineFlag == 0)
    {
        if (_lastPosition < (_sensorAmount - 1) * 1000 / 2)
        {
    #if DEBUG_MODE
            Serial.println("Lost line from left side");
    #endif
            return 0;
        }
        else
        {
    #if DEBUG_MODE
            Serial.println("Lost line from right side");
    #endif
            return (_sensorAmount - 1) * 1000;
        }
    }

    // Calculate position if line is detected
    _lastPosition = avg / sum;
    return _lastPosition;
}
