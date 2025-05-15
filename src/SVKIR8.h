/// Header file for reading IR Follow Line Sensors using multiplexer in SVK Robotics Line Follow Robot
#pragma once

#include <Arduino.h>
#include <math.h>


/// @brief Class for reading IR values using Multiplexer
class SVKIR8 {
    public:
        /// @brief Default constructor - uses default multiplexer pins
        SVKIR8();
        
        /// @brief Overloaded constructor with custom multiplexer pins
        /// @param pins Array of 4 pins (3 digital signal pins + 1 analog input pin)
        SVKIR8(const uint8_t *pins);

        /// @brief Sets Multiplexer 3 Digital Signal pins and Multiplexer Analog Output pin using default pins.
        void setMultiplexerPins();

        /// @brief Sets custom multiplexer pins
        /// @param pins Array of 4 pins (3 digital signal pins + 1 analog input pin)
        void setMultiplexerPins(const uint8_t *pins);

        /// @brief Returns the whole sensor value array read from each sensor via pointer. WARNING do not modify any values, as pointer is showing to _sensorValues address and will alter any values saved!
        /// @return Pointer pointing to _sensorValues array to be viewed in main program, (USE AS READ ONLY).
        uint16_t* getSensorValues() { return _sensorValues; }

        /// @brief Sets number of analog readings to average per analog sensor. WARNING for best performance, number of samples SHOULD BE a power of 2 (for example 1, 2, 4, 8 etc...).
        /// @param samples Number of samples
        void setSamplesPerSensor(uint8_t samples);

        /// @brief Returns number of analog samples per sensor
        /// @return Number of samples
        uint8_t getSamplesPerSensor() { return _samplesPerSensor; };

        /**
         * @brief Sets if user wants the robot to calibrate before running or not
         * 
         * @param calibrationMode Bool variable for: 
         *                       - true for calibration on
         *                       - false for calibration off
         */
        void setCalibrationMode(bool calibrationMode) { _calibrateOn = calibrationMode; };

        /// @brief Gets calibration mode (on or off)
        /// @return True for calibration on, False for calibration off
        bool getCalibrationMode() { return _calibrateOn; }

        /// @brief Stores sensor calibration data
        struct CalibrationData
        {
            /// @brief checks whether array pointers have been allocated and initialized
            bool initialized = false;
            /// @brief Minimum reading read during calibration
            uint16_t minimum[8];
            /// @brief Maximum reading read during calibration
            uint16_t maximum[8];
        };

        /// @brief Calibration data
        CalibrationData _calibration;

        /// @brief Calibrates Robot to black line values
        void calibrate();

        /// @brief Resets Robot Calibration values (if there are any values)
        void resetCalibration();

        /// @brief Reads IR sensor array raw analog values
        void read();

        /// @brief Reads IR sensors and gives calibrated values from 0 to 1000
        void readCalibrated();

        /// @brief Detects if robot is seeing black line and returns its position based on sensor readings
        /// @return Number representing position of the black line using sensor (number from 0 - 7000)
        uint16_t readLineBlack();


    private:
        // Amount of IR sensors
        static const uint8_t _sensorAmount = 8;

        // Multiplexer signal pins and output pin
        uint8_t _muxPins[4] = { 7, 4, 2, A7 };

        // Array of sensor values
        uint16_t _sensorValues[8];

        // checks if calibration is on and should the robot calibrate or not
        bool _calibrateOn = true;
        // Sets the amount of samples that should be taken for each sensor. Higher number reduces noise but slows down the process
        uint8_t _samplesPerSensor = 1;
        // Sets bit shift amount based on the log2 result of the _samplesPerSensor, used in the averaging of the sensor values
        uint8_t _shiftAmount = 0;
        // Sets max value readLine should see
        uint16_t _maxValue = 1023;
        // Last known line position
        uint16_t _lastPosition = 0;

        /// Selects channel of multiplexer in order to read sensor values
        void selectChannel(uint8_t channel);

        /// @brief Handles actually calibrating robot
        void calibratePrivate();

        /// Reads sensors
        void readPrivate();

        /// Sees if robot is seeing a line
        uint16_t readLinesPrivate();
};