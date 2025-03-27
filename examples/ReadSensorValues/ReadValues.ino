#include <SVKIR8.h>

/**
 * SVK IR 8 Sensor Array for Line Following Robots
 * 
 * This library controls an array of 8 IR sensors through a multiplexer for line following applications.
 * Features include:
 * - Automatic calibration to distinguish between line and background
 * - Configurable multiplexer pins
 * - Sensor readings scaled from 0 (white) to 1000 (black)
 * - Line position detection from 0-7000 (8 sensors)
 * 
 * The sensor array uses a 8:1 multiplexer controlled by 3 digital pins,
 * with one analog pin reading the multiplexer output.
 * 
 * Note: For custom pin configuration, use:
 * sensors.setMultiplexerPins({S2_pin, S1_pin, S0_pin, analog_pin});
 */

SVKIR8 sensors; // Instance of sensor class

const uint8_t sensorCount = 8;
uint16_t sensorValues[8];

void setup()
{
    // Set your multiplexer pins here (These pins are example pins)
    const uint8_t muxPins[4] = {0, 1, 2, A0};
    sensors.setMultiplexerPins(muxPins);

    delay(500);

    // Calibration sequence
    Serial.begin(9600);
    Serial.println("Starting calibration...");

    for(uint16_t i = 0; i < 100; i++)
    {
        sensors.calibrate();
    }

    // Print calibration results
    Serial.println("Calibration results (min/max):");
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensors._calibration.minimum[i]);
        Serial.print(' ');
    }
    Serial.println();

    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensors._calibration.maximum[i]);
        Serial.print(' ');
    }
    Serial.println("\nCalibration complete\n");
    delay(1000);
}

void loop()
{
    // Read sensor values and line position
    uint16_t position = sensors.readLineBlack();
    uint16_t* currentValues = sensors.getSensorValues();
    
    // Copy values to local array
    memcpy(sensorValues, currentValues, sizeof(sensorValues));

    // Display sensor readings
    Serial.println("Sensor values (0-1000):");
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print("Sensor ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(sensorValues[i]);
    }
    
    // Display line position
    Serial.print("Line position: ");
    Serial.println(position);
    Serial.println("-------------------");
    
    delay(1000);
}