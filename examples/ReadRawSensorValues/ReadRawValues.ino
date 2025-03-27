#include <SVKIR8.h>

/**
 * SVK IR 8 Sensor Array - Raw Value Reading
 * 
 * Reads raw analog values (0-1023) from an 8-channel IR sensor array via multiplexer.
 * 
 * Features:
 * - Reads uncalibrated sensor values (0 = max reflectance, 1023 = min reflectance)
 * - Uses 3 digital control pins + 1 analog read pin
 * - Prints tab-separated values for all 8 sensors
 * 
 * Note: For custom pin configuration, use:
 * sensors.setMultiplexerPins({S2_pin, S1_pin, S0_pin, analog_pin});
 */

SVKIR8 sensors;  // Using the correct class name

const uint8_t sensorCount = 8;  // Fixed for 8 sensors
uint16_t sensorValues[8];       // Fixed-size array instead of pointer

void setup()
{
    // Set you multiplexer pins here (These pins are example pins)
    const uint8_t muxPins[4] = {0, 1, 2, A0};
    sensors.setMultiplexerPins(muxPins);
    
    Serial.begin(9600);
    Serial.println("SVK IR Sensor Array - Raw Mode");
    Serial.println("Reading uncalibrated values (0-1023)");
    Serial.println("-----------------------------------");
}

void loop()
{
    // Read raw analog values (0-1023)
    sensors.read();  // Uses readPrivate() internally
    
    // Get current sensor values
    uint16_t* currentValues = sensors.getSensorValues();
    memcpy(sensorValues, currentValues, sizeof(sensorValues));
    
    // Print all sensor values (tab-separated)
    for (uint8_t i = 0; i < sensorCount; i++) {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();  // New line after each complete reading
    
    delay(500);  // Adjust delay as needed for your application
}