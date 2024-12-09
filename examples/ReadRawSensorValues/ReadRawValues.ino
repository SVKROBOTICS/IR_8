#include <SVKTigerSensors.h>;

/***
 * This is an example code for reading Raw Values from the SVKLine Follow robot created by
 * SVKRobotics. This robot controls and reads the IR Sensors using a 8-1 Multiplexer, that we use
 * 3 Signal Digital Pins to control what sensor is to be read, and then the Multiplexer Output to
 * read the analog value of each sensor.
 * 
 * 
 * Inside the Main loop the program will read the values of the black line while using the raw
 * values. The sensors will print to the serial monitor the values it's
 * currently seeing, which will be a number from 0 (maximum reflectance) to 1023(minimum reflectance) for
 * each sensor. That means 0 for white background and 1023 for the center of the black line.
 * 
 * 
*/


SVKTigerSensors sensors;


const uint8_t sensorCount = sensors.getSensorAmount();
uint16_t* sensorValues = nullptr;


void setup()
{
    sensors.setMultiplexerPins();
    Serial.begin(9600);
}


void loop()
{
    // Read Raw Analog values from sensors
    sensors.read();

    // Store Sensor values array to variable
    sensorValues = sensors.getSensorValues();

    // Print the sensor values as numbers from 0 to 1023, where 0 means maximum
    // reflectance and 1023 means minimum reflectance
    for (uint8_t i = 0; i < sensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println();

    delay(500);
}
