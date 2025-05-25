#ifndef MAX6675HANDLER_H
#define MAX6675HANDLER_H

#include <Arduino.h>
#include <SPI.h>

// MAX6675Handler class declaration
class MAX6675Handler {
public:
    MAX6675Handler(int csPin); // Constructor to initialize CS pin
    void begin();              // Initialize SPI and CS pin
    float readTemperature();   // Get temperature in Celsius

private:
    int _csPin;                // Chip Select pin
    uint16_t readRawData();    // Read raw SPI data
};

#endif