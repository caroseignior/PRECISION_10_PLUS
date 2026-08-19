#pragma once
#include <Adafruit_BME280.h>

class Pressure {
public:
    void begin();
    float temperature();   // °C
    float pressure();      // Pa
    float humidity();      // %
    float altitude();      // m (optionnel)

private:
    Adafruit_BME280 bme;
    bool ok = false;
};
