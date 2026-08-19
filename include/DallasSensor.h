#pragma once
#include <OneWire.h>
#include <DallasTemperature.h>

class DallasSensor {
public:
    DallasSensor(uint8_t pin);

    void begin();
    void update();        // Non bloquant
    float getTempC();     // Dernière valeur connue

private:
    OneWire oneWire;
    DallasTemperature sensors;

    unsigned long lastRequest = 0;
    float lastTemp = NAN;
};
