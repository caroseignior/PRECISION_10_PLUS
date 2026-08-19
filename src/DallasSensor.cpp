#include <Arduino.h>
#include "DallasSensor.h"

DallasSensor::DallasSensor(uint8_t pin)
    : oneWire(pin), sensors(&oneWire) {}

void DallasSensor::begin() {
    sensors.begin();
    sensors.setWaitForConversion(false);   // IMPORTANT : non bloquant
}

void DallasSensor::update() {
    unsigned long now = millis();

    // Conversion DS18B20 toutes les 1000 ms
    if (now - lastRequest > 1000) {
        sensors.requestTemperatures();     // NON BLOQUANT
        lastTemp = sensors.getTempCByIndex(0);
        lastRequest = now;
    }
}

float DallasSensor::getTempC() {
    return lastTemp;
}
