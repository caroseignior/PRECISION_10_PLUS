#include <Arduino.h>
#include "Pressure.h"

void Pressure::begin() {
    // Adresse la plus courante : 0x76 ou 0x77
    ok = bme.begin(0x77);   // ← ton scan I2C montre 0x77

    if (!ok) {
        Serial.println("BME280 non detecte !");
        return;
    }

    // Configuration stable pour environnement marin
    bme.setSampling(
        Adafruit_BME280::MODE_NORMAL,
        Adafruit_BME280::SAMPLING_X2,     // Température
        Adafruit_BME280::SAMPLING_X16,    // Pression
        Adafruit_BME280::SAMPLING_X1,     // Humidité (BME only)
        Adafruit_BME280::FILTER_X16,
        Adafruit_BME280::STANDBY_MS_500
    );

    Serial.println("BME280 OK");
}

float Pressure::temperature() {
    return ok ? bme.readTemperature() : NAN;   // °C
}

float Pressure::pressure() {
    return ok ? bme.readPressure() : NAN;      // Pa
}

float Pressure::humidity() {
    return ok ? bme.readHumidity() : NAN;      // %
}

float Pressure::altitude() {
    return ok ? bme.readAltitude(1013.25) : NAN;  // m
}
