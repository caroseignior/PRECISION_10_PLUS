#include "EnvManager.h"

Adafruit_BME280 EnvManager::bme;
bool EnvManager::ok = false;

void EnvManager::begin() {
    Serial.println("Init BME280...");
    ok = bme.begin(0x76);   // Ton scan confirme 0x76

    if (ok) Serial.println("BME280 OK");
    else    Serial.println("BME280 NOT FOUND");
}

void EnvManager::read(float &t_air, float &p_air, float &h_air) {
    if (!ok) {
        Serial.println("BME280 NOT READY");
        t_air = 20;
        p_air = 101325;
        h_air = 50;
        return;
    }

    t_air = bme.readTemperature();
    p_air = bme.readPressure();
    h_air = bme.readHumidity();
}
