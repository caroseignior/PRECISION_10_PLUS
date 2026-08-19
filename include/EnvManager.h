#ifndef ENV_MANAGER_H
#define ENV_MANAGER_H

#include <Arduino.h>
#include <Adafruit_BME280.h>

class EnvManager {
public:
    static void begin();
    static void read(float &t_air, float &p_air, float &h_air);

private:
    static Adafruit_BME280 bme;
    static bool ok;
};

#endif
