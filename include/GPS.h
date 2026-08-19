#pragma once
#include <TinyGPSPlus.h>

class GPSModule {
public:
    TinyGPSPlus gps;

    double latitude = 0;
    double longitude = 0;
    double sog = 0;
    double cog = 0;
    int satellites = 0;
    double hdop = 0;

    void begin();
    void update();

    bool hasDateTime();

    int year();
    int month();
    int day();

    int hour();
    int minute();
    int second();
};
