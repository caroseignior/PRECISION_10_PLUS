#include "GPS.h"
#include <Arduino.h>
#include "SystemConfig.h"   // GPS_BAUD, GPS_RX_PIN, GPS_TX_PIN

void GPSModule::begin() {
    Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
}

void GPSModule::update() {

    while (Serial2.available()) {
        gps.encode(Serial2.read());
    }

    if (gps.location.isValid()) {
        latitude  = gps.location.lat();
        longitude = gps.location.lng();
    }

    if (gps.speed.isValid()) {
        sog = gps.speed.knots();
    }

    if (gps.course.isValid()) {
        cog = gps.course.deg();
    }

    if (gps.satellites.isValid()) {
        satellites = gps.satellites.value();
    }

    if (gps.hdop.isValid()) {
        hdop = gps.hdop.hdop();
    }
}

// --- DATE / HEURE ---

bool GPSModule::hasDateTime() {
    return gps.date.isValid() && gps.time.isValid();
}

int GPSModule::year() {
    return gps.date.year();
}

int GPSModule::month() {
    return gps.date.month();
}

int GPSModule::day() {
    return gps.date.day();
}

int GPSModule::hour() {
    return gps.time.hour();
}

int GPSModule::minute() {
    return gps.time.minute();
}

int GPSModule::second() {
    return gps.time.second();
}
