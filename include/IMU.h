#pragma once
#include <Arduino.h>

// ---------------------------------------------------------
// Données IMU remappées dans le repère bateau
// X = avant, Y = tribord, Z = bas
// ---------------------------------------------------------
struct IMUData {
    // Accéléromètre (repère bateau)
    float ax;    // m/s², axe avant
    float ay;    // m/s², axe tribord
    float az;    // m/s², axe bas (vers la mer)

    // Gyroscope (repère bateau)
    float gx;    // rad/s, rotation autour X (roulis)
    float gy;    // rad/s, rotation autour Y (tangage)
    float gz;    // rad/s, rotation autour Z (lacet)

    // Magnétomètre calibré (repère bateau)
    float mx;    // axe avant
    float my;    // axe tribord
    float mz;    // axe bas

    // Attitude IMU brute (radians, calculée à partir de l'accéléromètre)
    float roll;      // radians, roulis
    float pitch;     // radians, tangage

    // Heading magnétomètre (degrés 0–360, avant fusion)
    float heading;   // degrés, cap mag brut (debug / calibration)
};

// ---------------------------------------------------------
// Orientation fusionnée (Madgwick / Mahony) en degrés
// Source unique pour écran, NMEA2000, BT
// ---------------------------------------------------------
struct Orientation {
    float pitch;     // degrés, tangage fusionné
    float roll;      // degrés, roulis fusionné
    float heading;   // degrés (0–360), cap fusionné
};

// ---------------------------------------------------------
// Fonctions IMU
// ---------------------------------------------------------
void imuInit();
void imuRead(IMUData &d);
void imuReadRawMag(float &mx, float &my, float &mz);
