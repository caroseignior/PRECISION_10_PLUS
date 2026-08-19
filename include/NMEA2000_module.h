#ifndef NMEA2000_MODULE_H
#define NMEA2000_MODULE_H

// --- Données moteur / environnement / navigation ---
// (mises à jour par le handler NMEA2000 dans NMEAEngine.cpp)

extern double fuelRate_Lh;     // L/h
extern double fuelUsed_L;      // Litres cumulés
extern double seaTemp_C;       // °C
extern double speedWater_kn;   // Noeuds
extern double depth_m;         // Mètres

// --- Flags de présence des données ---
extern bool hasFuelRate;
extern bool hasFuelUsed;
extern bool hasTempSea;
extern bool hasSpeed;
extern bool hasDepth;

#endif
