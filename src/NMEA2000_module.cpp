#include "NMEA2000_Module.h"

// --- Données NMEA2000 (mises à jour par le handler) ---
double fuelRate_Lh   = 0.0;   // L/h
double fuelUsed_L    = 0.0;   // Litres cumulés
double seaTemp_C     = 0.0;   // °C
double speedWater_kn = 0.0;   // Noeuds
double depth_m       = 0.0;   // Mètres

// --- Flags de présence ---
bool hasFuelRate = false;
bool hasFuelUsed = false;
bool hasTempSea  = false;
bool hasSpeed    = false;
bool hasDepth    = false;
