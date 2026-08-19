#pragma once
#include <stdint.h>
#include <N2kMsg.h>

// Structure satellite utilisée par PGN 129540
struct tSatelliteInfo {
    uint8_t PRN;
    uint8_t Elevation;
    uint16_t Azimuth;
    uint8_t SNR;
    double RangeResiduals;
};

// Fonctions d’envoi des PGN satellites
void sendN2kSatellitesInUse();   // PGN 129539 (DOPs)
void sendN2kSatellitesInView();  // PGN 129540 (Satellites in View)
