#pragma once
#include <stdint.h>

#define SATPARSER_MAX_SATS 32

// Structure interne pour stocker les satellites
struct SatInfo {
    uint8_t prn;
    int8_t  elevation;
    int16_t azimuth;
    uint8_t snr;
    bool    usedInFix;
};

// API publique du parseur Osiris
void SatParser_begin();
void SatParser_reset();
void SatParser_encodeChar(char c);

// Accès aux données
uint8_t        SatParser_getSatInViewCount();
uint8_t        SatParser_getSatInUseCount();
const SatInfo* SatParser_getSatList();
uint8_t        SatParser_getSatListSize();
float          SatParser_getHDOP();
