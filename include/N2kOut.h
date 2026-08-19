#ifndef N2KOUT_H
#define N2KOUT_H

#include <NMEA2000.h>
#include <N2kMessages.h>

// Offset manuel heading (en degrés)
extern float manualHeadingOffset;

// --- Navigation ---
void sendN2kPosition(double lat, double lon, uint8_t SID);
void sendN2kCOGSOG(double cogDeg, double sogKn, uint8_t SID);
void sendN2kMagneticVariation(double variation_rad, uint8_t SID);


// Heading en RADIANS (corrigé)
void sendN2kHeading(double headingRad, uint8_t SID);

// Attitude (pitch / roll)
void sendN2kAttitude(float pitchDeg, float rollDeg, uint8_t SID);

// --- Environnement ---
void sendN2kAirTemperature(float tempC, uint8_t SID);
void sendN2kHumidity(float humPercent, uint8_t SID);
void sendN2kPressure(float pressPa, uint8_t SID);
void sendN2kWaterTemperature(float tempC, uint8_t SID);

// --- Temps / GNSS ---
void sendN2kDateTime(uint16_t days, double seconds, uint8_t SID);
void sendN2kGNSSDOP(float hdop, float vdop, float tdop, uint8_t SID);

// --- GNSS Lowrance (PGN 129029 étendu) ---
void sendN2kGNSS_Lowrance(
    double lat,
    double lon,
    uint16_t days,
    double seconds,
    uint8_t sats,
    double hdop,
    double epe,
    uint8_t SID
);

// --- Satellites ---
void sendN2kSatellitesInUse();     // HDOP/VDOP/TDOP
void sendN2kSatellitesInView();    // PRN, elev, azim, SNR, used

// --- PGN propriétaire satellites ---
void sendProprietarySatInfo(
    uint8_t SID,
    uint8_t prn,
    uint8_t elev,
    uint16_t azim,
    uint8_t snr,
    bool used,
    float hdop,
    float epe,
    uint8_t satCount
);

void sendN2kRateOfTurn(double rot_rad_per_s, uint8_t SID);

#endif
