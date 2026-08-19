#include "N2kOut.h"
#include <N2kMessages.h>
#include <NMEA2000.h>
#include "sat_parser.h"

extern tNMEA2000 &NMEA2000;

// ---- MAGNETIC VARIATION -----

#include <N2kMessages.h>
#include <NMEA2000.h>

void sendN2kMagneticVariation(double variation_rad, uint8_t SID) {
    tN2kMsg N2kMsg;

    SetN2kMagneticVariation(
        N2kMsg,
        SID,
        N2kmagvar_Manual,   // Source de la variation
        0,                  // DaysSince1970 (0 = non spécifié)
        variation_rad       // Variation en radians
    );

    NMEA2000.SendMsg(N2kMsg);
}



//--- RATE OF TURN ----
void sendN2kRateOfTurn(double rot_rad_per_s, uint8_t SID) {
    tN2kMsg N2kMsg;

    SetN2kRateOfTurn(
        N2kMsg,
        SID,             // Sequence ID
        rot_rad_per_s    // Rate of turn en rad/s
    );

    NMEA2000.SendMsg(N2kMsg);
}

// --- POSITION ---
void sendN2kPosition(double lat, double lon, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN129025(msg, lat, lon);
    msg.Priority = 2;
    NMEA2000.SendMsg(msg);
}

// --- COG / SOG ---
void sendN2kCOGSOG(double cogDeg, double sogKn, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN129026(msg, SID, N2khr_true, DegToRad(cogDeg), KnotsToms(sogKn));
    msg.Priority = 2;
    NMEA2000.SendMsg(msg);
}

// --- HEADING (radians) ---
void sendN2kHeading(double headingRad, uint8_t SID) {

    // offset manuel en radians
    double headingRadCorrected = headingRad + DegToRad(manualHeadingOffset);

    // normalisation
    if (headingRadCorrected < 0) headingRadCorrected += 2*M_PI;
    if (headingRadCorrected >= 2*M_PI) headingRadCorrected -= 2*M_PI;

    tN2kMsg msg;
    SetN2kPGN127250(msg, SID, headingRadCorrected, 0, 0, N2khr_true);
    msg.Priority = 2;
    NMEA2000.SendMsg(msg);
}


// --- (ATTENTION : sendN2kAttitude est dans N2kAttitude.cpp) ---


// --- AIR TEMPERATURE ---
void sendN2kAirTemperature(float tempC, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN130316(msg, SID, 0, N2kts_OutsideTemperature, CToKelvin(tempC), N2kDoubleNA);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

void sendN2kHumidity(float humidityPercent, uint8_t SID) {
    tN2kMsg msg;
    float humForLowrance = humidityPercent * 100.0f;
    SetN2kPGN130313(msg, SID, 0, N2khs_InsideHumidity, humForLowrance, N2kDoubleNA);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

// --- PRESSURE ---
void sendN2kPressure(float pressPa, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN130314(msg, SID, 0, N2kps_Atmospheric, pressPa);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

// --- WATER TEMPERATURE ---
void sendN2kWaterTemperature(float tempC, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN130316(msg, SID, 1, N2kts_SeaTemperature, CToKelvin(tempC), N2kDoubleNA);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

// --- DATE / TIME ---
void sendN2kDateTime(uint16_t days, double seconds, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN126992(msg, SID, days, seconds, N2ktimes_GPS);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

// --- GNSS DOP ---
void sendN2kGNSSDOP(float hdop, float vdop, float tdop, uint8_t SID) {
    tN2kMsg msg;
    SetN2kPGN129539(msg, SID, N2kGNSSdm_Auto, N2kGNSSdm_Auto, hdop, vdop, tdop);
    msg.Priority = 3;
    NMEA2000.SendMsg(msg);
}

// --- GNSS LOWRANCE ---
void sendN2kGNSS_Lowrance(double lat, double lon, uint16_t days, double seconds,
                          uint8_t sats, double hdop, double epe, uint8_t SID) {
    tN2kMsg msg;

    SetN2kPGN129029(msg, SID, days, seconds, lat, lon, 0.0,
                    N2kGNSSt_GPS,
                    (sats >= 4 ? N2kGNSSm_GNSSfix : N2kGNSSm_noGNSS),
                    sats, hdop, epe, 0.0,
                    1, N2kGNSSt_GPS, 0, 0.1);

    msg.Priority = 2;
    NMEA2000.SendMsg(msg);
}

// --- SATELLITES IN USE ---
void sendN2kSatellitesInUse() {
    float rawHdop = SatParser_getHDOP();
    if (rawHdop < 0.5f || rawHdop > 20.0f) return;

    double hdop = rawHdop;
    double vdop = hdop * 1.2;
    double tdop = hdop * 1.5;

    tN2kMsg msg;
    msg.SetPGN(129539);
    msg.Priority = 2;

    SetN2kPGN129539(msg, 0xFF, N2kGNSSdm_Auto, N2kGNSSdm_Auto, hdop, vdop, tdop);
    NMEA2000.SendMsg(msg);
}

// --- SATELLITES IN VIEW ---
void sendN2kSatellitesInView() {
    const SatInfo* sats = SatParser_getSatList();
    uint8_t count = SatParser_getSatInViewCount();
    if (count == 0) return;
    if (count > 16) count = 16;

    SatInfo list[16];
    for (int i = 0; i < count; i++) list[i] = sats[i];

    for (int i = 0; i < count - 1; i++)
        for (int j = i + 1; j < count; j++)
            if (list[j].prn < list[i].prn)
                std::swap(list[i], list[j]);

    tN2kMsg msg;
    SetN2kPGN129540(msg, 0xFF, N2kDD072_Unavailable);

    for (int i = 0; i < count; i++) {
        tSatelliteInfo sat;
        sat.PRN = list[i].prn;
        sat.Elevation = (list[i].elevation < 0 ? 0 : list[i].elevation);
sat.Azimuth   = (list[i].azimuth   < 0 ? 0 : list[i].azimuth);
sat.SNR       = (list[i].snr       < 0 ? 0 : list[i].snr);

        sat.RangeResiduals = N2kDoubleNA;

        AppendN2kPGN129540(msg, sat);
    }

    NMEA2000.SendMsg(msg);
}

// --- PROPRIETARY SAT INFO ---
void sendProprietarySatInfo(uint8_t SID, uint8_t prn, uint8_t elev, uint16_t azim,
                            uint8_t snr, bool used, float hdop, float epe, uint8_t satCount) {
    tN2kMsg msg;

    msg.SetPGN(61184);
    msg.Priority = 3;

    msg.AddByte(SID);
    msg.AddByte(satCount);
    msg.AddByte(prn);
    msg.AddByte(elev);
    msg.Add2ByteUInt(azim);
    msg.AddByte(snr);
    msg.AddByte(used ? 1 : 0);
    msg.AddFloat(hdop);
    msg.AddFloat(epe);

    NMEA2000.SendMsg(msg);
}
