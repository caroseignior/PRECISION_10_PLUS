#include <NMEA2000.h>
#include <N2kMessages.h>
#include "N2kAttitude.h"

extern tNMEA2000 &NMEA2000;

void sendN2kAttitude(double roll_rad, double pitch_rad, uint8_t SID) {
    tN2kMsg N2kMsg;

    SetN2kAttitude(
        N2kMsg,
        SID,
        0.0,        // Yaw (non utilisé par Simrad/B&G)
        pitch_rad,  // Pitch
        roll_rad    // Roll
    );

    NMEA2000.SendMsg(N2kMsg);
}
