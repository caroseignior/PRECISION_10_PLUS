#include "yamaha_bridge.h"
#include <NMEA2000.h>
#include <N2kMessages.h>

#include "NMEA2000_module.h"   // fuelRate_Lh, fuelUsed_L, seaTemp_C

extern tNMEA2000 &NMEA2000;

// ------------------------------------------------------
// Envoi brut CAN via NMEA2000_esp32
// ------------------------------------------------------
static void sendYamahaRaw(uint32_t id, const uint8_t *d) {
    tN2kMsg msg;
    msg.SetPGN(id);
    msg.Priority = 3;

    for (int i = 0; i < 8; i++) msg.AddByte(d[i]);

    NMEA2000.SendMsg(msg);
}

// ------------------------------------------------------
// Fuel Rate → ID 0x650 (mL/min)
// ------------------------------------------------------
static void sendFuelRate() {
    uint8_t d[8] = {0};

    uint16_t ml_min = (uint16_t)((fuelRate_Lh * 1000.0) / 60.0);

    d[0] = ml_min & 0xFF;
    d[1] = ml_min >> 8;

    sendYamahaRaw(0x650, d);
}

// ------------------------------------------------------
// Fuel Used → ID 0x651 (0.1 L)
// ------------------------------------------------------
static void sendFuelUsed() {
    uint8_t d[8] = {0};

    uint16_t fu = (uint16_t)(fuelUsed_L * 10.0);

    d[0] = fu & 0xFF;
    d[1] = fu >> 8;

    sendYamahaRaw(0x651, d);
}

// ------------------------------------------------------
// Water Temp → ID 0x652 (0.1 °C)
// ------------------------------------------------------
static void sendWaterTemp() {
    uint8_t d[8] = {0};

    int16_t t = (int16_t)(seaTemp_C * 10.0);

    d[0] = t & 0xFF;
    d[1] = t >> 8;

    sendYamahaRaw(0x652, d);
}

void yamahaBridgeInit() {
    // Rien à faire : NMEA2000_esp32 gère déjà le CAN
}

void yamahaBridgeUpdate() {
    static unsigned long last = 0;
    if (millis() - last < 200) return;
    last = millis();

    if (hasFuelRate) sendFuelRate();
    if (hasFuelUsed) sendFuelUsed();
    if (hasTempSea)  sendWaterTemp();

    // IMPORTANT : on n’envoie PAS speed ni depth
}
