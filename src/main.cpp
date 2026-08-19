#include <Arduino.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <EEPROM.h>
#include <math.h>

#include "sat_parser.h"
#include "SystemConfig.h"
#include "IMU.h"
#include "Fusion.h"
#include "Pressure.h"
#include "DallasSensor.h"
#include "ConvertDMS.h"
#include "DateConv.h"

#include <NMEA2000_CAN.h>
#include <NMEA2000_esp32.h>
#include "N2kOut.h"
#include "N2kAttitude.h"
#include <N2kMessages.h>
#include "EnvManager.h"
#include "yamaha_bridge.h"

#include <BluetoothSerial.h>

#define EEPROM_SIZE 64
#define FW_VERSION "V2.1.0"

TinyGPSPlus gps;
Pressure pressure;
DallasSensor dallas(ONE_WIRE_PIN);

extern tNMEA2000 &NMEA2000;

BluetoothSerial SerialBT;

double latitude  = 0.0;
double longitude = 0.0;

float ofsetair   = 0.0f;
float ofsetmer   = 0.0f;
float ofsethdg   = 0.0f;
float manualHeadingOffset = 0.0f;

float magVariationDeg;

float ofsetroll  = 0.0f;
float ofsetpitch = 0.0f;

bool telemetryEnabled = true;
int  gpsBaud          = 4800;

struct OffsetsData {
    float ofsetair;
    float ofsetmer;
    float ofsethdg;
    float ofsetroll;
    float ofsetpitch;
    float magVariationDeg;     
    bool  telemetryEnabled;
    int   gpsBaud;
};

OffsetsData offsets;

void autoCalibratePitchRoll(float roll_deg_meas, float pitch_deg_meas) {
    ofsetroll  = -roll_deg_meas;
    ofsetpitch = -pitch_deg_meas;
}

void loadOffsets() {
    EEPROM.get(0, offsets);

    if (isnan(offsets.ofsetair))   offsets.ofsetair = 0;
    if (isnan(offsets.ofsetmer))   offsets.ofsetmer = 0;
    if (isnan(offsets.ofsethdg))   offsets.ofsethdg = 0;
    if (isnan(offsets.ofsetroll))  offsets.ofsetroll = 0;
    if (isnan(offsets.ofsetpitch)) offsets.ofsetpitch = 0;
    if (isnan(offsets.magVariationDeg)) offsets.magVariationDeg = 0.0f;


    if (offsets.telemetryEnabled != true &&
        offsets.telemetryEnabled != false)
        offsets.telemetryEnabled = true;

    if (offsets.gpsBaud < 4800 || offsets.gpsBaud > 115200)
        offsets.gpsBaud = 4800;

    ofsetair         = offsets.ofsetair;
    ofsetmer         = offsets.ofsetmer;
    ofsethdg         = offsets.ofsethdg;
    ofsetroll        = offsets.ofsetroll;
    ofsetpitch       = offsets.ofsetpitch;
    magVariationDeg = offsets.magVariationDeg;
    telemetryEnabled = offsets.telemetryEnabled;
    gpsBaud          = offsets.gpsBaud;
}

void saveOffsets() {
    offsets.ofsetair         = ofsetair;
    offsets.ofsetmer         = ofsetmer;
    offsets.ofsethdg         = ofsethdg;
    offsets.ofsetroll        = ofsetroll;
    offsets.ofsetpitch       = ofsetpitch;
    offsets.magVariationDeg = magVariationDeg;
    offsets.telemetryEnabled = telemetryEnabled;
    offsets.gpsBaud          = gpsBaud;

    EEPROM.put(0, offsets);
    EEPROM.commit();
}

unsigned long t_env  = 0;
unsigned long t_nav  = 0;
unsigned long t_time = 0;
unsigned long t_bt   = 0;

uint8_t SID = 0;

IMUData     imuData;
Orientation ori;

void setup() {
    Serial.begin(115200);
    delay(200);

    EEPROM.begin(EEPROM_SIZE);
    loadOffsets();

    Wire.begin(I2C_SDA, I2C_SCL);

    imuInit();
    fusionInit();

    Serial2.begin(gpsBaud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    pressure.begin();
    dallas.begin();
    EnvManager::begin();

    SatParser_begin();

    NMEA2000.SetProductInformation("12345678", 100, "PRECISION_10_PLUS", FW_VERSION, FW_VERSION);
    NMEA2000.SetDeviceInformation(42, 130, 25, 2046);

    NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
NMEA2000.EnableForward(false);

    NMEA2000.Open();

    SerialBT.begin("PRECISION_10_PLUS");

    yamahaBridgeInit();
}
void loop() {

    unsigned long now = millis();

    NMEA2000.ParseMessages();

    // ----------- IMU 9-DOF -----------  
    imuRead(imuData);

    // Fusion marine (pitch/roll/heading)
    fusionUpdate(imuData, ori);

    // Offsets appliqués sur la fusion
    float roll_deg   = ori.roll   + ofsetroll;
    float pitch_deg  = ori.pitch  + ofsetpitch;
    float heading_deg = ori.heading + ofsethdg + manualHeadingOffset;

    if (heading_deg < 0.0f)         heading_deg += 360.0f;
    else if (heading_deg >= 360.0f) heading_deg -= 360.0f;

    // ----------- HEADING FILTRÉ (stabilité mer formée) -----------  
    static float heading_filtered = 0.0f;
    static bool  heading_init     = false;

    const float alpha_hdg = 0.1f;  // 0.1 = stable, 0.2 = plus réactif

    if (!heading_init) {
        heading_filtered = heading_deg;
        heading_init     = true;
    } else {
        float diff = heading_deg - heading_filtered;

        // gestion du wrap 0–360
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;

        heading_filtered += alpha_hdg * diff;

        if (heading_filtered < 0.0f)         heading_filtered += 360.0f;
        else if (heading_filtered >= 360.0f) heading_filtered -= 360.0f;
    }

    // ----------- ENVIRONNEMENT -----------  
    float t_air, p_air, h_air;
    EnvManager::read(t_air, p_air, h_air);
    t_air += ofsetair;

    dallas.update();
    float waterTemp = dallas.getTempC() + ofsetmer;

    // ----------- GPS RAW -----------  
    while (Serial2.available()) {
        char c = Serial2.read();
        gps.encode(c);
        SatParser_encodeChar(c);
    }

    double lat = gps.location.isValid() ? gps.location.lat() : 0;
    double lon = gps.location.isValid() ? gps.location.lng() : 0;
    double sog = gps.speed.isValid()    ? gps.speed.knots() : 0;
    double cog = gps.course.isValid()   ? gps.course.deg()  : 0;

    static double cog_filtered = 0.0;
static double sog_filtered = 0.0;
static bool   nav_init     = false;

const double sog_low   = 1.0;   // < 1 kn : fort filtrage
const double sog_high  = 3.0;   // > 3 kn : filtrage léger
const double alpha_cog_slow = 0.05;
const double alpha_cog_fast = 0.3;
const double alpha_sog_slow = 0.05;
const double alpha_sog_fast = 0.3;

if (!nav_init) {
    cog_filtered = cog;
    sog_filtered = sog;
    nav_init     = true;
} else {
    double alpha_cog = (sog < sog_low) ? alpha_cog_slow : alpha_cog_fast;
    double alpha_sog = (sog < sog_low) ? alpha_sog_slow : alpha_sog_fast;

    // gestion wrap COG 0–360
    double diff_cog = cog - cog_filtered;
    if (diff_cog > 180.0)  diff_cog -= 360.0;
    if (diff_cog < -180.0) diff_cog += 360.0;

    cog_filtered += alpha_cog * diff_cog;
    sog_filtered += alpha_sog * (sog - sog_filtered);
}


    latitude  = lat;
    longitude = lon;

    double hdop = SatParser_getHDOP();

    yamahaBridgeUpdate();

    // ----------- GPS FILTRÉ + MODE MOUILLAGE -----------  
    static double lat_filtered = 0.0;
    static double lon_filtered = 0.0;
    static bool gps_init = false;

    const double sog_threshold = 0.54;   // < 1 km/h
    const double sog_anchor    = 0.3;    // mouillage
    const double alpha_slow    = 0.02;   // stable
    const double alpha_fast    = 0.5;    // réactif

    bool anchor_mode = (sog < sog_anchor);

    if (!gps_init) {
        lat_filtered = lat;
        lon_filtered = lon;
        gps_init     = true;
    } else {
        double alpha = (sog < sog_threshold) ? alpha_slow : alpha_fast;
        if (anchor_mode) alpha = 0.01;   // ultra stable au mouillage

        lat_filtered = lat_filtered + alpha * (lat - lat_filtered);
        lon_filtered = lon_filtered + alpha * (lon - lon_filtered);
    }

    // ----------- RATE OF TURN (PGN 127251) -----------  
    static float last_heading = 0.0f;
    static uint32_t last_rot_ms = 0;

    float rot_deg_per_s = 0.0f;

    uint32_t now_rot = millis();
    if (last_rot_ms != 0) {
        float dt = (now_rot - last_rot_ms) / 1000.0f;

        if (dt > 0.0f) {
            float diff = heading_filtered - last_heading;

            if (diff > 180.0f)  diff -= 360.0f;
            if (diff < -180.0f) diff += 360.0f;

            rot_deg_per_s = diff / dt;
        }
    }

    last_heading = heading_filtered;
    last_rot_ms  = now_rot;

    double rot_rad_per_s = rot_deg_per_s * M_PI / 180.0;

        // ----------- NMEA2000 NAVIGATION -----------  
    if (now - t_nav > 300) {
        SID++;

        if (gps.location.isValid()) {
            sendN2kPosition(lat_filtered, lon_filtered, SID);
            sendN2kCOGSOG(cog_filtered, sog_filtered, SID);
        }

        double heading_rad = heading_filtered * M_PI / 180.0;
        double roll_rad    = roll_deg   * M_PI / 180.0;
        double pitch_rad   = pitch_deg  * M_PI / 180.0;
        double rot_rad_per_s = rot_deg_per_s * M_PI / 180.0;
        double variation_rad = magVariationDeg * M_PI / 180.0;

        sendN2kHeading(heading_rad, SID);
        sendN2kAttitude(roll_rad, pitch_rad, SID);
        sendN2kRateOfTurn(rot_rad_per_s, SID);
        sendN2kMagneticVariation(variation_rad, SID);

        sendN2kSatellitesInUse();
        sendN2kSatellitesInView();

        t_nav = now;
    }

    // ----------- ENVIRONNEMENT (1500 ms) -----------  
    if (now - t_env > 1500) {

        if (isnan(t_air)) t_air = 20;
        if (isnan(p_air)) p_air = 101325;
        if (isnan(h_air)) h_air = 0;

        sendN2kAirTemperature(t_air, SID);
        sendN2kHumidity(h_air / 100.0, SID);
        sendN2kPressure(p_air, SID);
        sendN2kWaterTemperature(waterTemp, SID);

        t_env = now;
    }

    // ----------- DATE / HEURE (2000 ms) -----------  
    if (gps.date.isValid() && gps.time.isValid() && now - t_time > 2000) {

        uint16_t daysSince1970 = convertDateToDaysSince1970(
            gps.date.year(),
            gps.date.month(),
            gps.date.day()
        );

        double secondsSinceMidnight =
            gps.time.hour()   * 3600.0 +
            gps.time.minute() * 60.0  +
            gps.time.second();

        sendN2kDateTime(daysSince1970, secondsSinceMidnight, SID);
        t_time = now;
    }

    // ----------- BLUETOOTH TÉLÉMÉTRIE (1 Hz) -----------  
    if (SerialBT.hasClient() && telemetryEnabled && now - t_bt > 1000) {
        t_bt = now;

        SerialBT.print("{\"hdg\":");   SerialBT.print(heading_filtered, 1);
        SerialBT.print(",\"roll\":");  SerialBT.print(roll_deg, 1);
        SerialBT.print(",\"pitch\":"); SerialBT.print(pitch_deg, 1);
        SerialBT.print(",\"tair\":");  SerialBT.print(t_air, 1);
        SerialBT.print(",\"pair\":");  SerialBT.print(p_air, 1);
        SerialBT.print(",\"teau\":");  SerialBT.print(waterTemp, 1);
        SerialBT.print(",\"lat\":");   SerialBT.print(lat_filtered, 6);
        SerialBT.print(",\"lon\":");   SerialBT.print(lon_filtered, 6);
        SerialBT.print(",\"cog\":");   SerialBT.print(cog_filtered, 1);
        SerialBT.print(",\"sog\":");   SerialBT.print(sog_filtered, 1);

        SerialBT.println("}");
    }

        // ----------- COMMANDES BLUETOOTH -----------  
    if (SerialBT.available()) {

        String cmd = SerialBT.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();

        // ----------- HELP -----------  
        if (cmd == "HELP") {
            SerialBT.println("{");
            SerialBT.println("  \"SETVARIATION=\": \"OK SET MAGNETIC VARIATION\",");
            SerialBT.println("  \"CALPR\": \"Auto-calibration Pitch/Roll\",");
            SerialBT.println("  \"RESETPR\": \"Reset Pitch/Roll offsets\",");
            SerialBT.println("  \"CALHDG\": \"Auto-calibration Heading\",");
            SerialBT.println("  \"SETAIR=x\": \"Set Air Temperature offset (°C)\",");
            SerialBT.println("  \"SETSEA=x\": \"Set Sea Temperature offset (°C)\",");
            SerialBT.println("  \"SETHEADING=x\": \"Manual heading offset (°)\",");
            SerialBT.println("  \"GETALL\": \"Return main values\",");
            SerialBT.println("  \"RESETALL\": \"Reset ALL offsets\",");
            SerialBT.println("  \"VERSION\": \"Firmware version\",");
            SerialBT.println("  \"GPSPOS\": \"Return GPS Lat/Lon\",");
            SerialBT.println("  \"MAGXYZ\": \"Return magnetometer XYZ\"");
            SerialBT.println("}");
        }

        //............ SET MAGNETIC VARIATION --------
        if (cmd.startsWith("SETVARIATION=")) {
    magVariationDeg = cmd.substring(12).toFloat();
    saveOffsets();   // si tu veux la rendre persistante, ajoute magVariationDeg dans OffsetsData
    SerialBT.println("OK SET MAGNETIC VARIATION");
}

if (cmd == "GETNAV") {
    SerialBT.println("{");
    SerialBT.print("  \"hdg\": ");        SerialBT.println(heading_filtered, 2);
    SerialBT.print("  \"cog\": ");        SerialBT.println(cog_filtered, 2);
    SerialBT.print("  \"sog\": ");        SerialBT.println(sog_filtered, 2);
    SerialBT.print("  \"lat\": ");        SerialBT.println(lat_filtered, 6);
    SerialBT.print("  \"lon\": ");        SerialBT.println(lon_filtered, 6);
    SerialBT.print("  \"variation\": ");  SerialBT.println(magVariationDeg, 2);
    SerialBT.println("}");
}



        // ----------- CALIBRATION PITCH/ROLL -----------  
        if (cmd == "CALPR") {
            autoCalibratePitchRoll(roll_deg, pitch_deg);
            saveOffsets();
            SerialBT.println("OK AUTO CALIBRATION PITCH/ROLL");
        }

        // ----------- RESET PITCH/ROLL -----------  
        if (cmd == "RESETPR") {
            ofsetroll  = 0.0f;
            ofsetpitch = 0.0f;
            saveOffsets();
            SerialBT.println("OK RESET PITCH/ROLL");
        }

        // ----------- CALIBRATION HEADING -----------  
        if (cmd == "CALHDG") {
            float newOffset = -heading_deg;
            if (newOffset < 0.0f) newOffset += 360.0f;
            ofsethdg = newOffset;
            manualHeadingOffset = 0.0f;
            saveOffsets();
            SerialBT.println("OK AUTO CALIBRATION HDG");
        }

        // ----------- SET AIR TEMP OFFSET -----------  
        if (cmd.startsWith("SETAIR=")) {
            ofsetair = cmd.substring(7).toFloat();
            saveOffsets();
            SerialBT.println("OK SET AIR OFFSET");
        }

        // ----------- SET SEA TEMP OFFSET -----------  
        if (cmd.startsWith("SETSEA=")) {
            ofsetmer = cmd.substring(7).toFloat();
            saveOffsets();
            SerialBT.println("OK SET SEA OFFSET");
        }

        // ----------- SET MANUAL HEADING OFFSET -----------  
        if (cmd.startsWith("SETHEADING=")) {
            manualHeadingOffset = cmd.substring(11).toFloat();
            saveOffsets();
            SerialBT.println("OK SET HEADING OFFSET");
        }

        // ----------- GETALL -----------  
        if (cmd == "GETALL") {
            SerialBT.println("{");
            SerialBT.print("  \"hdg\": ");   SerialBT.println(heading_filtered, 2);
            SerialBT.print("  \"pitch\": "); SerialBT.println(pitch_deg, 2);
            SerialBT.print("  \"roll\": ");  SerialBT.println(roll_deg, 2);
            SerialBT.print("  \"tair\": ");  SerialBT.println(t_air, 2);
            SerialBT.print("  \"tsea\": ");  SerialBT.println(waterTemp, 2);
            SerialBT.println("}");
        }

        // ----------- RESET ALL OFFSETS -----------  
        if (cmd == "RESETALL") {
            ofsetpitch = 0.0f;
            ofsetroll  = 0.0f;
            ofsethdg   = 0.0f;
            ofsetair   = 0.0f;
            ofsetmer   = 0.0f;
            saveOffsets();
            SerialBT.println("OK RESET ALL OFFSETS");
        }

        // ----------- VERSION -----------  
        if (cmd == "VERSION") {
            SerialBT.print("PRECISION_10_PLUS ");
            SerialBT.println(FW_VERSION);
        }

        // ----------- GPS POSITION -----------  
        if (cmd == "GPSPOS") {
            SerialBT.println("{");
            SerialBT.print("  \"lat\": "); SerialBT.println(lat_filtered, 6);
            SerialBT.print("  \"lon\": "); SerialBT.println(lon_filtered, 6);
            SerialBT.println("}");
        }

        // ----------- MAGNETOMETER XYZ -----------  
        if (cmd == "MAGXYZ") {
            SerialBT.println("{");
            SerialBT.print("  \"mx\": "); SerialBT.println(imuData.mx);
            SerialBT.print("  \"my\": "); SerialBT.println(imuData.my);
            SerialBT.print("  \"mz\": "); SerialBT.println(imuData.mz);
            SerialBT.println("}");
        }
    }
}

