#include "IMU.h"
#include <SparkFunLSM6DS3.h>
#include <Adafruit_LIS3MDL.h>
#include <math.h>

LSM6DS3 imu(I2C_MODE, 0x6A);
Adafruit_LIS3MDL mag;

// Hard-iron offsets
static const float ox = 37.0f;
static const float oy = -47.0f;
static const float oz = -0.5f;

// Soft-iron correction matrix
static const float ma = 1.00f, mb = 0.0f,  mc = 0.0f;
static const float md = 0.0f, me = 0.92f, mf = 0.0f;
static const float mg = 0.0f, mh = 0.0f,  mi = 1.08f;

void imuInit() {
    imu.begin();

    mag.begin_I2C();
    mag.setPerformanceMode(LIS3MDL_ULTRAHIGHMODE);
    mag.setOperationMode(LIS3MDL_CONTINUOUSMODE);
    mag.setDataRate(LIS3MDL_DATARATE_155_HZ);
    mag.setRange(LIS3MDL_RANGE_4_GAUSS);
}

void imuRead(IMUData &d) {

    // ---------------------------------------------------------
    // 1) ACCEL brut module (X avant, Y bâbord, Z haut)
    // ---------------------------------------------------------
    float ax_raw = imu.readFloatAccelX();
    float ay_raw = imu.readFloatAccelY();
    float az_raw = imu.readFloatAccelZ();

    // Remap vers repère marin (X avant, Y tribord, Z bas)
    d.ax = ax_raw;
    d.ay = -ay_raw;
    d.az = -az_raw;

    // ---------------------------------------------------------
    // 2) GYRO brut module (même remap)
    // ---------------------------------------------------------
    float gx_raw = imu.readFloatGyroX();
    float gy_raw = imu.readFloatGyroY();
    float gz_raw = imu.readFloatGyroZ();

    d.gx = gx_raw;
    d.gy = -gy_raw;
    d.gz = -gz_raw;

    // ---------------------------------------------------------
    // 3) MAG brut module (X avant, Y bâbord, Z haut)
    // ---------------------------------------------------------
    sensors_event_t event;
    mag.getEvent(&event);

    float mx_raw = event.magnetic.x;
    float my_raw = event.magnetic.y;
    float mz_raw = event.magnetic.z;

    // Remap vers repère marin
    float mx_r = mx_raw;
    float my_r = -my_raw;
    float mz_r = mz_raw;

    // Hard-iron
    float mx_hi = mx_r - ox;
    float my_hi = my_r - oy;
    float mz_hi = mz_r - oz;

    // Soft-iron
    float mx_si = ma*mx_hi + mb*my_hi + mc*mz_hi;
    float my_si = md*mx_hi + me*my_hi + mf*mz_hi;
    float mz_si = mg*mx_hi + mh*my_hi + mi*mz_hi;

    // Normalisation
    float norm = sqrt(mx_si*mx_si + my_si*my_si + mz_si*mz_si);
    if (norm > 0.0f) {
        mx_si /= norm;
        my_si /= norm;
        mz_si /= norm;
    }

    d.mx = mx_si;
    d.my = my_si;
    d.mz = mz_si;

    // ---------------------------------------------------------
    // 4) Pitch / Roll IMU brut (radians)
    // ---------------------------------------------------------
    float roll  = atan2(d.ay, d.az);
    float pitch = atan2(-d.ax, sqrt(d.ay*d.ay + d.az*d.az));

    d.roll  = roll;
    d.pitch = pitch;

    // ---------------------------------------------------------
    // 5) Heading magnétomètre (degrés 0–360)
    // ---------------------------------------------------------
    float hdg = atan2(d.my, d.mx) * 180.0f / M_PI;
    if (hdg < 0) hdg += 360.0f;

    d.heading = hdg;
}

// ---------------------------------------------------------
// Lecture brute magnétomètre (sans remap, pour calibration)
// ---------------------------------------------------------
void imuReadRawMag(float &mx, float &my, float &mz) {
    sensors_event_t event;
    mag.getEvent(&event);

    mx = event.magnetic.x;
    my = event.magnetic.y;
    mz = event.magnetic.z;
}
