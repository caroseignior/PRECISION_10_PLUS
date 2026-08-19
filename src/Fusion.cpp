#include "Fusion.h"
#include <MadgwickAHRS.h>

Madgwick filter;

void fusionInit() {
    // Fréquence de mise à jour IMU (100 Hz)
    filter.begin(100);
}

void fusionUpdate(const IMUData &d, Orientation &o) {

    // ---------------------------------------------------------
    // Madgwick 9-DOF avec axes déjà remappés dans imu.cpp :
    // X = avant, Y = tribord, Z = bas
    // ---------------------------------------------------------
    filter.update(
        d.gx, d.gy, d.gz,   // gyro remappé
        d.ax, d.ay, d.az,   // accel remappé
        d.mx, d.my, d.mz    // mag calibré + remappé
    );

    // ---------------------------------------------------------
    // Madgwick renvoie les angles en DEGRÉS
    // ---------------------------------------------------------
    o.pitch   = filter.getPitch();   // tangage fusionné
    o.roll    = filter.getRoll();    // roulis fusionné
    float yaw = filter.getYaw();
yaw = -yaw;                      // inversion du sens
if (yaw < 0.0f)      yaw += 360.0f;
else if (yaw >= 360.0f) yaw -= 360.0f;

o.heading = yaw;

}
