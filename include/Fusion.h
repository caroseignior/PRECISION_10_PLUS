#pragma once
#include "IMU.h"

// Initialise le filtre Madgwick (fréquence définie dans fusionInit)
void fusionInit();

// Met à jour l'orientation fusionnée (pitch, roll, heading) en DEGRÉS
// à partir des données IMU remappées et calibrées
void fusionUpdate(const IMUData &d, Orientation &o);
