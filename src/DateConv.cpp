#include "DateConv.h"

static bool isLeap(int y) {
    return ((y % 4 == 0) && (y % 100 != 0)) || (y % 400 == 0);
}

uint16_t convertDateToDaysSince1970(int year, int month, int day) {
    uint16_t days = 0;

    // Années complètes depuis 1970
    for (int y = 1970; y < year; y++) {
        days += isLeap(y) ? 366 : 365;
    }

    // Mois complets de l'année en cours
    static const int ml[12] = {31,28,31,30,31,30,31,31,30,31,30,31};

    for (int m = 1; m < month; m++) {
        days += ml[m-1];
        if (m == 2 && isLeap(year)) days++;
    }

    // Jours du mois
    days += (day - 1);

    return days;
}
