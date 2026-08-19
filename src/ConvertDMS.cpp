#include "ConvertDMS.h"
#include <math.h>

String convertLatToDMS(double lat) {
    char hemi = (lat >= 0) ? 'N' : 'S';
    lat = fabs(lat);

    int deg = (int)lat;
    double minFull = (lat - deg) * 60.0;
    int min = (int)minFull;
    int sec = (int)((minFull - min) * 60.0);

    char buf[24];
    sprintf(buf, "%02d°%02d'%02d\"%c", deg, min, sec, hemi);
    return String(buf);
}

String convertLonToDMS(double lon) {
    char hemi = (lon >= 0) ? 'E' : 'W';
    lon = fabs(lon);

    int deg = (int)lon;
    double minFull = (lon - deg) * 60.0;
    int min = (int)minFull;
    int sec = (int)((minFull - min) * 60.0);

    char buf[24];
    sprintf(buf, "%03d°%02d'%02d\"%c", deg, min, sec, hemi);
    return String(buf);
}
