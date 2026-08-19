#include "sat_parser.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

// ======================================================
//   CONSTANTES INTERNES
// ======================================================
static const float SATPARSER_NA = -1.0f;

// ======================================================
//   ÉTAT INTERNE
// ======================================================

static char   s_sentenceBuf[128];
static size_t s_sentenceLen = 0;
static bool   s_inSentence  = false;

static SatInfo s_sats[SATPARSER_MAX_SATS];
static uint8_t s_satCount       = 0;
static uint8_t s_satsInView     = 0;
static uint8_t s_satsUsedInFix  = 0;

static float   s_hdop = SATPARSER_NA;

// ======================================================
//   PROTOTYPES INTERNES
// ======================================================
static void SatParser_handleSentence(const char *sentence);
static void handleGSA(char *sentence);
static void handleGSV(char *sentence);
static bool startsWith(const char *s, const char *p);
static bool validateChecksum(const char *sentence);
static int  splitCSV(char *line, char *fields[], int maxFields);
static SatInfo* findOrAllocSat(uint8_t prn);

// ======================================================
//   API PUBLIQUE
// ======================================================
void SatParser_begin() {
  SatParser_reset();
}

void SatParser_reset() {
  s_sentenceLen   = 0;
  s_inSentence    = false;
  s_satCount      = 0;
  s_satsInView    = 0;
  s_satsUsedInFix = 0;
  s_hdop          = SATPARSER_NA;

  for (uint8_t i = 0; i < SATPARSER_MAX_SATS; i++) {
    s_sats[i].prn        = 0;
    s_sats[i].elevation  = -1;
    s_sats[i].azimuth    = -1;
    s_sats[i].snr        = 0;
    s_sats[i].usedInFix  = false;
  }
}

void SatParser_encodeChar(char c) {
  if (c == '$') {
    s_inSentence  = true;
    s_sentenceLen = 0;
    s_sentenceBuf[s_sentenceLen++] = c;
    s_sentenceBuf[s_sentenceLen]   = '\0';
    return;
  }

  if (!s_inSentence) return;

  if (c == '\r' || c == '\n') {
    if (s_sentenceLen > 6) {
      s_sentenceBuf[s_sentenceLen] = '\0';
      SatParser_handleSentence(s_sentenceBuf);
    }
    s_inSentence  = false;
    s_sentenceLen = 0;
    return;
  }

  if (s_sentenceLen + 1 < sizeof(s_sentenceBuf)) {
    s_sentenceBuf[s_sentenceLen++] = c;
    s_sentenceBuf[s_sentenceLen]   = '\0';
  } else {
    s_inSentence  = false;
    s_sentenceLen = 0;
  }
}

// Accès aux données
uint8_t SatParser_getSatInViewCount() { return s_satsInView; }
uint8_t SatParser_getSatInUseCount()  { return s_satsUsedInFix; }
const SatInfo* SatParser_getSatList() { return s_sats; }
uint8_t SatParser_getSatListSize()    { return SATPARSER_MAX_SATS; }
float SatParser_getHDOP()             { return s_hdop; }

// ======================================================
//   ROUTAGE
// ======================================================
static void SatParser_handleSentence(const char *sentence) {
  char buf[128];
  size_t len = strlen(sentence);
  if (len >= sizeof(buf)) len = sizeof(buf) - 1;
  memcpy(buf, sentence, len);
  buf[len] = '\0';

  if (startsWith(buf, "$GPGSA") ||
      startsWith(buf, "$GNGSA") ||
      startsWith(buf, "$BDGSA") ||
      startsWith(buf, "$GLGSA")) {
    handleGSA(buf);
    return;
  }

  if (startsWith(buf, "$GPGSV") ||
      startsWith(buf, "$GNGSV") ||
      startsWith(buf, "$BDGSV") ||
      startsWith(buf, "$GLGSV")) {
    handleGSV(buf);
    return;
  }
}

// ======================================================
//   UTILITAIRES
// ======================================================
static bool startsWith(const char *s, const char *p) {
  return strncmp(s, p, strlen(p)) == 0;
}

static bool validateChecksum(const char *sentence) {
  const char *star = strchr(sentence, '*');
  if (!star) return false;

  uint8_t cs = 0;
  const char *p = sentence + 1;
  while (p < star) cs ^= (uint8_t)(*p++);

  uint8_t expected = (uint8_t)strtoul(star + 1, nullptr, 16);
  return (cs == expected);
}

static int splitCSV(char *line, char *fields[], int maxFields) {
  int count = 0;
  char *p = line;
  while (count < maxFields && p && *p) {
    fields[count++] = p;
    char *comma = strchr(p, ',');
    if (!comma) break;
    *comma = '\0';
    p = comma + 1;
  }
  return count;
}

static SatInfo* findOrAllocSat(uint8_t prn) {
  if (prn == 0) return nullptr;

  for (uint8_t i = 0; i < s_satCount; i++) {
    if (s_sats[i].prn == prn) return &s_sats[i];
  }

  if (s_satCount >= SATPARSER_MAX_SATS) return nullptr;

  SatInfo &sat = s_sats[s_satCount++];
  sat.prn       = prn;
  sat.elevation = -1;
  sat.azimuth   = -1;
  sat.snr       = 0;
  sat.usedInFix = false;
  return &sat;
}

// ======================================================
//   GSA : HDOP + satellites utilisés
// ======================================================
static void handleGSA(char *sentence) {
  if (!validateChecksum(sentence)) return;

  char *star = strchr(sentence, '*');
  if (star) *star = '\0';

  char *fields[32];
  int n = splitCSV(sentence, fields, 32);
  if (n < 18) return;

  if (fields[2][0] == '1') return;

  float hdop = atof(fields[16]);
  if (hdop < 0.5f || hdop > 10.0f) return;

  s_hdop = hdop;

  for (uint8_t i = 0; i < s_satCount; i++) {
    s_sats[i].usedInFix = false;
  }

  s_satsUsedInFix = 0;
  for (int i = 3; i <= 14; i++) {
    if (fields[i] && fields[i][0] != '\0') {
      uint8_t prn = (uint8_t)atoi(fields[i]);
      SatInfo *sat = findOrAllocSat(prn);
      if (sat) {
        sat->usedInFix = true;
        s_satsUsedInFix++;
      }
    }
  }
}

// ======================================================
//   GSV : sats en vue + SNR
// ======================================================
static void handleGSV(char *sentence) {
  if (!validateChecksum(sentence)) return;

  char *star = strchr(sentence, '*');
  if (star) *star = '\0';

  char *fields[32];
  int n = splitCSV(sentence, fields, 32);
  if (n < 4) return;

  int totalSats = atoi(fields[3]);
  if (totalSats < 0) totalSats = 0;
  if (totalSats > SATPARSER_MAX_SATS) totalSats = SATPARSER_MAX_SATS;
  s_satsInView = (uint8_t)totalSats;

  for (int i = 4; i + 3 < n; i += 4) {
    if (fields[i][0] == '\0') continue;

    uint8_t prn = (uint8_t)atoi(fields[i]);
    SatInfo *sat = findOrAllocSat(prn);
    if (!sat) continue;

    int elev = (fields[i+1][0] != '\0') ? atoi(fields[i+1]) : -1;
    int az   = (fields[i+2][0] != '\0') ? atoi(fields[i+2]) : -1;
    int snr  = (fields[i+3][0] != '\0') ? atoi(fields[i+3]) : 0;

    sat->elevation = elev;
    sat->azimuth   = az;
    sat->snr       = snr;
  }
}
