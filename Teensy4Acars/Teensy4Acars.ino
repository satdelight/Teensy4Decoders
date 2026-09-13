// ============================================================================
//  Teensy4Acars.ino — ACARS 2400-baud MSK decoder for Teensy 4.x
//  ----------------------------------------------------------------------------
//  Audio chain : AudioInputUSB (44.1 kHz) -> MSK demodulator -> CRC check,
//  decoded frames are printed on the (USB) Serial console.
//
//  Based on ACARSDEC
//    Original repository: https://github.com/TLeconte/acarsdec
//    Copyright (C) 2007-2025 Thierry Leconte
//    Modified / extended by Thibaut VARENE (f00b4r0)
//      Copyright (C) 2024-2026 Thibaut VARENE
//      https://github.com/f00b4r0/acarsdec/
//  Ported from msk.c / acars.c / syndrom.h to the Teensy 4.x Audio library,
//  INTRATE = 44100 (AudioInputUSB).
//  Port: Copyright (C) 2025-2026 Holger Nyga (satdelight)
//        https://github.com/satdelight
//
//  SPDX-License-Identifier: GPL-2.0-only
//
//  This program is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License version 2, as
//  published by the Free Software Foundation.
//
//  This program is distributed in the hope that it will be useful, but
//  WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
// ============================================================================
#include <Audio.h>
#include <math.h>
#include "crc.h"

AudioInputUSB            usbAudio;
AudioRecordQueue         audioQueue;
AudioConnection          patch(usbAudio, 0, audioQueue, 0);

const int LED_PIN = 13;
unsigned long ledTurnOffTime = 0;
const unsigned long LED_DURATION = 500;

// Show frames with CRC errors: toggle via 'c' (default: off)
bool showCrcBad = false;
bool bannerShown = false;

// Raw sample dump (int16, full rate): toggle via 'd', for offline analysis only
bool dumpMode = false;

// ------------------------------------------------------------------
//  ACARS 2400 baud MSK demodulator based on acarsdec
//  Ported from msk.c / acars.c, INTRATE = 44100 (AudioInputUSB)
// ------------------------------------------------------------------
#define INTRATE 44100
#define PLLG    (38e-4f)
#define PLLC    (0.52f)
#define FLEN    ((INTRATE / 1200) + 1)   // 37
#define MFLTOVER 12
#define FLENO   (FLEN * MFLTOVER + 1)    // 445

#define SYN 0x16
#define SOH 0x01
#define ETX 0x83
#define ETB 0x97

static float h[FLENO];

typedef struct {
  double MskPhi;
  double MskDf;
  float  MskClk;
  double MskLvlSum;
  int    MskBitCount;
  unsigned int MskS;
  unsigned int idx;
  float inbre[FLEN];
  float inbim[FLEN];

  unsigned char outbits;
  int nbits;

  int state;   // WSYN SYN2 SOH1 TXT CRC1 CRC2 END
  unsigned char txt[240];
  int len;
  unsigned char crc0, crc1;
  int ok;               // accepted frames
  int dropPerr;         // dropped: >3 parity errors
  int dropCrc;          // dropped: CRC error (when option off)
  int nSync;            // SYN pair found (frame candidate)
  int nSOH;             // SOH hit (TXT started)
  int nEtx;             // ETX/ETB reached (frameDone)
  long nBits;           // total demodulated bits
  int perr;             // parity bit errors in current frame
  int hist[8];          // perr histogram: 0,1,2,3,4,5..8,9..20,>20
  unsigned short crc;   // running CRC-16
} ch_t;

enum { WSYN, SYN2, SOH1, TXT, CRC1, CRC2, END };

static ch_t ch;

static int oddParity(unsigned char b) {
  b = b ^ (b >> 4);
  b = b ^ (b >> 2);
  b = b ^ (b >> 1);
  return b & 1;
}

static void resetAcars(void) {
  ch.state = WSYN;
  ch.MskDf = 0.0;
  ch.nbits = 1;
}

static void printBody(int from) {
  for (int i = from; i < ch.len; i++) {
    char c = ch.txt[i] & 0x7F;
    if (c >= 32 && c < 127) Serial.print(c);
    else if (c == 0x0D || c == 0x0A) Serial.print(' ');
    else if (c == 0x01) Serial.print("<SOH>");
    else if (c == 0x02) Serial.print("<STX>");
    else if (c == 0x03) Serial.print("<ETX>");
    else if (c == 0x17) Serial.print("<ETB>");
  }
}

static void frameDone(void) {
  digitalWrite(LED_PIN, HIGH);
  ledTurnOffTime = millis() + LED_DURATION;

  int b = ch.perr;
  int h = (b > 5) ? ((b > 8) ? ((b > 20) ? 7 : 6) : 5) : b;
  ch.hist[h]++;

  if (ch.perr > 3) { ch.dropPerr++; return; }
  if (ch.crc != 0 && !showCrcBad) { ch.dropCrc++; return; }
  ch.ok++;

  if (dumpMode) return;

  // Output: 1:1 port of acarsdec fmt_msg() (FMT_FULL), output.c,
  //   [#1 (L:+5.1/0.0 E:0) --------------------------------
  //   Mode : 2 Label : _d Id : 3 Nak
  //   Aircraft reg: A6-EFM Flight id: EK9658
  //   No: S33A
  //   <text>
  // Differences because the Teensy has no clock: the date is not
  // printed (acarsdec behaves the same for clockless sources.
  // Ack handling like acarsdec: "Nak" is only shown when the byte is
  // NAK (0x15). Any other value is not a defined protocol status, so
  // nothing is printed for it (acarsdec would print the raw byte).

  float lvl = 0.0f;
  if (ch.MskBitCount > 0)
    lvl = 10.0f * log10f((float)(ch.MskLvlSum / ch.MskBitCount) + 1e-8f);

  char bid = (ch.len > 11) ? (char)(ch.txt[11] & 0x7F) : 0;
  bool downlink = bid >= '0' && bid <= '9';

  // [#1 (L:+5.1/0.0 E:0)  --------------------------------
  Serial.print("\n[#1 (L:");
  char lvlStr[16];
  snprintf(lvlStr, sizeof(lvlStr), "%+5.1f/%.1f", lvl, 0.0f);
  Serial.print(lvlStr);
  Serial.print(" E:");
  Serial.print(ch.perr);
  Serial.println(") --------------------------------");

  // Mode : 2  Label : _d  Id : 3    (label[1]=0x7f -> 'd', like outputmsg())
  char label0 = ch.txt[9] & 0x7F;
  char label1 = ch.txt[10] & 0x7F;
  if (label1 == 0x7F) label1 = 'd';
  Serial.print("Mode : ");
  Serial.print((char)(ch.txt[0] & 0x7F));
  Serial.print(" Label : ");
  Serial.print(label0);
  Serial.print(label1);

  if (bid != 0) {
    Serial.print(" Id : ");
    Serial.print(bid);
    char ack = ch.txt[8] & 0x7F;
    if (ack == 0x15)
      Serial.print(" Nak");
    Serial.println();
    Serial.print("Aircraft reg: ");
    int aStart = 1;                   // skip leading dots, like outputmsg()
    while (aStart <= 7 && (ch.txt[aStart] & 0x7F) == '.') aStart++;
    for (int i = aStart; i <= 7; i++) Serial.print((char)(ch.txt[i] & 0x7F));
    if (downlink) {
      if (ch.len >= 23) {
        Serial.print(" Flight id: ");
        for (int i = 17; i <= 22; i++) Serial.print((char)(ch.txt[i] & 0x7F));
      }
      Serial.println();
      if (ch.len >= 17) {
        Serial.print("No: ");
        for (int i = 13; i <= 16; i++) Serial.print((char)(ch.txt[i] & 0x7F));
        Serial.println();
      }
    } else {
      Serial.println();
    }
  } else {
    Serial.println();
  }

  if (ch.crc != 0)
    Serial.println("  *CRC-BAD*");    // Teensy extension for the crc-bad display

  // Downlinks: text starts after No(4) + Flight id(6); uplinks/squitters
  // right after SOT (byte 12). acarsdec prints it without indentation.
  int bodyStart = downlink ? 23 : 13;
  if (ch.len > bodyStart) {
    printBody(bodyStart);
    Serial.println();
  }
}

static void decodeAcars_ch(void) {
  unsigned char r = ch.outbits;
  switch (ch.state) {
    case WSYN:
      if (r == SYN) { ch.state = SYN2; ch.nbits = 8; ch.nSync++; return; }
      if (r == (unsigned char)~SYN) { ch.MskS ^= 2; ch.state = SYN2; ch.nbits = 8; ch.nSync++; return; }
      ch.nbits = 1; return;
    case SYN2:
      if (r == SYN) { ch.state = SOH1; ch.nbits = 8; return; }
      if (r == (unsigned char)~SYN) { ch.MskS ^= 2; ch.nbits = 8; return; }
      resetAcars(); return;
    case SOH1:
      if (r == SOH) {
        ch.state = TXT; ch.len = 0; ch.perr = 0; ch.crc = 0; ch.nbits = 8; ch.nSOH++;
        ch.MskLvlSum = 0; ch.MskBitCount = 0;
        return;
      }
      resetAcars(); return;
    case TXT:
      ch.txt[ch.len] = r; ch.len++;
      if (oddParity(r) == 0) ch.perr++;   // odd parity missing
      update_crc(ch.crc, r);
      if (r == ETX || r == ETB) { ch.state = CRC1; ch.nbits = 8; ch.nEtx++; return; }
      if (ch.len > 240) { resetAcars(); return; }
      ch.nbits = 8; return;
    case CRC1:
      ch.crc0 = r; update_crc(ch.crc, r); ch.state = CRC2; ch.nbits = 8; return;
    case CRC2:
      ch.crc1 = r; update_crc(ch.crc, r);
      frameDone();
      ch.state = END; ch.nbits = 8; return;
    case END:
      resetAcars(); ch.nbits = 8; return;
  }
}

static inline void putbit(float v) {
  ch.outbits >>= 1;
  if (v > 0) ch.outbits |= 0x80;
  ch.nbits--;
  ch.nBits++;
  if (ch.nbits <= 0) decodeAcars_ch();
}

static void printStats(void) {
  static unsigned long last = 0;
  unsigned long now_t = millis();
  double sec = (now_t - last) / 1000.0;
  Serial.print("\n--- STATS (");
  Serial.print(sec);
  Serial.print(" s) ---\nBits  : ");
  Serial.print(ch.nBits);
  Serial.print("  ("); Serial.print(sec > 0 ? ch.nBits / sec : 0.0); Serial.println(" bit/s)");
  Serial.print("Sync  : "); Serial.println(ch.nSync);
  Serial.print("SOH   : "); Serial.println(ch.nSOH);
  Serial.print("ETX   : "); Serial.println(ch.nEtx);
  Serial.print("OK    : "); Serial.println(ch.ok);
  Serial.print("Drop  : perr>3="); Serial.print(ch.dropPerr);
  Serial.print("  crc="); Serial.println(ch.dropCrc);
  Serial.print("perr-Hist (0,1,2,3,4,5..8,9..20,>20): ");
  for (int i = 0; i < 8; i++) { Serial.print(ch.hist[i]); Serial.print(" "); }
  Serial.println();
  ch.nBits = 0; ch.nSync = 0; ch.nSOH = 0; ch.nEtx = 0;
  ch.ok = 0; ch.dropPerr = 0; ch.dropCrc = 0;
  for (int i = 0; i < 8; i++) ch.hist[i] = 0;
  last = now_t;
}

static void demodMSK(const float* buf, int len) {
  unsigned int idx = ch.idx;
  double p = ch.MskPhi;
  for (int n = 0; n < len; n++) {
    double s = 1800.0 / INTRATE * 2.0 * M_PI + ch.MskDf;
    p += s;
    if (p >= 2.0 * M_PI) p -= 2.0 * M_PI;

    float in = buf[n];
    float c = (float)cos(-p), si = (float)sin(-p);
    ch.inbre[idx] = in * c;
    ch.inbim[idx] = in * si;
    idx = (idx + 1) % FLEN;

    ch.MskClk += (float)s;
    if (ch.MskClk >= (float)(3 * M_PI / 2.0 - s / 2)) {
      ch.MskClk -= (float)(3 * M_PI / 2.0);
      int o = (int)(MFLTOVER * (ch.MskClk / s + 0.5));
      if (o > MFLTOVER) o = MFLTOVER;
      float re = 0, im = 0;
      for (int j = 0; j < FLEN; j++) {
        float hh = h[o + j * MFLTOVER];
        re += hh * ch.inbre[(j + idx) % FLEN];
        im += hh * ch.inbim[(j + idx) % FLEN];
      }
      float lvl = sqrtf(re * re + im * im);
      re /= lvl + 1e-8f;
      im /= lvl + 1e-8f;
      ch.MskLvlSum += (double)(lvl * lvl) / 4.0;
      ch.MskBitCount++;

      float vo, dphi;
      if (ch.MskS & 1) {
        vo = im;
        dphi = (vo >= 0) ? -re : re;
      } else {
        vo = re;
        dphi = (vo >= 0) ? im : -im;
      }
      if (ch.MskS & 2) putbit(-vo);
      else putbit(vo);
      ch.MskS++;

      ch.MskDf = PLLC * ch.MskDf + (1.0f - PLLC) * PLLG * dphi;
    }
  }
  ch.idx = idx;
  ch.MskPhi = p;
}

void processBlock(const int16_t* buf) {
  static float fbuf[128];
  for (int i = 0; i < 128; i++) fbuf[i] = buf[i] * (1.0f / 32768.0f);
  demodMSK(fbuf, 128);
}

static void printBanner(void) {
  Serial.println("=========================================");
  Serial.println("  Teensy ACARS Decoder (acarsdec port)   ");
  Serial.println("  MSK 2400 baud, 1200/2400 Hz, 44.1 kHz  ");
  Serial.println("  (c) 2007-2025 Thierry Leconte");
  Serial.println("  (c) 2024-2026 Thibaut VARENE (f00b4r0)");
  Serial.println("  Port: (c) 2025-2026 H.Nyga (satdelight)");
  Serial.println("  License: GPL-2.0-only                   ");
  Serial.println("  'c' = show CRC-BAD frames (toggle)     ");
  Serial.println("  's' = print stats                      ");
  Serial.println("  'd' = raw sample dump (int16) on/off   ");
  Serial.println("=========================================");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Matched filter impulse response (600 Hz cosine half-wave window)
  for (int i = 0; i < FLENO; i++) {
    float v = cosf(2.0f * (float)M_PI * 600.0f / INTRATE / MFLTOVER * (i - (FLENO - 1) / 2));
    h[i] = (v < 0) ? 0.0f : v;
  }

  ch.MskPhi = 0; ch.MskClk = 0; ch.MskS = 0; ch.MskDf = 0;
  ch.idx = 0; ch.outbits = 0; ch.nbits = 8;
  ch.state = WSYN; ch.len = 0; ch.ok = 0; ch.perr = 0; ch.crc = 0;
  ch.dropPerr = 0; ch.dropCrc = 0; ch.nSync = 0; ch.nSOH = 0; ch.nEtx = 0; ch.nBits = 0;
  for (int i = 0; i < 8; i++) ch.hist[i] = 0;

  AudioMemory(32);
  audioQueue.begin();
}

void loop() {
  // Print the banner whenever a terminal opens the USB serial port
  // (DTR low->high). The Teensy keeps running while unplugged or with
  // no terminal, so setup() alone would print it only once on power-on.
  if (Serial.dtr()) {
    if (!bannerShown) {
      bannerShown = true;
      printBanner();
    }
  } else {
    bannerShown = false;
  }

  if (Serial.available()) {
    char b = Serial.read();
    if (b == 'c') {
      showCrcBad = !showCrcBad;
      Serial.print("CRC-BAD display: ");
      Serial.println(showCrcBad ? "ON" : "OFF");
    }
    if (b == 's') {
      printStats();
    }
    if (b == 'd') {
      dumpMode = !dumpMode;
      Serial.print("Dump: ");
      Serial.println(dumpMode ? "ON (raw int16 samples, 44.1 kHz)" : "OFF");
    }
  }

  if (audioQueue.available() >= 1) {
    int16_t* buffer = audioQueue.readBuffer();
    if (dumpMode) Serial.write((const uint8_t*)buffer, 256);  // 128 * int16
    processBlock(buffer);
    audioQueue.freeBuffer();
  }

  if (ledTurnOffTime > 0 && millis() > ledTurnOffTime) {
    digitalWrite(LED_PIN, LOW);
    ledTurnOffTime = 0;
  }
}
