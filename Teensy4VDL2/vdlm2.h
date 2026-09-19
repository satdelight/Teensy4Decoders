// ============================================================================
//  vdlm2.h — VDL Mode 2 (D8PSK) decoder types and interfaces
//  Ported from vdlm2dec (Thierry Leconte, GPL-2.0) to Teensy 4.x Single-Core
//  ----------------------------------------------------------------------------
//  Original: https://github.com/TLeconte/vdlm2dec
//  Port: Holger Nyga (satdelight), 2026
//  SPDX-License-Identifier: GPL-2.0-only
// ============================================================================
#ifndef VDLM2_H
#define VDLM2_H

#include <stdint.h>

#define MFLTLEN   65
#define MBUFLEN   17
#define NBPH      17
#define D8DWN     4

// Symbol / sample-rate relationships used by the Teensy port:
//   symbol rate         = 10500 Bd  (VDL2)
//   demod input rate    = 84000 Hz  (= 8 samples / symbol, as in the reference)
//   USB audio rate      = 44100 Hz
//   resampler up factor = 40/21     (44.1k -> 84k)
#define VDL2SYM   10500
#define DEMODRATE 84000

typedef struct { float r; float i; } vcplx;

typedef struct mskblk_s {
  int chn;
  int Fr;
  float ppm;
  int nbrow, nlbyte;
  unsigned char data[65][255];   // up to 8 rows x (249 data + 6 RS)
} msgblk_t;

typedef struct {
  vcplx Inbuff[MBUFLEN];          // matched filter delay line
  float Ph[NBPH * D8DWN];         // phase ring for sync detection

  int chn;
  int Fr;
  int ink, Phidx;
  float df;                       // residual frequency offset (rad/symbol)
  int clk;
  float p2err, perr;
  float pfr;
  float P1;

  unsigned int scrambler;
  unsigned int nbits;
  unsigned int nbyte, nrow, nbrow, nlbyte;
  unsigned char bits;

  enum { WSYNC, GETHEAD, GETDATA, GETFEC } state;
  msgblk_t *blk;
} channel_t;

#ifdef __cplusplus
extern "C" {
#endif

// d8psk.c
int initD8psk(channel_t *ch);
void demodD8psk(channel_t *ch, float re, float im);
unsigned int reversebits(unsigned int bits, int n);

// viterbi.c
void viterbi_init(void);
void viterbi_add(float V, int n);
float viterbi_end(unsigned int *bits);

// crc.c
extern const unsigned short crc_ccitt_table[256];
#define update_crc(crc,c) crc = (crc >> 8) ^ crc_ccitt_table[(crc ^ (c)) & 0xff];

// rs.c
int rs(unsigned char *data, int *eras_pos, int no_eras);

// acarsmsg — ACARS message fields (from vdlm2dec)
typedef struct {
  char mode;
  char reg[9];
  char ack;
  char label[3];
  char bid;
  char bs;
  char no[5];
  char fid[7];
  char txt[256];
  char be;
} acarsmsg_t;

// vdlm2.c
int initVdlm2(channel_t *ch);
void stopVdlm2(void);
void decodeVdlm2(channel_t *ch);
int  vdlm2_pending(void);
void vdlm2_process(void);
void vdlm2_stats(unsigned long *frames, unsigned long *crcbad, unsigned long *shortf);

// Teensy4VDL2.ino
void out(msgblk_t *blk, unsigned char *hdata, int l);

#ifdef __cplusplus
}
#endif
#endif