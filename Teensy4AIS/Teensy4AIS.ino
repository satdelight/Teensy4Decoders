// ============================================================================
//  Teensy4AIS - AIS (GMSK-9600, 161.975/162.025 MHz) decoder on Teensy 4.x
//  ----------------------------------------------------------------------------
//  Input : SDR++ -> Windows USB audio tap (16 bit stereo, 44100 Hz)
//          L = Channel A (161.975 MHz), R = Channel B (162.025 MHz)
//  Demod : resample 44.1k -> 48 kHz, GMSK matched filter, clock + a/d sync
//  Frame : bit sync, HDLC flag + AIVDM payload, CRC-16 check, plain text
//  Output: NMEA AIVDM sentences on Serial (USB CDC); optional live stats.
//  Keys  : 's' stats 30s, 'l' input level, 'v' NMEA plain text (all default off)
//  Board : Tools > USB Type > "Serial + MIDI + Audio" (AudioInputUSB needed).
//  Note  : DSP core (receiver.c, filter.c, protodec.c, hmalloc.c, callbacks.h)
//          lies as flat files in this sketch folder (48 kHz calibrated).
//  ----------------------------------------------------------------------------
//  Based on rtl-ais / AIS-receiver by Ruben Undheim, Heikki Hannikainen,
//          Oh7lzb and Tomi Manninen (GPL-2.0). Port to Teensy 4.x: Holger Nyga.
//  License: GPL-2.0-or-later (see the DSP files for the full notice).
// ============================================================================

#include <Audio.h>
#include <string.h>

extern "C" {
#include "receiver.h"
#include "callbacks.h"
}

/* --------------------------------------------------------------------------
 * Non-blocking output buffer
 * --------------------------------------------------------------------------
 * The USB serial write blocks as soon as the host terminal can no longer
 * keep up (one frame per line at 115200 baud, bursts of many frames in a
 * row).  Blocking in the middle of the decoder stops loop() from reading the
 * audio queue and the Teensy effectively hangs.  All decoded output goes
 * into this byte ring buffer instead; loop() drains it only as fast as
 * Serial.availableForWrite() allows, so a slow terminal can never stall the
 * decoder.  If the buffer overflows, the excess bytes are dropped (counted
 * in outDropped) instead of blocking. */

#define OUTBUF_SIZE 4096

class OutBuf {
public:
	uint8_t tb[OUTBUF_SIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	volatile uint32_t dropped;

	OutBuf() : head(0), tail(0), dropped(0) {}

	uint16_t available() const {
		return (uint16_t)((head - tail) & (OUTBUF_SIZE - 1));
	}

	uint16_t room() const {
		return (uint16_t)((OUTBUF_SIZE - 1 - available()) & (OUTBUF_SIZE - 1));
	}

	void putc(uint8_t c) {
		uint16_t next = (head + 1) & (OUTBUF_SIZE - 1);
		if (next == tail) {          /* full: drop instead of blocking */
			dropped++;
			return;
		}
		tb[head] = c;
		head = next;
	}

	void put(const uint8_t *p, uint16_t n) {
		while (n--)
			putc(*p++);
	}

	size_t write(const char *s, size_t n) {
		put((const uint8_t *)s, (uint16_t)n);
		return n;
	}

	size_t write(char c) {
		putc((uint8_t)c);
		return 1;
	}

	void print(const char *s) {
		put((const uint8_t *)s, (uint16_t)strlen(s));
	}

	void print(const __FlashStringHelper *p) {
		const char *s = (const char *)p;
		char c;
		while ((c = (char)pgm_read_byte((const uint8_t *)s++)))
			putc((uint8_t)c);
	}

	void print(char c) {
		putc((uint8_t)c);
	}

	void print(int v) {
		char b[12];
		snprintf(b, sizeof(b), "%d", v);
		print(b);
	}

	void print(unsigned int v) {
		char b[12];
		snprintf(b, sizeof(b), "%u", v);
		print(b);
	}

	void print(long v) {
		char b[16];
		snprintf(b, sizeof(b), "%ld", v);
		print(b);
	}

	void print(unsigned long v) {
		char b[16];
		snprintf(b, sizeof(b), "%lu", v);
		print(b);
	}

	void print(float v, int d) {
		char b[24];
		snprintf(b, sizeof(b), "%.*f", d, (double)v);
		print(b);
	}

	void println() {
		putc('\n');
	}

	void println(const char *s) {
		print(s);
		println();
	}

	void println(const __FlashStringHelper *p) {
		print(p);
		println();
	}

	void println(int v) {
		print(v);
		println();
	}

	void println(unsigned int v) {
		print(v);
		println();
	}

	void println(float v, int d) {
		print(v, d);
		println();
	}

	/* Copy up to maxn bytes to dst; return count copied. */
	uint16_t pull(uint8_t *dst, uint16_t maxn) {
		uint16_t n = 0;
		while (n < maxn && tail != head) {
			dst[n++] = tb[tail];
			tail = (tail + 1) & (OUTBUF_SIZE - 1);
		}
		return n;
	}
};

static OutBuf outb;

/* --------------------------------------------------------------------------
 * Configuration
 * -------------------------------------------------------------------------- */

/* AIS_STEREO = 1: both channels (L=161.975, R=162.025); 0: channel A only.
 * With only one tuned frequency the USB tap yields mono audio; R is then
 * identical to L -- disable B so the statistics are not counted twice. */
#define AIS_STEREO 0

#define RATIO_N   160
#define RATIO_D   147
#define BLK_IN    AUDIO_BLOCK_SAMPLES               /* 128 */
#define BLK_OUT   (BLK_IN * RATIO_N / RATIO_D)      /* 139 */

/* --------------------------------------------------------------------------
 * Global objects and variables
 * -------------------------------------------------------------------------- */

/* Audio input: L = CH-A, R = CH-B */
AudioInputUSB      usbIn;
AudioRecordQueue   qA;
#if AIS_STEREO
AudioRecordQueue   qB;
#endif
AudioConnection    patchA(usbIn, 0, qA, 0);
#if AIS_STEREO
AudioConnection    patchB(usbIn, 1, qB, 0);
#endif

/* DSP instances, one per channel */
static struct receiver *rxA;
#if AIS_STEREO
static struct receiver *rxB;
#endif

/* Diagnostics state */
static float minLvl     = 100.0f;
static float maxLvl     = 0.0f;
static bool  bLvlOn     = false;
static bool  bStatsOn   = false;
static bool  bVerbose   = false;      /* 'v': replace raw NMEA with plain text */

/* --------------------------------------------------------------------------
 * Resampler 44.1 kHz -> 48 kHz (ratio 160/147)
 * -------------------------------------------------------------------------- */

/*
 * Linear interpolator. Since 139 * 128 / 139 == 128, the phase at every
 * block boundary is exactly integer -- the resampler needs no state. At the
 * block end at most one output sample is dropped (held).
 */
static void resample_44_1_to_48k(const int16_t *in, int nIn, int16_t *out, int nOut)
{
	int j, k, frac;
	int32_t a, b;

	for (j = 0; j < nOut; j++) {
		k = (j * nIn) / nOut;
		frac = (j * nIn) % nOut;

		if (k >= nIn - 1) {
			out[j] = in[nIn - 1];
			continue;
		}
		a = in[k];
		b = in[k + 1];
		out[j] = (int16_t)(a + (int32_t)(((int64_t)(b - a) * frac) / nOut));
	}
}

/* --------------------------------------------------------------------------
 * AIVDM plain-text decoder (toggled by key 'v')
 * -------------------------------------------------------------------------- */
#define Serial outb   /* decoder + callbacks write into the ring buffer */

/* Bit mask: payload is a sequence of 6-bit characters, MSB first. */
static uint32_t vbits(const uint8_t *payload, int nchars,
                      int start, int length, bool signed_)
{
	int i;
	uint32_t val = 0;
	for (i = 0; i < length; i++) {
		uint8_t c = payload[(start + i) / 6];
		val = (val << 1) | ((c >> (5 - ((start + i) % 6))) & 1);
	}
	if (signed_ && (val & (1u << (length - 1))))
		val -= (1u << length);
	(void)nchars;
	return val;
}

/* 6-bit-ASCII field per ITU-R M.1371 / IEC-PAS:
 * Value 0..31  -> '@'..'_' (ASCII 64..95)
 * Value 32..63 -> ' '..'?' (ASCII 32..63)
 * Padding ('@'/space) at the field end is removed. */
static void vprint_text(const uint8_t *p, int n, int start, int chars)
{
	char buf[64];
	int k, end = 0;

	if (chars > (int)(sizeof(buf) - 1))
		chars = (int)(sizeof(buf) - 1);
	for (k = 0; k < chars; k++) {
		uint8_t v = (uint8_t)vbits(p, n, start + k * 6, 6, false);
		buf[k] = (v < 32) ? (char)(v + 64) : (char)v;
		if (buf[k] != '@' && buf[k] != ' ')
			end = k + 1;
	}
	buf[end] = '\0';
	Serial.print(buf);
}

/* Print a coordinate (1/600000 deg) with 6 decimal places. */
static void vprint_coord(int32_t milli)
{
	bool neg = milli < 0;

	if (neg) {
		milli = -milli;
		Serial.write('-');
	}
	Serial.print((int)(milli / 600000));
	Serial.write('.');
	{
		int32_t f = milli % 600000;
		int d;
		for (d = 0; d < 6; d++) {
			f *= 10;
			Serial.print((int)(f / 600000));
			f %= 600000;
		}
	}
}

/* Print a tenth value (e.g. SOG in 0.1 kn) with 1 decimal place. */
static void vprint_tenths(uint32_t v)
{
	Serial.print((int)(v / 10));
	Serial.write('.');
	Serial.print((int)(v % 10));
}

/* Print a COG value (0.1 deg, 12 bits): 3600 = not available. */
static void vprint_cog12(uint32_t v)
{
	if (v == 3600)
		Serial.print(F("n/a"));
	else
		vprint_tenths(v);
}

/* Print a hundredth value (e.g. draught in 0.01 m) with 2 decimal places. */
static void vprint_centis(uint32_t v)
{
	Serial.print((int)(v / 100));
	Serial.write('.');
	Serial.print((int)((v / 10) % 10));
	Serial.print((int)(v % 10));
}

/* Print a byte as a 2-digit lowercase hex number (as aisdec.py does). */
static void vprint_hex(uint8_t b)
{
	static const char dig[] = "0123456789abcdef";
	Serial.write(dig[b >> 4]);
	Serial.write(dig[b & 15]);
}

static const char *const vnav_status[] = {
	"under way using engine", "at anchor", "not under command",
	"restricted manoeuvrability", "constrained by draught", "moored",
	"aground", "engaged in fishing", "under way sailing",
	"reserved HSC", "reserved WIG", "AtoN", "reserved", "reserved",
	"AIS-SART", "undefined"
};

/* Ship type from ITU-R M.1371 table 54, categorized by the leading digit
 * (special craft subtypes 30-39 are named individually). */
static const char *v_ship_type_name(uint32_t t)
{
	if (t == 0)
		return "n/a";
	if (t >= 30 && t <= 35)
		return "fishing";
	if (t == 36)
		return "sailing";
	if (t == 37)
		return "pleasure craft";
	if (t >= 38 && t <= 39)
		return "fishing";
	if (t >= 20 && t <= 29)
		return "WIG";
	if (t >= 40 && t <= 49)
		return "HSC";
	if (t >= 50 && t <= 59)
		return "passenger";
	if (t >= 60 && t <= 69)
		return "cargo";
	if (t >= 70 && t <= 79)
		return "tanker";
	if (t >= 80 && t <= 89)
		return "other";
	if (t >= 90 && t <= 99)
		return "other";
	return "?";
}

static void vdecode_types123(const uint8_t *p, int n)
{
	uint32_t nav = vbits(p, n, 38, 4, false);

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  SOG:      "));
	vprint_tenths(vbits(p, n, 50, 10, false));
	Serial.println(F(" kn"));
	Serial.print(F("  Pos:      "));
	vprint_coord((int32_t)vbits(p, n, 89, 27, true));
	Serial.print(F(" N "));
	vprint_coord((int32_t)vbits(p, n, 61, 28, true));
	Serial.print(F(" E   (Accuracy "));
	Serial.print(vbits(p, n, 60, 1, false) ? F("high") : F("low"));
	Serial.println(F(")"));
	Serial.print(F("  COG:      "));
	vprint_cog12(vbits(p, n, 116, 12, false));
	Serial.print(F(" deg   Heading: "));
	{
		uint32_t hdg = vbits(p, n, 128, 9, false);
		if (hdg == 511 || hdg > 359)
			Serial.print(F("n/a"));
		else
			Serial.print((int)hdg);
	}
	Serial.println(F(" deg"));
	Serial.print(F("  Timestamp:"));
	Serial.print((int)vbits(p, n, 136, 6, false));
	Serial.println(F(" s"));
}

static void vdecode_type5(const uint8_t *p, int n)
{
	uint32_t mt = vbits(p, n, 274, 4, false);
	uint32_t dd = vbits(p, n, 278, 5, false);
	uint32_t hh = vbits(p, n, 283, 5, false);
	uint32_t mm = vbits(p, n, 288, 6, false);
	uint32_t st = vbits(p, n, 232, 8, false);

	Serial.print(F("  MMSI:     "));
	Serial.print((int)vbits(p, n, 8, 30, false));
	Serial.print(F("   IMO: "));
	Serial.println((int)vbits(p, n, 40, 30, false));
	Serial.print(F("  Callsign: "));
	vprint_text(p, n, 70, 7);
	Serial.println();
	Serial.print(F("  Name:     "));
	vprint_text(p, n, 112, 20);
	Serial.println();
	Serial.print(F("  Ship type:"));
	Serial.print(v_ship_type_name(st));
	Serial.print(F(" ("));
	Serial.print((int)st);
	Serial.println(F(")"));
	Serial.print(F("  Size:     Length "));
	Serial.print((int)(vbits(p, n, 240, 9, false) + vbits(p, n, 249, 9, false)));
	Serial.print(F(" m  Beam "));
	Serial.print((int)(vbits(p, n, 258, 6, false) + vbits(p, n, 264, 6, false)));
	Serial.println(F(" m"));
	Serial.print(F("  ETA:      "));
	if (mt) {
		if (dd < 10)
			Serial.print('0');
		Serial.print((int)dd);
		Serial.write('.');
		if (mt < 10)
			Serial.print('0');
		Serial.print((int)mt);
		Serial.write(' ');
		if (hh < 10)
			Serial.print('0');
		Serial.print((int)hh);
		Serial.write(':');
		if (mm < 10)
			Serial.print('0');
		Serial.print((int)mm);
		Serial.println(F(" UTC"));
	} else {
		Serial.println(F("n/a"));
	}
	Serial.print(F("  Draught:  "));
	if (vbits(p, n, 294, 8, false) == 0)
		Serial.print(F("n/a"));
	else
		vprint_tenths(vbits(p, n, 294, 8, false));
	Serial.print(F(" m   Destination: "));
	vprint_text(p, n, 302, 20);
	Serial.println();
}

/* ERI ship type class for a few common codes */
static const char *v_eri_code_name(uint32_t c)
{
	switch (c) {
	case 8000: return "Vessel type unknown";
	case 8010: return "Motor cargo ship";
	case 8020: return "Motor tanker";
	case 8021: return "Motor tanker (N)";
	case 8022: return "Motor tanker (C)";
	case 8023: return "Motor tanker (dry)";
	case 8030: return "Container ship";
	case 8040: return "Gas tanker";
	case 8050: return "Cargo/tug";
	case 8060: return "Tanker/tug";
	case 8370: return "Passenger ship";
	default:   return NULL;
	}
}

/* Inland AIS regional message DAC 200 / FI 10: ship static + voyage. */
static void vdecode_rfm10(const uint8_t *p, int n)
{
	const char *cn = v_eri_code_name(vbits(p, n, 127, 14, false));

	Serial.print(F("  Inland RFM10:"));
	Serial.println();
	Serial.print(F("    ENI:      "));
	vprint_text(p, n, 56, 8);
	Serial.println();
	Serial.print(F("    Length:   "));
	vprint_tenths(vbits(p, n, 104, 13, false));
	Serial.print(F(" m"));
	Serial.println();
	Serial.print(F("    Beam:     "));
	vprint_tenths(vbits(p, n, 117, 10, false));
	Serial.print(F(" m"));
	Serial.println();
	Serial.print(F("    Type:     "));
	if (cn) {
		Serial.print(cn);
		Serial.write(' ');
	}
	Serial.print(F("("));
	Serial.print((int)vbits(p, n, 127, 14, false));
	Serial.println(F(")"));
	Serial.print(F("    Draught:  "));
	vprint_centis(vbits(p, n, 144, 11, false));
	Serial.print(F(" m"));
	Serial.println();
	Serial.print(F("    Cones:    "));
	Serial.print((int)vbits(p, n, 141, 3, false));
	Serial.println();
	Serial.print(F("    Loaded:   "));
	switch (vbits(p, n, 155, 2, false)) {
	case 1:
		Serial.println(F("loaded"));
		break;
	case 2:
		Serial.println(F("unloaded"));
		break;
	default:
		Serial.println(F("n/a"));
		break;
	}
}

static void vdecode_type8(const uint8_t *p, int n)
{
	uint32_t dac = vbits(p, n, 40, 10, false);
	uint32_t fi  = vbits(p, n, 50, 6, false);
	int i;
	int nb = n * 6 - 56;

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  DAC:      "));
	Serial.print((int)dac);
	Serial.print(F("  FI: "));
	Serial.println((int)fi);
	Serial.print(F("  Data:     "));
	if (nb - nb % 8 > 0) {
		for (i = 0; i < nb - nb % 8; i += 8) {
			if (i)
				Serial.write(' ');
			uint8_t b = (uint8_t)vbits(p, n, 56 + i, 8, false);
			vprint_hex(b);
		}
	}
	Serial.println();
	if (dac == 200 && fi == 10)
		vdecode_rfm10(p, n);
}

/* Type 15: Interrogation - a base station asks 1-2 transponders for
 * (type, slot). Layout per ITU-R M.1371 / gpsd AIVDM docs:
 * target MMSI from bit 40, (optional second request from bit 90),
 * (optional second target station from bit 110). */
static void vdecode_type15(const uint8_t *p, int n)
{
	int nb = n * 6;

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Interrogate target "));
	Serial.print((int)vbits(p, n, 40, 30, false));
	Serial.print(F(", request Type "));
	Serial.print((int)vbits(p, n, 70, 6, false));
	Serial.print(F(", slot "));
	Serial.println((int)vbits(p, n, 76, 12, false));
	if (nb >= 110) {
		Serial.print(F("  Also request Type "));
		Serial.print((int)vbits(p, n, 90, 6, false));
		Serial.print(F(", slot "));
		Serial.println((int)vbits(p, n, 96, 12, false));
	}
	if (nb >= 160) {
		Serial.print(F("  Interrogate target "));
		Serial.print((int)vbits(p, n, 110, 30, false));
		Serial.print(F(", request Type "));
		Serial.print((int)vbits(p, n, 140, 6, false));
		Serial.print(F(", slot "));
		Serial.println((int)vbits(p, n, 146, 12, false));
	}
}

/* Type 4: Base Station Report - time/position of a base station. */
static void vdecode_type4(const uint8_t *p, int n)
{
	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Time:     "));
	{
		uint32_t y = vbits(p, n, 38, 14, false);
		uint32_t mo = vbits(p, n, 52, 4, false);
		uint32_t d = vbits(p, n, 56, 5, false);
		uint32_t h = vbits(p, n, 61, 5, false);
		uint32_t mi = vbits(p, n, 66, 6, false);
		uint32_t s = vbits(p, n, 72, 6, false);
		if (y) {
			Serial.print((int)y);
			Serial.write('-');
			if (mo < 10)
				Serial.write('0');
			Serial.print((int)mo);
			Serial.write('-');
			if (d < 10)
				Serial.write('0');
			Serial.print((int)d);
			Serial.write(' ');
			if (h < 10)
				Serial.write('0');
			Serial.print((int)h);
			Serial.write(':');
			if (mi < 10)
				Serial.write('0');
			Serial.print((int)mi);
			Serial.write(':');
			if (s < 10)
				Serial.write('0');
			Serial.println((int)s);
		} else {
			Serial.println(F("n/a"));
		}
	}
	Serial.print(F("  Pos:      "));
	vprint_coord((int32_t)vbits(p, n, 107, 27, true));
	Serial.print(F(" N "));
	vprint_coord((int32_t)vbits(p, n, 79, 28, true));
	Serial.println(F(" E"));
	Serial.print(F("  EPFD:     type "));
	Serial.print((int)vbits(p, n, 134, 4, false));
	Serial.print(F("   RAIM: "));
	Serial.println(vbits(p, n, 148, 1, false) ? F("yes") : F("no"));
}

/* Type 6: Binary Addressed Message (e.g. lock/bridge/people). */
static void vdecode_type6(const uint8_t *p, int n)
{
	int i, nb = n * 6, cnt = 0;

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  To:       "));
	Serial.println((int)vbits(p, n, 40, 30, false));
	Serial.print(F("  DAC:      "));
	Serial.print((int)vbits(p, n, 72, 10, false));
	Serial.print(F("  FID: "));
	Serial.println((int)vbits(p, n, 82, 6, false));
	if (nb > 88) {
		Serial.print(F("  Data:     "));
		for (i = 88; i + 8 <= nb; i += 8) {
			if (i > 88)
				Serial.write(' ');
			uint8_t b = (uint8_t)vbits(p, n, i, 8, false);
			vprint_hex(b);
			cnt++;
		}
		if (!cnt)
			Serial.println(F("(empty)"));
		else
			Serial.println();
	}
}

static const char *v_epfd_name(uint32_t e)
{
	switch (e) {
	case 1: return "GPS";
	case 2: return "GLONASS";
	case 3: return "GPS/GLONASS";
	case 7: return "surveyed";
	case 8: return "Galileo";
	default: return NULL;
	}
}

/* Type 9: SAR-Aircraft Position Report. */
static void vdecode_type9(const uint8_t *p, int n)
{
	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Alt:      "));
	Serial.print((int)vbits(p, n, 38, 12, false));
	Serial.print(F(" m   SOG: "));
	vprint_tenths(vbits(p, n, 50, 10, false));
	Serial.println(F(" kn"));
	Serial.print(F("  Pos:      "));
	vprint_coord((int32_t)vbits(p, n, 89, 27, true));
	Serial.print(F(" N "));
	vprint_coord((int32_t)vbits(p, n, 61, 28, true));
	Serial.print(F(" E   (Accuracy "));
	Serial.print(vbits(p, n, 60, 1, false) ? F("high") : F("low"));
	Serial.println(F(")"));
	Serial.print(F("  COG:      "));
	vprint_cog12(vbits(p, n, 116, 12, false));
	Serial.print(F(" deg   TS: "));
	Serial.println((int)vbits(p, n, 128, 6, false));
}

/* Type 14: Safety-Related Broadcast - text message (6-bit ASCII). */
static void vdecode_type14(const uint8_t *p, int n)
{
	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Text:     "));
	vprint_text(p, n, 40, (n * 6 - 40) / 6);
	Serial.println();
}

/* Type 18: Standard Class B CS Position Report. */
static void vdecode_type18(const uint8_t *p, int n)
{
	uint32_t hdg = vbits(p, n, 124, 9, false);

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  SOG:      "));
	vprint_tenths(vbits(p, n, 46, 10, false));
	Serial.println(F(" kn"));
	Serial.print(F("  Pos:      "));
	vprint_coord((int32_t)vbits(p, n, 85, 27, true));
	Serial.print(F(" N "));
	vprint_coord((int32_t)vbits(p, n, 57, 28, true));
	Serial.print(F(" E   (Accuracy "));
	Serial.print(vbits(p, n, 56, 1, false) ? F("high") : F("low"));
	Serial.println(F(")"));
	Serial.print(F("  COG:      "));
	vprint_cog12(vbits(p, n, 112, 12, false));
	Serial.print(F(" deg   Heading: "));
	if (hdg > 359)
		Serial.print(F("n/a"));
	else
		Serial.print((int)hdg);
	Serial.println(F(" deg"));
	Serial.print(F("  TS:       "));
	Serial.println((int)vbits(p, n, 133, 6, false));
}

/* Type 19: Extended Class B - like 18 + name/type/dimensions.
 * (Type 18 fields are printed by vdecode_type18.) */
static void vdecode_type19(const uint8_t *p, int n)
{
	vdecode_type18(p, n);
	Serial.print(F("  Name:     "));
	vprint_text(p, n, 143, 20);
	Serial.println();
	Serial.print(F("  Type:     "));
	Serial.println((int)vbits(p, n, 263, 8, false));
	Serial.print(F("  Size:     "));
	Serial.print((int)(vbits(p, n, 271, 9, false) + vbits(p, n, 280, 9, false)));
	Serial.print(F(" x "));
	Serial.print((int)(vbits(p, n, 289, 6, false) + vbits(p, n, 295, 6, false)));
	Serial.println(F(" m"));
}

/* Type 21: AtoN Report (buoy/light). */
static void vdecode_type21(const uint8_t *p, int n)
{
	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Type:     "));
	Serial.println((int)vbits(p, n, 38, 5, false));
	Serial.print(F("  Name:     "));
	vprint_text(p, n, 43, 20);
	Serial.println();
	Serial.print(F("  Pos:      "));
	vprint_coord((int32_t)vbits(p, n, 192, 27, true));
	Serial.print(F(" N "));
	vprint_coord((int32_t)vbits(p, n, 164, 28, true));
	Serial.println(F(" E"));
}

/* Type 24: Static Data Report (Class B). Part A = name, Part B = static. */
static void vdecode_type24(const uint8_t *p, int n)
{
	uint32_t part = vbits(p, n, 38, 2, false);

	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	if (part == 0) {
		Serial.print(F("  Name:     "));
		vprint_text(p, n, 40, 20);
		Serial.println();
	} else if (part == 1) {
		Serial.print(F("  Type:     "));
		Serial.println((int)vbits(p, n, 40, 8, false));
		Serial.print(F("  Callsign: "));
		vprint_text(p, n, 90, 7);
		Serial.println();
		Serial.print(F("  Size:     "));
		Serial.print((int)(vbits(p, n, 132, 9, false) + vbits(p, n, 141, 9, false)));
		Serial.print(F(" x "));
		Serial.print((int)(vbits(p, n, 150, 6, false) + vbits(p, n, 156, 6, false)));
		Serial.println(F(" m"));
	} else {
		Serial.println(F("  (part number n/a)"));
	}
}

/* Type 27: Long Range AIS (satellite) - compact, 96 bits. */
static void vdecode_type27(const uint8_t *p, int n)
{
	Serial.print(F("  MMSI:     "));
	Serial.println((int)vbits(p, n, 8, 30, false));
	Serial.print(F("  Status:   "));
	Serial.println(vnav_status[vbits(p, n, 40, 4, false) & 15]);
	Serial.print(F("  Pos:      "));
	{
		/* Lon/Lat as minutes/10 (I1): 1 = 0.1 arc-minute */
		int32_t lat = vbits(p, n, 62, 17, true);
		int32_t lon = vbits(p, n, 44, 18, true);
		Serial.print((float)lat / 600.0f, 5);
		Serial.print(F(" N "));
		Serial.print((float)lon / 600.0f, 5);
		Serial.println(F(" E"));
	}
	Serial.print(F("  SOG:      "));
	Serial.print((int)vbits(p, n, 79, 6, false));
	Serial.println(F(" kn"));
	Serial.print(F("  COG:      "));
	Serial.println((int)vbits(p, n, 85, 9, false));
}

static void vprint_decoded(const uint8_t *p, int n)
{
	uint32_t t = vbits(p, n, 0, 6, false);

	Serial.println(F("============================================================"));
	if (t < 4) {
		Serial.print(F("Type "));
		Serial.print((int)t);
		Serial.print(F(" - "));
		Serial.println(vnav_status[vbits(p, n, 38, 4, false) & 15]);
		vdecode_types123(p, n);
	} else if (t == 4) {
		Serial.println(F("Type 4 - Base Station Report"));
		vdecode_type4(p, n);
	} else if (t == 5) {
		Serial.println(F("Type 5 - Static and voyage data"));
		vdecode_type5(p, n);
	} else if (t == 6) {
		Serial.println(F("Type 6 - Binary Addressed Message"));
		vdecode_type6(p, n);
	} else if (t == 8) {
		Serial.println(F("Type 8 - Binary Broadcast"));
		vdecode_type8(p, n);
	} else if (t == 9) {
		Serial.println(F("Type 9 - SAR Aircraft Position Report"));
		vdecode_type9(p, n);
	} else if (t == 14) {
		Serial.println(F("Type 14 - Safety-Related Broadcast"));
		vdecode_type14(p, n);
	} else if (t == 15) {
		Serial.println(F("Type 15 - Interrogation"));
		vdecode_type15(p, n);
	} else if (t == 18) {
		Serial.println(F("Type 18 - Standard Class B Position Report"));
		vdecode_type18(p, n);
	} else if (t == 19) {
		Serial.println(F("Type 19 - Extended Class B Position Report"));
		vdecode_type19(p, n);
	} else if (t == 21) {
		Serial.println(F("Type 21 - Aid-to-Navigation Report"));
		vdecode_type21(p, n);
	} else if (t == 24) {
		Serial.println(F("Type 24 - Static Data Report"));
		vdecode_type24(p, n);
	} else if (t == 27) {
		Serial.println(F("Type 27 - Long Range AIS Broadcast"));
		vdecode_type27(p, n);
	} else {
		Serial.print(F("Type "));
		Serial.print((int)t);
		Serial.println(F(" (not decoded individually)"));
	}
}

/* --------------------------------------------------------------------------
 * Callbacks
 * -------------------------------------------------------------------------- */

static void my_nmea_callback(const char *sentence, unsigned int length,
                             unsigned char sentences,
                             unsigned char sentencenum)
{
	/* Buffer for reassembled multi-part messages */
	static uint8_t vpayload[128];
	static int vpCount = 0;

	if (!bVerbose) {
		while (length > 0 && (sentence[length - 1] == '\r' || sentence[length - 1] == '\n'))
			length--;
		Serial.write(sentence, length);
		Serial.println();
		return;
	}

	/* Collect payload (last field before the checksum) */
	if (sentencenum == 1)
		vpCount = 0;
	{
		int j;
		const char *p = sentence;
		for (j = 0; j < 5; j++) {
			p = strchr(p, ',');
			if (!p)
				return;
			p++;
		}
		while (*p && *p != '*' && *p != ',' && vpCount < (int)sizeof(vpayload)) {
			unsigned char c = (unsigned char)*p++;
			vpayload[vpCount++] = (c >= 96) ? (c - 56) : (c - 48);
		}
	}

	if (bVerbose && sentencenum == sentences)
		vprint_decoded(vpayload, vpCount);
}

/* Level callback: level is the peak of the incoming audio blocks,
 * as percent of full scale. Output throttled to ~1/s, plus every 5 s a
 * min/max window. Disable with key 'l'. */
static void my_level_callback(float level, int channel, unsigned char high)
{
	static unsigned long t0 = 0;
	static unsigned long last = 0;
	unsigned long now = millis();

	if (!bLvlOn)
		return;

	if (now - last < 1000)
		return;
	last = now;

	if (level < minLvl) minLvl = level;
	if (level > maxLvl) maxLvl = level;
	if (high)
		Serial.print(F("[LVL+] "));
	else
		Serial.print(F("[LVL-] "));
	Serial.print(level, 1);
	Serial.print(F("  CH="));
	Serial.print((char)('A' + channel));
	if (now - t0 >= 5000) {
		Serial.print(F("   min="));
		Serial.print(minLvl, 1);
		Serial.print(F(" max="));
		Serial.print(maxLvl, 1);
		minLvl = 100.0f;
		maxLvl = 0.0f;
		t0 = now;
	}
	Serial.println();
}

/* Decoder counters: recv = good (CRC ok), crc_err = CRC error,
 * badsize = dropped due to wrong length. */
static void dump_stats(void)
{
	Serial.print(F("A: recv="));
	Serial.print(rxA->decoder->receivedframes);
	Serial.print(F(" crc_err="));
	Serial.print(rxA->decoder->lostframes);
	Serial.print(F(" badsize="));
	Serial.println(rxA->decoder->lostframes2);
#if AIS_STEREO
	Serial.print(F("B: recv="));
	Serial.print(rxB->decoder->receivedframes);
	Serial.print(F(" crc_err="));
	Serial.print(rxB->decoder->lostframes);
	Serial.print(F(" badsize="));
	Serial.println(rxB->decoder->lostframes2);
#endif
}

/* --------------------------------------------------------------------------
 * Setup / Loop
 * -------------------------------------------------------------------------- */
#undef Serial        /* from here on: real USB serial for keys and boot text */

void setup()
{
	Serial.begin(115200);
	while (!Serial && millis() < 3000)
		;

	AudioMemory(64);

	qA.begin();
#if AIS_STEREO
	qB.begin();
#endif

	rxA = init_receiver('A', 1, 0, 0);
#if AIS_STEREO
	rxB = init_receiver('B', 1, 0, 0);
#endif

	/* The callback variables are defined in protodec.c and receiver.c;
	 * here only the pointers are set. */
	on_nmea_sentence_received = my_nmea_callback;
	on_sound_level_changed    = my_level_callback;

	Serial.println(F("=== Teensy4AIS: GMSK-9600, 44.1k tap -> 48k DSP ==="));
	Serial.println(F("Keys: 's' stats 30s, 'l' level, 'v' NMEA plain text"));
}

void loop()
{
	static uint8_t out_chunk[64];

	static int16_t ia[BLK_IN];
	static int16_t oa[BLK_OUT];
#if AIS_STEREO
	static int16_t ib[BLK_IN];
	static int16_t ob[BLK_OUT];
#endif
	static unsigned long lastStats = 0;
	unsigned long now = millis();

	/* Drain the output ring buffer only as fast as the USB-CDC TX can take
	 * it, so a slow terminal never blocks the decoder. */
	while (outb.available() > 0) {
		int room = Serial.availableForWrite();
		uint16_t pull = outb.available();
		if (pull > (uint16_t)room)
			pull = (uint16_t)room;
		if (pull > sizeof(out_chunk))
			pull = sizeof(out_chunk);
		if (pull == 0)
			break;
		Serial.write(out_chunk, outb.pull(out_chunk, pull));
	}

	while (Serial.available()) {
		char c = (char)Serial.read();
		if (c == 's') {
			bStatsOn = !bStatsOn;
			Serial.print(F("[stats "));
			Serial.print(bStatsOn ? "ON" : "OFF");
			Serial.println(F("]"));
			if (bStatsOn)
				lastStats = millis();
		} else if (c == 'l') {
			bLvlOn = !bLvlOn;
			Serial.print(F("[lvl "));
			Serial.print(bLvlOn ? "ON" : "OFF");
			Serial.println(F("]"));
		} else if (c == 'v') {
			bVerbose = !bVerbose;
			Serial.print(F("[verbose "));
			Serial.print(bVerbose ? "ON - AIVDM plain text" : "OFF - raw NMEA");
			Serial.println(F("]"));
		}
	}

	if (bStatsOn && now - lastStats >= 30000) {
		lastStats = now;
		dump_stats();
	}

#if AIS_STEREO
	if (qA.available() >= 1 && qB.available() >= 1) {
#else
	if (qA.available() >= 1) {
#endif
		memcpy(ia, qA.readBuffer(), sizeof(ia));
		qA.freeBuffer();

		resample_44_1_to_48k(ia, BLK_IN, oa, BLK_OUT);
		receiver_run(rxA, (short *)oa, BLK_OUT);
#if AIS_STEREO
		memcpy(ib, qB.readBuffer(), sizeof(ib));
		qB.freeBuffer();
		resample_44_1_to_48k(ib, BLK_IN, ob, BLK_OUT);
		receiver_run(rxB, (short *)ob, BLK_OUT);
#endif
	}
}
