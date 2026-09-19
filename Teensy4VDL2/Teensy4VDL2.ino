// ============================================================================
//  Teensy4VDL2 - VDL Mode 2 (pi/4-D8PSK @ 10500 Bd) decoder on Teensy 4.x
//  ----------------------------------------------------------------------------
//  Input : SDR++ RAW mode -> USB audio (16 bit stereo, 44100 Hz), L = I, R = Q
//  Demod : resample 44.1k -> 84 kHz, matched filter, sync + timing recovery
//  Frame : viterbi header FEC, RS(255,249), HDLC un-stuffing, CRC-16/FCS
//  Based on vdlm2dec (Thierry Leconte, GPL-2.0). Port: Holger Nyga 2026.
// ============================================================================
#include <Audio.h>
#include "vdlm2.h"

#define VDL2FREQ 136975000u    // RF center, only used for ppm reporting

#define RATIO_N   21.0f        // 40 output (84k) per 21 input (44.1k) samples
#define RATIO_D   40.0f

AudioInputUSB        audioIn;
AudioOutputUSB       audioOut;
AudioRecordQueue     qI;
AudioRecordQueue     qQ;
AudioConnection      patch1(audioIn, 0, qI, 0);
AudioConnection      patch2(audioIn, 1, qQ, 0);
AudioConnection      patch3(audioIn, 0, audioOut, 0);
AudioConnection      patch4(audioIn, 1, audioOut, 1);

#define LED_ON_MS 15UL          // pulse width of the "frame received" LED blink
static bool ledOn = false;          // LED currently lit (software state)
static unsigned long ledOffAt = 0;  // timestamp when the LED may go off again


channel_t ch;

static float rszph = 0.0f;     // resampler phase 0..1 between input samples
static float rpI = 0.0f, rpQ = 0.0f;   // previous input sample
static uint16_t rszInit = 0;

static inline void resample_and_demod(float l, float r)
{
	if (!rszInit) {
		rpI = l;
		rpQ = r;
		rszInit = 1;
		return;
	}
	while (rszph <= 1.0f) {
		float f = rszph;
		float Io = rpI + (l - rpI) * f;
		float Qo = rpQ + (r - rpQ) * f;
		demodD8psk(&ch, Io, Qo);
		rszph += RATIO_N / RATIO_D;
	}
	rszph -= 1.0f;
	rpI = l;
	rpQ = r;
}

// ---------------- AVLC / ACARS output (ported from vdlm2dec out.c) ----------------

static unsigned int icaoaddr(unsigned char *hdata)
{
	return (reversebits(hdata[0] >> 2, 6) << 21) |
	       (reversebits(hdata[1] >> 1, 7) << 14) |
	       (reversebits(hdata[2] >> 1, 7) << 7) |
	       (reversebits(hdata[3] >> 1, 7));
}

static void outaddr(unsigned int addr)
{
	unsigned int type = addr >> 24;
	addr = addr & 0xffffff;

	switch (type) {
	case 0:  Serial.print(F("T0: ")); break;
	case 1:  Serial.print(F("Aircraft: ")); break;
	case 2:  Serial.print(F("T2: ")); break;
	case 3:  Serial.print(F("T3: ")); break;
	case 4:  Serial.print(F("GroundA: ")); break;
	case 5:  Serial.print(F("GroundD: ")); break;
	case 6:  Serial.print(F("T6: ")); break;
	case 7:  Serial.print(F("All")); break;
	default: Serial.print(F("T?")); break;
	}
	if (type != 7) {
		char buf[8];
		snprintf(buf, sizeof(buf), "%06X ", addr);
		Serial.print(buf);
	}
}

static const char *Sfrm[4] = { "RR", "RNR", "REJ", "SREJ" };

static const char *Ufrm[2][32] = {
	{"UI", "SIM", "0x02", "SARM", "UP", "0x05", "0x06", "SABM",
	 "DISC", "0x09", "0x0a", "SARME", "0x0c", "0x0d", "0x0e", "SABME",
	 "SNRM", "0x11", "0x12", "RSET", "0x14", "0x15", "0x16", "XID",
	 "0x18", "0x19", "0x1a", "SNRME", "TEST", "0x1d", "0x1e", "0x1f"},
	{"UI", "RIM", "0x02", "DM", "0x04", "0x05", "0x06", "0x07",
	 "RD", "0x09", "0x0a", "0x0b", "UA", "0x0d", "0x0e", "0x0f",
	 "0x10", "FRMR", "0x12", "0x13", "0x14", "0x15", "0x16", "XID",
	 "0x18", "0x19", "0x1a", "0x1b", "TEST", "0x1d", "0x1e", "0x1f"}
};

static void outlinkctrl(unsigned char lc, int rep)
{
	Serial.print(F("Frame-"));
	if (lc & 1) {
		if (lc & 2) {
			Serial.print(F("U: "));
			Serial.println(Ufrm[rep][((lc >> 3) & 0x1c) | ((lc >> 2) & 0x3)]);
		} else {
			Serial.print(F("S: Nr: "));
			Serial.print((lc >> 5) & 0x7);
			Serial.print(' ');
			Serial.println(Sfrm[(lc >> 2) & 0x3]);
		}
	} else {
		Serial.print(F("I: Ns: "));
		Serial.print((lc >> 1) & 0x7);
		Serial.print(F(" Nr: "));
		Serial.println((lc >> 5) & 0x7);
	}
}

// --- ACARS registration / country prefix tables (from vdlm2dec outacars.c) ---

static const char *regpre1[] =
    { "C", "B", "F", "D", "2", "I", "P", "M", "G", "Z", "" };
static const char *regpre2[] = {
        "YA", "ZA", "7T", "C3", "D2", "VP", "V2", "LV", "LQ", "EK", "P4", "VH",
            "OE", "4K", "C6",
        "S2", "8P", "EW", "OO", "V3", "TY", "VQ", "A5", "CP", "T9", "E7", "A2",
            "PP", "PR", "PT",
        "PU", "V8", "LZ", "XT", "9U", "XU", "TJ", "D4", "TL", "TT", "CC", "HJ",
            "HK", "D6", "TN",
        "E5", "9Q", "TI", "TU", "9A", "CU", "5B", "OK", "OY", "J2", "J7", "HI",
            "4W", "HC", "SU",
        "YS", "3C", "E3", "ES", "ET", "DQ", "OH", "TR", "C5", "4L", "9G", "SX",
            "J3", "TG", "3X",
        "J5", "8R", "HH", "HR", "HA", "TF", "VT", "PK", "EP", "YI", "EI", "EJ",
            "4X", "6Y", "ZJ", "JY", "Z6",
        "UP", "5Y", "T3", "9K", "EX", "YL", "OD", "7P", "A8", "5A", "HB",
            "LY", "LX", "Z3",
        "5R", "7Q", "9M", "8Q", "TZ", "9H", "V7", "5T", "3B", "XA", "XB", "XC",
            "V6", "ER", "3A",
        "JU", "4O", "CN", "C9", "XY", "XZ", "V5", "C2", "9N", "PH", "PJ", "ZK",
            "ZL", "ZM", "YN", "5U", "LN",
        "AP", "SU", "E4", "HP", "P2", "ZP", "OB", "RP", "SP", "SN", "CR",
            "CS", "A7", "YR",
        "RA", "RF", "V4", "J6", "J8", "5W", "T7", "S9", "HZ", "6V", "6W", "YU", "S7",
            "9L", "9V", "OM",
        "S5", "H4", "6O", "ZS", "ZT", "ZU", "Z8", "EC", "4R", "ST", "PZ", "SE",
            "HB", "YK", "EY",
        "5H", "HS", "5V", "A3", "9Y", "TS", "TC", "EZ", "T2", "5X", "UR", "A6",
            "4U", "CX",
        "YJ", "VN", "7O", "9J", ""
};
static const char *regpre3[] = { "A9C", "A4O", "9XR", "3DC", "" };

static void fixreg(char *reg, const char *add)
{
	char src[8];
	const char *p, *t;
	int i;

	memcpy(src, add, 7);
	src[7] = 0;

	for (p = src; *p == '.'; p++);

	if (strlen(p) >= 4) {
		t = NULL;
		for (i = 0; regpre3[i][0] != 0; i++)
			if (memcmp(p, regpre3[i], 3) == 0) {
				t = p + 3;
				break;
			}
		if (t == NULL)
			for (i = 0; regpre2[i][0] != 0; i++)
				if (memcmp(p, regpre2[i], 2) == 0) {
					t = p + 2;
					break;
				}
		if (t == NULL)
			for (i = 0; regpre1[i][0] != 0; i++)
				if (*p == regpre1[i][0]) {
					t = p + 1;
					break;
				}
		if (t && *t != '-') {
			memcpy(reg, p, t - p);
			reg[t - p] = 0;
			strcat(reg, "-");
			strcat(reg, t);
			reg[8] = 0;
			return;
		}
	}

	strncpy(reg, p, 8);
	reg[8] = 0;
}

static int outacars(unsigned char *txt, int len)
{
	acarsmsg_t msg;
	int i, k;
	unsigned int crc;

	crc = 0;
	for (i = 0; i < len - 1; i++) {
		update_crc(crc, txt[i]);
		txt[i] &= 0x7f;
	}
	if (crc) {
		Serial.println(F("  CRC error"));
		return 0;
	}

	k = 0;
	msg.mode = txt[k]; k++;
	fixreg(msg.reg, (const char *)&txt[k]); k += 7;

	msg.ack = txt[k];
	if (msg.ack == 0x15)
		msg.ack = '!';
	k++;

	msg.label[0] = txt[k]; k++;
	msg.label[1] = txt[k];
	if (msg.label[1] == 0x7f)
		msg.label[1] = 'd';
	k++;
	msg.label[2] = '\0';

	msg.bid = txt[k];
	if (msg.bid == 0)
		msg.bid = ' ';
	k++;

	msg.bs = txt[k]; k++;

	msg.no[0] = '\0';
	msg.fid[0] = '\0';
	msg.txt[0] = '\0';

	if (msg.bs != 0x03) {
		if (msg.mode <= 'Z' && msg.bid <= '9') {
			for (i = 0; i < 4 && k < len - 4; i++, k++)
				msg.no[i] = txt[k];
			msg.no[i] = '\0';
			for (i = 0; i < 6 && k < len - 4; i++, k++)
				msg.fid[i] = txt[k];
			msg.fid[i] = '\0';
		}
		for (i = 0; (k < len - 4); i++, k++)
			msg.txt[i] = txt[k];
		msg.txt[i] = 0;
	}
	msg.be = txt[k];

	Serial.print(F("  ACARS mode="));
	Serial.write(msg.mode);
	Serial.print(F(" reg="));
	Serial.print(msg.reg);
	if (msg.fid[0]) {
		Serial.print(F(" fid="));
		Serial.print(msg.fid);
	}
	Serial.print(F(" label="));
	Serial.print(msg.label);
	Serial.print(F(" bid="));
	Serial.write(msg.bid);
	Serial.print(F(" ack="));
	Serial.write(msg.ack);
	Serial.print(F(" bs="));
	Serial.print((unsigned)msg.bs, HEX);
	if (msg.no[0]) {
		Serial.print(F(" no="));
		Serial.print(msg.no);
	}
	Serial.println();

	if (msg.txt[0]) {
		Serial.print(F("  Message:\n"));
		Serial.print(msg.txt);
		Serial.println();
	}
	if (msg.be == 0x17)
		Serial.println(F("  Block End"));

	return 1;
}

// --- XID parser (ported from vdlm2dec outxid.c / outprivategr, GPL-2.0) ---

static void getlatlon(unsigned char *p, float *lat, float *lon)
{
	short slat, slon;

	slat = (((short)p[0]) << 8) | (unsigned short)(p[1] & 0xf0);
	slon = (((short)(p[1] & 0x0f)) << 12) | ((unsigned short)(p[2]) << 4);

	*lat = (float)slat / 160.0f;
	*lon = (float)slon / 160.0f;
}

static void outprivategr(unsigned char *p, int len)
{
	int i;

	i = 0;
	do {
		short glen = p[i + 1];
		char buf[64];

		switch (p[i]) {
		case 0:
			break;
		case 0x01:
			Serial.print(F("Connection management: "));
			if (p[i + 2] & 1)
				Serial.print(F("HO; "));
			else if (p[i + 2] & 2)
				Serial.print(F("LCR; "));
			else
				Serial.print(F("LE; "));
			if (p[i + 2] & 4)
				Serial.print(F("GDA; "));
			else
				Serial.print(F("VDA; "));
			if (p[i + 2] & 8)
				Serial.println(F("ESS"));
			else
				Serial.println(F("ESN"));
			break;
		case 0x02:
			Serial.print(F("Signal quality "));
			Serial.print(p[i + 2], DEC);
			Serial.println(F("%"));
			break;
		case 0x03:
			Serial.print(F("XID sequencing "));
			Serial.print(p[i + 2] >> 4, DEC);
			Serial.print(':');
			Serial.println(p[i + 2] & 0x7, DEC);
			break;
		case 0x04:
			Serial.print(F("Specific options: "));
			if (p[i + 2] & 1)
				Serial.print(F("GDA; "));
			else
				Serial.print(F("VDA; "));
			if (p[i + 2] & 2)
				Serial.print(F("ESS; "));
			else
				Serial.print(F("ESN; "));
			if (p[i + 2] & 4)
				Serial.print(F("IHS; "));
			else
				Serial.print(F("IHN; "));
			if (p[i + 2] & 8)
				Serial.print(F("BHS; "));
			else
				Serial.print(F("BHN; "));
			if (p[i + 2] & 0x10)
				Serial.println(F("BCS"));
			else
				Serial.println(F("BCN"));
			break;
		case 0x05:
			Serial.print(F("Expedited subnetwork connection 0x"));
			Serial.println(p[i + 2], HEX);
			break;
		case 0x06:
			Serial.print(F("LCR cause 0x"));
			Serial.println(p[i + 2], HEX);
			break;
		case 0x81:
			Serial.print(F("Modulation support 0x"));
			Serial.println(p[i + 2], HEX);
			break;
		case 0x82: {
			unsigned int addr;
			int n = 0;

			Serial.print(F("Acceptable alternative ground stations: "));
			while (n < p[i + 1]) {
				addr = icaoaddr(&(p[i + 2 + n]));
				snprintf(buf, sizeof(buf), "%06X ", addr & 0xffffff);
				Serial.print(buf);
				n += 4;
			}
			Serial.println();
			break;
		}
		case 0x83: {
			char da[5];
			da[4] = '\0';
			memcpy(da, &(p[i + 2]), 4);
			Serial.print(F("Destination airport: "));
			Serial.println(da);
			break;
		}
		case 0x84: {
			float lat, lon;
			int alt;

			getlatlon(&(p[i + 2]), &lat, &lon);
			alt = p[i + 5] * 1000;
			snprintf(buf, sizeof(buf), "Aircraft position: %5.1f %6.1f ",
				 (double)lat, (double)lon);
			Serial.print(buf);
			if (alt == 0)
				Serial.println(F(" alt: <=999"));
			else if (alt == 255000)
				Serial.println(F(" alt: >=255000"));
			else {
				Serial.print(F(" alt: "));
				Serial.println(alt, DEC);
			}
			break;
		}
		case 0xc0: {
			unsigned int addr, mod, freq;
			int n = 0;

			Serial.print(F("Frequency support: "));
			while (n < p[i + 1]) {
				mod = ((uint32_t)(p[i + 2 + n]) & 0xf0) >> 4;
				freq = ((uint32_t)(p[i + 2 + n]) & 0x0f) << 8 |
				    (uint32_t)(p[i + 3 + n]);
				addr = icaoaddr(&(p[i + 4 + n]));
				snprintf(buf, sizeof(buf), "%03.2f (%01X) %06X ",
					 (double)(freq + 10000) / 100.0,
					 mod & 0x0f, addr & 0xffffff);
				Serial.print(buf);
				n += 6;
			}
			Serial.println();
			break;
		}
		case 0xc1: {
			char id[5];
			int n = 0;

			id[4] = '\0';
			Serial.print(F("Airport coverage: "));
			while (n < p[i + 1]) {
				memcpy(id, &(p[i + 2 + n]), 4);
				Serial.print(id);
				Serial.print(' ');
				n += 4;
			}
			Serial.println();
			break;
		}
		case 0xc3: {
			char id[5];
			id[4] = '\0';
			memcpy(id, &(p[i + 2]), 4);
			Serial.print(F("Nearest airport: "));
			Serial.println(id);
			break;
		}
		case 0xc4: {
			unsigned int adm, ars;

			adm = ((uint32_t)p[i + 2] << 16) |
			    ((uint32_t)p[i + 3] << 8) | (uint32_t)p[i + 4];
			ars = ((uint32_t)p[i + 5] << 16) |
			    ((uint32_t)p[i + 6] << 8) | (uint32_t)p[i + 7];
			snprintf(buf, sizeof(buf), "ATN router nets: ADM %06X ARS %06X",
				 adm, ars);
			Serial.println(buf);
			break;
		}
		case 0xc5: {
			unsigned int mask;

			mask = icaoaddr(&(p[i + 2]));
			snprintf(buf, sizeof(buf), "Station system mask: %06X",
				 mask & 0xffffff);
			Serial.println(buf);
			break;
		}
		case 0xc8: {
			float lat, lon;

			getlatlon(&(p[i + 2]), &lat, &lon);
			snprintf(buf, sizeof(buf), "Station position: %5.1f %6.1f",
				 (double)lat, (double)lon);
			Serial.println(buf);
			break;
		}
		default:
			Serial.print(F("unknown private id 0x"));
			Serial.println(p[i], HEX);
			break;
		}
		i += 2 + glen;
	} while (i < len);
}

static int outxid(unsigned char *p, int len)
{
	int i;
	int dec = 0;

	i = 0;
	do {
		short glen = p[i + 1] * 256 + p[i + 2];

		if (p[i] == 0x80) {
			/* public parameters: skip */
			i += 3 + glen;
			continue;
		}
		if (p[i] == 0xf0) {
			outprivategr(&(p[i + 3]), glen);
			i += 3 + glen;
			dec = 1;
			break;
		}
		Serial.print(F("unknown XID group 0x"));
		Serial.println(p[i], HEX);
		i += 3 + glen;
	} while (i < len);

	return dec;
}

void out(msgblk_t *blk, unsigned char *hdata, int l)
{
	digitalWriteFast(LED_BUILTIN, HIGH);   // valid frame -> LED on (retriggerable)
	ledOn = true;
	ledOffAt = millis() + LED_ON_MS;
	int rep = (hdata[5] & 2) >> 1;
	unsigned int faddr, taddr;
	int dec = 0;

	faddr = icaoaddr(&hdata[5]);
	taddr = icaoaddr(&hdata[1]);

	char buf[64];
	Serial.print(F("\nVDL2 f="));
	Serial.print(blk->Fr);
	Serial.print(F(" ppm="));
	Serial.print(blk->ppm, 2);
	Serial.print(F(" len="));
	Serial.println(l);

	Serial.print(rep ? F("Response ") : F("Command "));
	Serial.print(F("from "));
	outaddr(faddr);
	Serial.print(F("to "));
	outaddr(taddr);
	Serial.println();

	if (l > 9)
		outlinkctrl(hdata[9], rep);

	// XID
	if (l >= 14 && hdata[10] == 0x82) {
		Serial.println(F("  XID:"));
		dec |= outxid(&hdata[11], l - 14);
	}

	// ACARS
	if (l >= 16 && hdata[10] == 0xff && hdata[11] == 0xff && hdata[12] == 1)
		dec |= outacars(&hdata[13], l - 16);

	if (l > 13 && dec == 0) {
		Serial.print(F("  hex:"));
		for (int i = 0; i < l; i++) {
			snprintf(buf, sizeof(buf), " %02x", hdata[i]);
			Serial.print(buf);
		}
		Serial.println();
	}
	Serial.println();
}

extern uint8_t usb_audio_receive_setting;
extern uint8_t usb_audio_transmit_setting;
extern volatile uint32_t usb_audio_underrun_count;
extern volatile uint32_t usb_audio_overrun_count;

static bool statEnabled = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  
	Serial.println(F("=== BOOT (A03-VDL2) ==="));

	AudioMemory(40);

	ch.chn = 0;
	ch.Fr = (int)VDL2FREQ;
	initD8psk(&ch);
	initVdlm2(&ch);

	qI.begin();
	qQ.begin();
}

void loop()
{
	if (qI.available() >= 1 && qQ.available() >= 1) {
		const int16_t *bI = qI.readBuffer();
		const int16_t *bQ = qQ.readBuffer();

		for (int i = 0; i < 128; i++) {
			float l = (float)bI[i] * (1.0f / 32768.0f);
			float r = (float)bQ[i] * (1.0f / 32768.0f);
			resample_and_demod(l, r);
		}

		qI.freeBuffer();
		qQ.freeBuffer();
	}

	if (ledOn && (int32_t)(millis() - ledOffAt) >= 0) {
		ledOn = false;
		digitalWriteFast(LED_BUILTIN, LOW);
	}

	vdlm2_process();

	while (Serial.available() > 0) {
		if (Serial.read() == 's') statEnabled = !statEnabled;
	}

	static unsigned long last = 0;
	if (statEnabled && millis() - last >= 2000) {
		last = millis();
		unsigned long nf, nc, ns;
		vdlm2_stats(&nf, &nc, &ns);
		Serial.print(F("STAT frames="));
		Serial.print(nf);
		Serial.print(F(" crc="));
		Serial.print(nc);
		Serial.print(F(" short="));
		Serial.print(ns);
		Serial.print(F(" mcu="));
		Serial.print(AudioMemoryUsageMax());
		Serial.print(F(" q="));
		Serial.print(qI.available());
		Serial.print('/');
		Serial.print(qQ.available());
		Serial.print(F(" usb="));
		Serial.print(usb_audio_receive_setting);
		Serial.print('/');
		Serial.print(usb_audio_transmit_setting);
		Serial.print(F(" uo="));
		Serial.print(usb_audio_underrun_count);
		Serial.print('/');
		Serial.print(usb_audio_overrun_count);
		Serial.println();
	}
}
