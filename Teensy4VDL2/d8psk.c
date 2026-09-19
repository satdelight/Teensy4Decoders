/*
 *  Copyright (c) 2017 Thierry Leconte
 *
 *  This code is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Library General Public License version 2
 *  published by the Free Software Foundation.
 *
 *  SPDX-License-Identifier: GPL-2.0-only
 *
 *  Ported to Teensy 4.x (single core, vcplx instead of complex float)
 *  by Holger Nyga (satdelight), 2026.
 */
#include <stdlib.h>
#include <math.h>
#include "vdlm2.h"
#include "d8psk.h"

int initD8psk(channel_t * ch)
{
	ch->ink = 0;
	ch->Phidx = 0;
	ch->df = 0;
	ch->perr = 100;
	ch->P1 = 0;

	return 0;
}

unsigned int reversebits(const unsigned int bits, const int n)
{
	int i;
	unsigned int in, out;

	in = bits;
	out = 0;
	for (i = 0; i < n; i++) {
		out <<= 1;
		out |= in & 1;
		in >>= 1;
	}
	return out;
}

static inline float descrambler(channel_t * ch, const float V)
{
	int b;

	b = (ch->scrambler ^ (ch->scrambler >> 14)) & 1;
	ch->scrambler = (ch->scrambler << 1) | b;
	if (b)
		return (1.0 - V);
	else
		return V;

}

static void putbit(channel_t * ch, const float SV)
{
	float V;

	V = descrambler(ch, SV);

	switch (ch->state) {
	case WSYNC:
		return;
	case GETHEAD:
		{
			unsigned int bits, len;

			if (ch->nbits < 3)
				V = 0;
			viterbi_add(V, ch->nbits);
			ch->nbits++;
			if (ch->nbits < 25)
				return;

			viterbi_end(&bits);

			bits >>= 5;	/* remove FEC */

			len = reversebits(bits, 17);

			ch->nbrow = ch->blk->nbrow = len / 1992 + 1;
			ch->nlbyte = ch->blk->nlbyte = (len % 1992 + 7) / 8;

			if (len < 12 * 8) {
				ch->state = WSYNC;
				return;
			}

			if (ch->blk->nbrow > 8) {
				ch->state = WSYNC;
				return;
			}

			ch->state = GETDATA;
			ch->nrow = ch->nbyte = 0;
			ch->nbits = ch->bits = 0;

			return;
		}
	case GETDATA:
		{
			if (V > 0.5) {
				ch->bits |= 1 << ch->nbits;
			}

			ch->nbits++;
			if (ch->nbits < 8)
				return;

			ch->blk->data[ch->nrow][ch->nbyte] = ch->bits;

			ch->nbits = 0;
			ch->bits = 0;
			ch->nrow++;

			if (ch->nrow == ch->nbrow) {
				ch->nrow = 0;
				ch->nbyte++;
			}

			if (ch->nlbyte)
				while (ch->nrow == ch->nbrow - 1
				       && ch->nbyte >= ch->nlbyte
				       && ch->nbyte < 249) {
					ch->blk->data[ch->nrow][ch->nbyte] = 0;
					ch->nrow = 0;
					ch->nbyte++;
				}

			if (ch->nbyte == 249) {
				ch->state = GETFEC;
				ch->nrow = ch->nbyte = 0;

				if (ch->nlbyte <= 2) {
					ch->nlbyte = 0;
					ch->nbrow--;
				} else if (ch->nlbyte <= 30)
					ch->nlbyte = 2;
				else if (ch->nlbyte <= 67)
					ch->nlbyte = 4;
				else
					ch->nlbyte = 0;

			}
			return;
		}
	case GETFEC:
		{
			if (V > 0.5) {
				ch->bits |= 1 << ch->nbits;
			}

			ch->nbits++;
			if (ch->nbits < 8)
				return;

			ch->blk->data[ch->nrow][ch->nbyte + 249] = ch->bits;

			ch->nbits = 0;
			ch->bits = 0;
			ch->nrow++;

			if (ch->nrow == ch->nbrow) {
				ch->nrow = 0;
				ch->nbyte++;
			}

			if (ch->nlbyte)
				while (ch->nrow == ch->nbrow - 1
				       && ch->nbyte >= ch->nlbyte
				       && ch->nbyte < 6) {
					ch->blk->data[ch->nrow][ch->nbyte +
								249] = 0;
					ch->nrow = 0;
					ch->nbyte++;
				}

			if (ch->nbyte == 6) {

				decodeVdlm2(ch);

				ch->state = WSYNC;
			}
			return;
		}
	}

}

static inline void putgreycode(channel_t * ch, const float v)
{
	int i = (int) roundf(128.0 * v / M_PI + 128.0);
	putbit(ch, Grey1[i]);
	putbit(ch, Grey2[i]);
	putbit(ch, Grey3[i]);
}

static inline float filteredphase(channel_t * ch)
{
	int i, k;
	float sr = 0, si = 0;

	for (i = ch->clk, k = ch->ink; i < MFLTLEN;
	     i = i + 4, k = (k + 1) % MBUFLEN) {
		sr += ch->Inbuff[k].r * mflt[i];
		si += ch->Inbuff[k].i * mflt[i];
	}
	/* phase */
	return atan2f(si, sr);
}

void demodD8psk(channel_t * ch, float re, float im)
{
	float P;

	ch->Inbuff[ch->ink].r = re;
	ch->Inbuff[ch->ink].i = im;
	ch->ink = (ch->ink + 1) % MBUFLEN;

	ch->clk += 4;

	if (ch->state == WSYNC) {

		float fr, M;
		float Pr[NBPH], Pu, Pv;
		float err;
		int l;

		if (ch->clk < 8)
			return;
		ch->clk -= 8;

		P = filteredphase(ch);

		ch->Phidx = (ch->Phidx + 1) % (NBPH * D8DWN);
		ch->Ph[ch->Phidx] = P;

		/* detect sync */

		Pu = 0;
		M = Pv = Pr[0] =
		    ch->Ph[(ch->Phidx + D8DWN) % (NBPH * D8DWN)] - SW[0];
		for (l = 1; l < NBPH; l++) {
			float Pc, Pd;
			Pc = ch->Ph[(ch->Phidx + (l + 1) * D8DWN) %
				    (NBPH * D8DWN)] - SW[l];
			/* unwarp */
			Pd = Pc - Pv;
			Pv = Pc;
			if (Pd > M_PI) {
				Pu -= 2 * M_PI;
			} else if (Pd < -M_PI) {
				Pu += 2 * M_PI;
			}
			Pr[l] = Pc + Pu;
			M += Pr[l];
		}
		M /= NBPH;
		fr = 0;
		for (l = 0; l < NBPH; l++) {
			Pr[l] -= M;
			fr += Pr[l] * (l - (NBPH - 1) / 2);
		}
		fr /= 408;
		err = 0;
		for (l = 0; l < NBPH; l++) {
			float e;
			e = Pr[l] - (l - (NBPH - 1) / 2) * fr;
			err += e * e;
		}

		if (ch->perr < 4.0 && err > ch->perr) {
			float of;

			ch->state = GETHEAD;
			ch->nbits = 0;
			ch->scrambler = 0x4D4B;
			viterbi_init();
			ch->df = ch->pfr;
			ch->blk->ppm = VDL2SYM * ch->df /
			    (2.0 * M_PI * ch->Fr) * 1e6;
			of = 4 * (ch->p2err - 4 * ch->perr +
				  3 * err) / (ch->p2err - 2 * ch->perr + err);
			ch->clk = (int) roundf(of);
			ch->P1 = filteredphase(ch);

			ch->perr = ch->p2err = 500;
		} else {
			ch->p2err = ch->perr;
			ch->perr = err;
			ch->pfr = fr;
		}
	} else {
		float D;

		if (ch->clk < 32)
			return;
		ch->clk -= 32;

		P = filteredphase(ch);

		D = (P - ch->P1) - ch->df;
		if (D > M_PI)
			D -= 2 * M_PI;
		if (D < -M_PI)
			D += 2 * M_PI;

		putgreycode(ch, D);

		ch->P1 = P;
	}
}