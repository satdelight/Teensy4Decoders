/*
 *  Copyright (c) 2016 Thierry Leconte
 *
 *  This code is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Library General Public License version 2
 *  published by the Free Software Foundation.
 *
 *  SPDX-License-Identifier: GPL-2.0-only
 *
 *  Single-core Teensy 4.x port by Holger Nyga (satdelight), 2026.
 *  The reference blk_thread() producer/consumer pair is replaced by a
 *  synchronous queue: decodeVdlm2() (called from the demodulator in the
 *  audio data path) drops a block into a static 3-buffer pool and
 *  vdlm2_process() drains it in loop().
 */
#include <string.h>
#include "vdlm2.h"

#define PPPINITFCS16    0xffff	/* Initial FCS value */
#define PPPGOODFCS16    0xf0b8	/* Good final FCS value */

static msgblk_t blkpool[3];
static msgblk_t *pendring[2];
static int npending;	/* number of blocks waiting in pendring */
static int prd, pwr;	/* ring read / write index */
static int blkidx;

static unsigned long nframes, ncrc, nshort;

static void check_frame(msgblk_t * blk, unsigned char *hdata, int l)
{
	int i;
	unsigned short crc;

	if (l < 13) {
		nshort++;
		return;
	}

	/* crc */
	crc = PPPINITFCS16;
	for (i = 1; i < l - 1; i++) {
		update_crc(crc, hdata[i]);
	}
	if (crc != PPPGOODFCS16) {
		ncrc++;
		return;
	}

	nframes++;
	out(blk, hdata, l);
}

static int set_eras(int *eras_pos, int nb)
{
	int nbera = 0;

	if (nb <= 67) {
		nbera = 2;
		eras_pos[0] = 253;
		eras_pos[1] = 254;
	}
	if (nb <= 30) {
		nbera = 4;
		eras_pos[0] = 251;
		eras_pos[1] = 252;
		eras_pos[2] = 253;
		eras_pos[3] = 254;
	}

	return nbera;
}

void vdlm2_process(void)
{
	msgblk_t *blk;
	int i, n, k, r, s, t;
	static unsigned char hdata[65 * 249];
	int nbera;
	int eras_pos[6];

	if (npending == 0)
		return;

	blk = pendring[prd];
	prd = (prd + 1) % 2;
	npending--;

	k = s = t = 0;
	hdata[k] = 0;
	for (r = 0; r < blk->nbrow; r++) {
		int by;

		if (r == blk->nbrow - 1) {
			by = blk->nlbyte;
			nbera = set_eras(eras_pos, by);
		} else {
			by = 249;
			nbera = 0;
		}

		/* reed solomon FEC */
		rs(blk->data[r], eras_pos, nbera);

		/* HDLC bit un stuffing */
		for (i = 0; i < by; i++) {

			for (n = 0; n < 8; n++) {
				if (blk->data[r][i] & (1 << n)) {
					hdata[k] |= 1 << s;
					t++;
				} else {
					if (t == 5) {
						t = 0;
						continue;
					}
					t = 0;
				}
				s++;
				if (s == 8) {
					s = 0;
					if (hdata[k] == 0x7e) {
						if (k == 0) {
							k++;
							hdata[k] = 0;
						} else if (k == 1) {
							hdata[1] = 0;
						} else if (k > 1) {
							check_frame(blk, hdata, k + 1);
							k++;
							hdata[k] = 0;
						}
					} else if (k > 0) {
						k++;
						hdata[k] = 0;
					}
				}
			}
		}

	}
}

int initVdlm2(channel_t * ch)
{
	ch->state = WSYNC;
	memset(blkpool, 0, sizeof(blkpool));
	memset(pendring, 0, sizeof(pendring));
	npending = 0;
	prd = pwr = 0;
	blkidx = 0;
	nframes = ncrc = nshort = 0;
	ch->blk = &blkpool[0];
	ch->blk->chn = ch->chn;
	ch->blk->Fr = ch->Fr;

	return 0;
}

void stopVdlm2(void)
{
}

int vdlm2_pending(void)
{
	return npending > 0;
}

void vdlm2_stats(unsigned long *frames, unsigned long *crcbad,
		 unsigned long *shortf)
{
	*frames = nframes;
	*crcbad = ncrc;
	*shortf = nshort;
}

void decodeVdlm2(channel_t * ch)
{
	msgblk_t *b;

	if (npending == 2) {
		/* ring full: loop() must drain faster than bursts arrive:
		   drop the block rather than corrupting the waiting ring */
		return;
	}

	pendring[pwr] = ch->blk;
	pwr = (pwr + 1) % 2;
	npending++;

	b = &blkpool[(blkidx + 1) % 3];
	if (pendring[0] == b || pendring[1] == b)
		b = &blkpool[(blkidx + 2) % 3];

	ch->blk = b;
	blkidx = b - blkpool;
	ch->blk->chn = ch->chn;
	ch->blk->Fr = ch->Fr;
}