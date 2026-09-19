# Teensy4VDL2 — VDL Mode 2 Decoder for Teensy 4.x

A standalone [VDL Mode 2](https://en.wikipedia.org/wiki/VDL_Mode_2) (VHF
pi/4-D8PSK @ 10500 Bd) decoder running on a Teensy 4.0/4.1, based on the
[vdlm2dec](https://github.com/TLeconte/vdlm2dec) project. It demodulates the
VDL2 air/ground links on e.g. **136.975 MHz** (and nearby VHF channels) from a
44.1 kHz stereo USB audio feed and prints the decoded frames, aircraft position
and ATN router advertisements on the USB Serial console (**115200 baud 8N1**).

## Features

- pi/4-D8PSK demodulator at 10500 Bd with symbol timing recovery
  (resampler 44.1 kHz → 84 kHz)
- Viterbi decoder for the FEC-protected frame header
- Reed-Solomon RS(255,249) error correction
- HDLC bit un-stuffing with CRC-16/FCS validation
- Decoded header lines: link type, addressing, aircraft position,
  ATN router nets/advertisements, XID parameters when present
- Live statistics (frames, CRC drops, short frames, audio and USB counters)
- LED blink on successfully decoded frames
- No display / no network needed — pure headless decoder via USB Serial

## Hardware

- Teensy 4.0 or 4.1
- A VHF AM receiver / SDR feeding the audio into the Teensy via **USB Audio**:

  1. Tune an RTL-SDR (e.g. NooElec NESDR) to **136.975 MHz**, AM mode.
  2. Set the filter bandwidth to about **20 kHz** and turn off the output AGC
     (the demodulator is amplitude-normalized, but sustained clipping of strong
     bursts degrades the signal).
  3. Route SDR++'s audio output to the Teensy's **USB audio input**
     (Windows: set the Teensy as the default output device).

  The Teensy enumerates as a combined USB device: **Serial (CDC)**, **MIDI**
  (reserved, unused) and **Audio** (USB Audio Class, 2-ch 16-bit, L = I, R = Q).

## Wiring

No external wiring is required for the USB audio path. The on-board LED (pin 13)
is used as a frame indicator.

If you prefer an analog feed, connect the audio source (e.g. the headphone out
of a scanner with a suitable level) to the Teensy audio shield / line-in adapter
instead and adapt the sketch accordingly.

## Serial console

Connect a serial terminal (e.g. Arduino IDE Serial Monitor, PuTTY, minicom) to
the Teensy's USB port. **The terminal must be set to 115200 baud, 8 data bits,
no parity, 1 stop bit (115200 8N1).**

| Key | Function |
|-----|----------|
| `s` | Toggle periodic decode statistics (default: off) |

## Example output

Frames are printed as hex dumps together with decoded header lines. The Teensy
has no clock, so it prints no date; a reader terminal can add the UTC receipt
time (see below):

```
VDL2 f=136975000 ppm=1.62 len=180
Command from Aircraft: 20B662 to GroundD: 485F82
Frame-I: Ns: 5 Nr: 5
  ACARS mode=2 reg=PH-BKO fid=KL0877 label=H1 bid=9 ack=! bs=2 no=E79B
  Message:
#EIB8 ;
19 1100;0010;0000;1000;1100;0010;0000;0100
20 0111;1000;0000;0000;0111;1000;0000;0000
21 109.7;109.4
22 ;19;SEP;26;10:06:40

VDL2 f=136975000 ppm=1.60 len=13
Response from Aircraft: 485F82 to GroundD: 20B662
Frame-S: Nr: 6 RR

```

### Statistics fields (key `s`, printed every 2 s)
---
`STAT frames=17 crc=28 short=0 mcu=8 q=0/0 usb=1/0 uo=237640/18`

| Field | Meaning |
|-------|---------|
| `frames` | Successfully decoded frames (valid CRC-16/FCS, printed) |
| `crc` | Frames rejected by the CRC-16/FCS check (decode drops) |
| `short` | Frames aborted before completion (short/truncated) |
| `mcu` | Maximum audio memory used (AudioMemoryUsageMax) |
| `q` | Audio queue fill levels, I/Q |
| `usb` | USB audio receive/transmit settings (current) |
| `uo` | USB Audio underrun / overrun counters |


## Reception tips

- The demodulator is amplitude-blind, so the relative level of strong and weak
  stations matters, not the absolute volume. Leave enough headroom that strong
  bursts do not clip the 16-bit USB audio path.
- If long frames drop out mid-transmission, this is usually reception fading /
  multipath, not a decoding problem — antenna placement is the lever.
- For best sensitivity, maximize signal-to-noise ratio at the front end and
  avoid automatic gain that pumps on strong in-band carriers.

## Credits

- Original decoder: **vdlm2dec** — Copyright (C) [Thierry Leconte](https://github.com/TLeconte/vdlm2dec)
- Teensy 4.x port: Copyright (C) 2026 [Holger Nyga (satdelight)](https://github.com/satdelight)

## License

**GPL-2.0-only** — see the header of `Teensy4VDL2.ino` and the
[GNU General Public License v2](https://www.gnu.org/licenses/gpl-2.0.html).
