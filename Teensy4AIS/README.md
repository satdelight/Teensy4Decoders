# Teensy4AIS — AIS (GMSK-9600) Decoder for Teensy 4.x

A standalone AIS (GMSK @ 9600 baud) decoder running on a Teensy 4.0/4.1, based on the [rtl-ais](https://github.com/dgiardini/rtl-ais) project. It demodulates the AIS marine channels **161.975 MHz (A)** and **162.025 MHz (B)** from a 44.1 kHz stereo USB audio feed and prints either the raw NMEA **AIVDM** sentences or decoded plain text on the USB Serial console (**115200 baud 8N1**).

## Features

- GMSK demodulator at 9600 baud with a 44.1 kHz → 48 kHz resampler (stereo: L = channel A, R = channel B)
- Bit sync, HDLC flag detection, bit un-stuffing and CRC-16/FCS frame validation
- AIVDM sentence assembly including multi-part messages
- Plain-text decoder for message types 1–9, 14, 15, 18, 19, 21, 24 and 27 (position, base station, static & voyage, binary with inland RFM10, SAR aircraft, safety text, interrogation, Class B, AtoN, long-range satellite) with `n/a` for unavailable values
- Non-blocking output: decoded data goes through a ring buffer, so a slow terminal can never hang the receiver
- Optional live statistics, input level meter and raw/plain-text toggle — all disabled by default
- Headless: no display, no network needed — pure USB Serial

## Hardware

- Teensy 4.0 or 4.1
- An AIS receiver / SDR feeding the audio into the Teensy via **USB Audio**:

  1. Tune an RTL-SDR (e.g. NooElec NESDR) to **161.975 MHz** (channel A) or **162.025 MHz** (channel B), FM mode, filter bandwidth ~25 kHz.
  2. Turn off the output AGC where possible (the demodulator is amplitude-normalized, but sustained clipping of strong bursts degrades the signal).
  3. Route SDR++'s audio output to the Teensy's **USB audio input** (Windows: set the Teensy as the default output device).

  The Teensy enumerates as a combined USB device: **Serial (CDC)**, **MIDI** (reserved, unused) and **Audio** (USB Audio Class, 2-ch 16-bit, L = channel A, R = channel B).

## Wiring

No external wiring is required for the USB audio path. The DSP core (`receiver.c`, `filter.c`, `protodec.c`, `hmalloc.c`, `callbacks.h`) lies as flat files in this sketch folder (48 kHz calibrated).

If you prefer an analog feed, connect the audio source (e.g. a scanner headphone out at a suitable level) to a Teensy audio shield / line-in adapter instead and adapt the sketch accordingly.

## Serial console

Connect a serial terminal (e.g. Arduino IDE Serial Monitor, PuTTY, minicom) to the Teensy's USB port. **The terminal must be set to 115200 baud, 8 data bits, no parity, 1 stop bit (115200 8N1).**

In the Arduino IDE select **Tools > USB Type > "Serial + MIDI + Audio"** (AudioInputUSB is required).

| Key | Function |
|---|---|
| `s` | Toggle live decode statistics (default: off, printed every 30 s) |
| `l` | Toggle input level meter (default: off, ~1 s, min/max every 5 s) |
| `v` | Toggle plain-text decoding vs. raw NMEA echo (default: off = raw NMEA) |

## Example output

By default the sketch simply echoes the received AIVDM sentences:

```
=== Teensy4AIS: GMSK-9600, 44.1k tap -> 48k DSP ===
Keys: 's' stats 30s, 'l' level, 'v' NMEA plain text
!AIVDM,1,1,,A,339elh5P00PP57rMNOSf4?vr2DT:,0*05
!AIVDM,1,1,,A,839elh0j2d=<d<>Lt1715hQU0000,0*48
```

Pressing `v` switches to decoded plain text:

```
============================================================
Type 3 - moored
  MMSI:     211514560
  SOG:      0.0 kn
  Pos:      51.513761 N 7.007956 E   (Accuracy high)
  COG:      n/a deg   Heading: n/a deg
  Timestamp:55 s
============================================================
Type 5 - Static and voyage data
  MMSI:     211514560   IMO: 0
  Callsign: DC2755
  Name:     CHRISTINE
  Ship type:passenger (53)
  Size:     Length 57 m  Beam 14 m
  ETA:      n/a
  Draught:  n/a m   Destination: 
```

The Teensy has no clock, so it prints no date; a reader terminal can add the UTC receipt time.

### Statistics (key `s`, printed every 30 s)

```
A: recv=17 crc_err=3 badsize=0
```

| Field | Meaning |
|---|---|
| `recv` | Frames decoded with valid CRC-16/FCS (printed) |
| `crc_err` | Frames rejected by the CRC-16/FCS check |
| `badsize` | Frames dropped because of a wrong length |

With `AIS_STEREO = 1` (see `Teensy4AIS.ino`) a second line is printed for channel B.

### Level meter (key `l`, ~1 per second)

```
[LVL-] 42.3  CH=A   min=12.1 max=88.7
```

`LVL+`/`LVL-` indicate whether the peak is above/below a reference; every 5 s the min/max window since the last reset is appended.

## Reception tips

- The demodulator is amplitude-blind, so the relative level of strong and weak stations matters, not the absolute volume. Leave enough headroom that strong bursts do not clip the 16-bit USB audio path.
- If frames drop out mid-burst this is usually reception fading / multipath, not a decoding problem — antenna placement is the lever.
- For best sensitivity, maximize signal-to-noise ratio at the front end and avoid AGC that pumps on strong in-band carriers.
- Tune exactly to 161.975 / 162.025 MHz with ~25 kHz bandwidth; the sketch expects a 44.1 kHz sample rate (resampled to 48 kHz internally).

## Credits

- Original decoder: **rtl-ais / AIS-receiver** — Copyright (C) Ruben Undheim, Heikki Hannikainen, Oh7lzb and Tomi Manninen
- Teensy 4.x port: Copyright (C) 2026 Holger Nyga (satdelight)

## License

**GPL-2.0-or-later** — the sketch and the DSP core are derived from rtl-ais / AIS-receiver and gMFSK, and carry the standard GPL notice in their file headers. The full license text is available at the [GNU General Public License v2](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html).
