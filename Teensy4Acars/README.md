# Teensy4Acars — ACARS MSK Decoder for Teensy 4.x

A standalone [ACARS](https://en.wikipedia.org/wiki/ACARS) (VHF 2400-baud MSK) decoder
running on a Teensy 4.0/4.1, based on the [acarsdec](https://github.com/f00b4r0/acarsdec/)
project. It demodulates the standard 1200/2400 Hz MSK tone pair that ACARS ground
stations and aircraft use on e.g. 131.825 MHz (and nearby VHF channels) and prints the
decoded frames on the USB Serial console (**115200 baud, 8N1**).

## Features

- 2400 baud MSK demodulator (1200 Hz mark / 2400 Hz space) with matched filter
  and PI-PLL carrier tracking
- Odd-parity framing with SYN/SOH/ETX/ETB state machine and CRC-16 check
- ACARS message reassembly into printable text
- Live statistics (bits, sync hits, frames, drop counters, perr histogram)
- Optional raw int16 sample dump for offline analysis
- LED blink on successfully received frames
- No display / no network needed — pure headless decoder via USB Serial

## Hardware

- Teensy 4.0 or 4.1
- A VHF AM receiver / SDR feeding the audio into the Teensy, e.g.:

  **SDR++ → Windows audio → Teensy via USB Audio** (recommended setup):

  1. Tune an RTL-SDR (e.g. NooElec NESDR) to **131.825 MHz**, AM mode.
  2. Set the filter bandwidth to about **12 kHz** and turn off the output AGC
     (the decoder is amplitude-normalized, but sustained clipping of strong
     bursts degrades the signal).
  3. Route SDR++'s audio output to the Teensy's **USB audio input**
     (Windows: set the Teensy as the default output device).

  The Teensy enumerates as a standard USB audio class device; the sketch reads
  the audio stream via `AudioInputUSB` at 44.1 kHz.

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
| `c` | Toggle display of CRC-BAD frames (default: off) |
| `s` | Print statistics |
| `d` | Toggle raw int16 sample dump (44.1 kHz) for offline analysis |

## Example output

Decoded frames are printed in the classic acarsdec full-format style
(`fmt_msg()` from acarsdec's `output.c`). The Ack byte is printed as
`Nak` only when it is the defined protocol value NAK (0x15); any other
value is a raw byte without aviation meaning and is therefore omitted.
The Teensy has no clock, so it prints no date; a reader terminal can
add the UTC receipt time (see below):

```
[#1 (L:-2.5/0.0 E:0) ------------------------------
Mode : 2 Label : _d Id : 1 Nak
Aircraft reg: A6-EFM Flight id: EK9658
No: S33A
EK9658<ETX>
```

```
--- STATS (60.0 s) ---
Bits  : 722400  (12040 bit/s)
Sync  : 12
SOH   : 8
ETX   : 8
OK    : 6
Drop  : perr>3=3  crc=0
perr-Hist (0,1,2,3,4,5..8,9..20,>20): 4 1 1 0 0 1 0 1
```

## Reception tips

- The decoder is amplitude-blind (the phase detector operates on a normalized
  signal), so the relative level of strong and weak stations matters, not the
  absolute volume. Leave enough headroom that strong bursts do not clip the
  16-bit USB audio path.
- If long frames drop out mid-transmission, this is usually reception fading /
  multipath, not a decoding problem — antenna placement is the lever.
- For best sensitivity, maximize signal-to-noise ratio at the front end and
  avoid automatic gain that pumps on strong in-band carriers.

## Credits

- Original decoder: **acarsdec** — Copyright (C) 2007-2025 [Thierry Leconte](https://github.com/TLeconte/acarsdec)
- Maintained fork with MSK/PLL improvements: Copyright (C) 2024-2026 [Thibaut VARENE (f00b4r0)](https://github.com/f00b4r0/acarsdec)
- Teensy 4.x port: Copyright (C) 2025-2026 [Holger Nyga (satdelight)](https://github.com/satdelight)

## License

**GPL-2.0-only** — see the header of `Teensy4Acars.ino` and the
[GNU General Public License v2](https://www.gnu.org/licenses/gpl-2.0.html).
