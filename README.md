# tvc

Custom thrust-vector-controlled model rocket. See [SPEC.md](SPEC.md) for the full design and
[CLAUDE.md](CLAUDE.md) for build/test commands.

## Wiring note (SPEC.md Section 4.5)

If you are wiring from an older harness or from `legacy/`'s pin maps: the finalized pin map
**moves the RFM95 radio's RST line from pin 24 to pin 9** (freeing pins 24/25 for the second
AS5600 encoder's I2C bus, Wire2) **and puts the HX711 load-cell amp on pins 6 (DOUT) / 7
(SCK)**. `legacy/` configs predate this and conflict with it; SPEC.md Section 4.5 wins.
