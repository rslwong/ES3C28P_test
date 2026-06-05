# ES3C28P sketches

Self-contained Arduino projects for the **ES3C28P** board (ESP32-S3-WROOM-1,
2.8" 240×320 IPS, ILI9341V display + FT6336G capacitive touch + ES8311/FM8002E
audio).

Four sketches (AI generated) are included:

- **`ES3C28P_radio`** *(default)* — internet radio. Connects to WiFi, streams
  MP3 stations, and drives the speaker through the ES8311 codec. On-screen
  touch controls: `PREV`/`NEXT` station, `VOL-`/`VOL+`, a volume bar, and
  `PLAY`/`STOP`. The current track title (ICY metadata) is shown live.
- **`ES3C28P_mic`** — microphone test / real-time spectrum analyzer. Captures
  the on-board mic (ES8311 ADC → I2S), runs a 256-point FFT, and draws a
  log-spaced bar-graph spectrum with peak-hold markers. A level meter and a
  `LISTENING`/`quiet` indicator light up when the mic detects sound, and the
  dominant frequency is shown.
- **`ES3C28P_walkie`** — two-board push-to-talk walkie-talkie over **ESP-NOW**
  (no WiFi router needed). Hold the on-screen `PTT` button to transmit the mic
  as raw 16-bit PCM; release to receive and play incoming audio through the
  speaker. Flash the *same* binary to both boards — no pairing/config needed.
  The screen shows a `TX`/`RX` banner and live packet stats.
- **`ES3C28P_demo`** — graphics + touch showcase (shapes, then a finger-paint
  app with a color palette and `CLEAR` button).

## Layout

```
arduino-cli.yaml          local arduino-cli config (core + libs stay in ./.arduino)
Makefile                  build / upload / monitor targets
ES3C28P_radio/            internet-radio sketch (default)
ES3C28P_mic/              microphone spectrum analyzer
ES3C28P_walkie/           ESP-NOW push-to-talk walkie-talkie (needs 2 boards)
ES3C28P_demo/             graphics + touch demo
.arduino/                 ESP32 core + libraries (created by `make deps`)
build/                    compiled output (created by `make build`)
```

Everything is installed **under this directory** — no global Arduino
libraries or cores are referenced.

## Usage

```sh
make deps      # one-time: download the ESP32 core + libraries into ./.arduino
make flash     # compile + upload the radio (default sketch)
make monitor   # serial monitor @ 115200 (Ctrl-C to exit)
```

`make build` runs `deps` automatically the first time. Pick a sketch with
`SKETCH=`, and override the auto-detected serial port if needed:

```sh
make flash                          # ES3C28P_radio (default)
make flash SKETCH=ES3C28P_demo      # the graphics/touch demo
make flash PORT=/dev/cu.usbmodem1101
make ports                          # list attached boards
```

## Internet radio (ES3C28P_radio)

WiFi credentials and the station list are hard-coded near the top of
[ES3C28P_radio.ino](ES3C28P_radio/ES3C28P_radio.ino) (`WIFI_SSID`/`WIFI_PASS`
and the `STATIONS[]` array — add your own plain-HTTP MP3/AAC stream URLs).

Audio chain: ESP32-S3 I2S → **ES8311** codec (configured over I2C) →
**FM8002E** amp (enable on GPIO1, active-low) → speaker. The ES8311 has no
Arduino driver, so it is initialised with a raw register sequence for
MCLK = 256·fs; volume is handled by the ESP32-audioI2S library (0–21).

Audio GPIO: MCLK=4, BCLK=5, LRCK=7, DOUT=8 (playback), DIN=6 (mic), PA_EN=1.

If you get no sound: the amp-enable polarity (`PA_ON`) and the ES8311 I2C
address (`0x18`) are the first things to check; both are near the top of the
sketch. The serial log shows codec init and stream status.

`make clean` removes `build/`; `make distclean` also removes `./.arduino`.

## Microphone spectrum analyzer (ES3C28P_mic)

```sh
make flash SKETCH=ES3C28P_mic
```

The on-board mic feeds the **ES8311** codec's ADC, which streams 16-bit samples
over I2S (DIN=GPIO6) to the ESP32. The sketch configures the codec for the
analog-mic/ADC path (the playback init from the radio sketch, extended to power
up the ADC), captures 256-sample frames at 16 kHz, runs an in-sketch radix-2
FFT, and renders a log-spaced bar graph (Nyquist = 8 kHz). The speaker amp is
left disabled — this sketch only listens.

If the bars look dead or saturated, tune `MIC_PGA_GAIN` (analog) / `ADC_VOLUME`
(digital), or the `SPEC_MIN_DB` / `NOISE_GATE_DB` thresholds near the top of
[ES3C28P_mic.ino](ES3C28P_mic/ES3C28P_mic.ino).

## Walkie-talkie (ES3C28P_walkie)

```sh
make flash SKETCH=ES3C28P_walkie     # flash BOTH boards with this same binary
```

A half-duplex push-to-talk intercom between two ES3C28P boards using **ESP-NOW**
— a connectionless, peer-to-peer radio protocol that needs no WiFi router and
gives ~1–5 ms latency, exactly the walkie-talkie model. Audio is 16 kHz mono,
sent as **raw 16-bit PCM** (256 kbps — well within ESP-NOW, so no codec is
needed), chunked into 120-sample packets (244 bytes, ~7.5 ms each, ~133/sec) to
stay under the 250-byte ESP-NOW payload cap.

How it works:

- **Hold `PTT` → transmit.** The amp is muted (no feedback), the mic is captured
  (ES8311 ADC → I2S DIN=6), and each chunk is broadcast over ESP-NOW.
- **Release → receive.** The amp turns on, incoming packets land in a small
  jitter buffer, and playback (I2S DOUT=8 → ES8311 DAC → FM8002E amp → speaker)
  starts after a ~30 ms cushion so dropped/late packets don't click. Mono is
  duplicated into both I2S slots.
- The screen shows a big `TX`/`RX` banner, the `PTT` button, and live counters
  (`TX` / `RX` / dropped / ring-overflow). PTT keeps transmitting as long as a
  finger is on the screen, so sliding off the button won't cut you off.

This sketch runs the ES8311 **full-duplex** — one combined register init powers
up both the ADC mic path (from `ES3C28P_mic`) and the DAC playback path (from
`ES3C28P_radio`) at once (16-bit I2S, MCLK = 256·fs).

**Zero config:** packets go to the ESP-NOW broadcast address, so the identical
binary works on both boards with nothing to set up. To lock to a single partner
instead, set `USE_BROADCAST 0` and paste the other board's STA MAC (printed on
serial at boot) into `PEER_MAC`, near the top of
[ES3C28P_walkie.ino](ES3C28P_walkie/ES3C28P_walkie.ino).

If the mic is too quiet/hot, tune `MIC_PGA_GAIN` / `ADC_VOLUME` (same knobs as
the mic sketch). One detail to watch on first run: full-duplex relies on the
`ESP_I2S` library allocating both TX and RX channels from the single
`setPins()` call. Capture is already proven by `ES3C28P_mic`; if playback turns
out silent, that channel allocation is the first place to look.

## Hardware pin map

| Function | GPIO | | Function | GPIO |
|----------|------|-|----------|------|
| TFT SCLK | 12   | | Touch SDA | 16 |
| TFT MOSI | 11   | | Touch SCL | 15 |
| TFT MISO | 13   | | Touch INT | 17 |
| TFT CS   | 10   | | Touch RST | 18 |
| TFT DC   | 46   | | Touch addr | 0x38 |
| TFT RST  | (tied to ESP32-S3 reset) | | | |
| TFT backlight | 45 (active HIGH) | | | |

## Notes

- The FT6336G is read directly over I2C (its register map is trivial and some
  touch libraries reject the "G" silicon on a chip-ID check), so no extra touch
  library is needed.
- If the touch axes come out mirrored, flip `TOUCH_FLIP_X` / `TOUCH_FLIP_Y` near
  the top of [ES3C28P_demo.ino](ES3C28P_demo/ES3C28P_demo.ino).
- If the colors look inverted on your panel, add `tft.invertDisplay(true);` in
  `setup()`.
