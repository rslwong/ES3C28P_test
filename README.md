# ES3C28P sketches

Self-contained Arduino projects for the **ES3C28P** board (ESP32-S3-WROOM-1,
2.8" 240×320 IPS, ILI9341V display + FT6336G capacitive touch + ES8311/FM8002E
audio).

Two sketches are included:

- **`ES3C28P_radio`** *(default)* — internet radio. Connects to WiFi, streams
  MP3 stations, and drives the speaker through the ES8311 codec. On-screen
  touch controls: `PREV`/`NEXT` station, `VOL-`/`VOL+`, a volume bar, and
  `PLAY`/`STOP`. The current track title (ICY metadata) is shown live.
- **`ES3C28P_demo`** — graphics + touch showcase (shapes, then a finger-paint
  app with a color palette and `CLEAR` button).

## Layout

```
arduino-cli.yaml          local arduino-cli config (core + libs stay in ./.arduino)
Makefile                  build / upload / monitor targets
ES3C28P_radio/            internet-radio sketch (default)
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
