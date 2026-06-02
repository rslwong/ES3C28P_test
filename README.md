# ES3C28P graphics + touch demo

A self-contained Arduino demo for the **ES3C28P** board (ESP32-S3-WROOM-1,
2.8" 240×320 IPS, ILI9341V display + FT6336G capacitive touch).

On boot it runs a short graphics showcase (text, lines, rectangles, circles,
triangles, color bars) then drops into a finger-paint app with a 6-color
palette and a `CLEAR` button.

## Layout

```
arduino-cli.yaml          local arduino-cli config (core + libs stay in ./.arduino)
Makefile                  build / upload / monitor targets
ES3C28P_demo/             the sketch
.arduino/                 ESP32 core + libraries (created by `make deps`)
build/                    compiled output (created by `make build`)
```

Everything is installed **under this directory** — no global Arduino
libraries or cores are referenced.

## Usage

```sh
make deps      # one-time: download the ESP32 core + libraries into ./.arduino
make flash     # compile and upload to the board
make monitor   # serial monitor @ 115200 (Ctrl-C to exit)
```

`make build` runs `deps` automatically the first time. The serial port is
auto-detected; override it if needed:

```sh
make flash PORT=/dev/cu.usbmodem1101
make ports                              # list attached boards
```

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
