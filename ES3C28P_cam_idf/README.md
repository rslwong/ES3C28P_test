# ES3C28P_cam_idf

ESP-IDF build of the Tapo camera viewer (`../ES3C28P_cam` is the Arduino one).
It shows the camera's MJPEG `stream8` (640x360, scaled to 320x176) on the 2.8" panel.

Why ESP-IDF: the Arduino core's lwIP is precompiled with a 5.7 KB TCP window,
which caps the RTSP stream at ~2 Mbit/s (~1 image/s). Here `sdkconfig.defaults`
sets a 64 KB window, so the board takes the camera's full ~6-9 Mbit/s and shows
every clean image the camera produces (~2-3/s).

## Self-contained

Everything lives in this folder; copy it anywhere and build.

```
main/main.c                 the app
main/jpegcheck.c            rejects torn images (see below)
main/glcdfont.h             5x7 font (from Adafruit GFX, BSD)
test/jpegcheck_test.c       host test for jpegcheck.c: `make test`
main/secrets.h.example      camera address + login -> copy to main/secrets.h
components/esp_new_jpeg/    Espressif JPEG decoder 1.0.2 (esp32s3 lib only, vendored)
sdkconfig.defaults          board + network settings
partitions.csv              keeps the radio's saved WiFi (same NVS spot as Arduino)
Makefile                    deps / build / flash / monitor
.esp-idf/, .espressif/      ESP-IDF v5.5.6 + toolchain, created by `make deps` (gitignored)
```

## Use

```
cp main/secrets.h.example main/secrets.h   # fill in the Tapo "Camera Account"
make deps      # one-time, ~4 GB download into this folder
make flash
make monitor   # Ctrl-] to quit
```

WiFi comes from what `ES3C28P_radio` saved on the board. On a fresh board,
add `WIFI_SSID` / `WIFI_PASS` to `secrets.h`. `make flash` in the parent folder
puts the radio back.

The serial log prints throughput and image rates every 5 s:
`rx 7.6 Mbit/s | RTP frames 20/s, JPEGs 6/s, torn 1/s, shown 2.7/s | decode 46 ms`.
"torn" and duplicate images come from the camera (see the notes at the top of
`main/main.c`). A torn image is the top of one picture glued to the bottom of
another, which shows as half the screen flashing a different colour. The
decoder doesn't always notice, so `jpegcheck.c` first confirms that the
compressed data holds exactly as many blocks as the header says. Its verdict
matched libjpeg on all 144 images captured from the camera.
