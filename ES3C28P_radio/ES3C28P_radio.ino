/*
 * ES3C28P_radio  -  Internet radio with on-screen touch controls.
 *
 * Board : ES3C28P  (ESP32-S3-WROOM-1, N16R8 - 16MB flash / 8MB PSRAM, USB-C)
 * Screen: 2.8" IPS 240x320, ILI9341V, SPI
 * Touch : FT6336G capacitive, I2C (addr 0x38)
 * Audio : I2S -> ES8311 codec (I2C addr 0x18) -> FM8002E amp -> speaker
 *
 * Libraries (installed locally under ./.arduino/user/libraries by the Makefile):
 *   - ESP32-audioI2S      (schreibfaul1)  WiFi streaming + decode + I2S out
 *   - Adafruit GFX / ILI9341 / BusIO       display
 * The ES8311 codec is configured directly over I2C (no Arduino driver exists);
 * the register table below is a known-good DAC-playback init for MCLK = 256*fs.
 *
 * Touch UI: PREV / NEXT station, VOL- / VOL+, a volume bar, and PLAY/STOP.
 */

#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include "Audio.h"          // ESP32-audioI2S
#include "secrets.h"        // WiFi credentials (gitignored; see secrets.h.example)

// ---------------------------------------------------------------------------
// Pin map  (see board datasheet / BSP)
// ---------------------------------------------------------------------------
// Display - SPI
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_CS   10
#define TFT_DC   46
#define TFT_RST  -1          // tied to ESP32-S3 reset
#define TFT_BL   45          // backlight, active HIGH

// Shared I2C bus (touch FT6336 @0x38 and codec ES8311 @0x18)
#define I2C_SDA  16
#define I2C_SCL  15
#define TOUCH_INT 17
#define TOUCH_RST 18
#define FT6336_ADDR 0x38

// Audio I2S  (BSP: DOUT/playback = GPIO8, DIN/mic = GPIO6)
#define I2S_MCLK 4
#define I2S_BCLK 5
#define I2S_LRCK 7
#define I2S_DOUT 8
#define PA_EN    1           // FM8002E amplifier enable
#define PA_ON  LOW           // datasheet: low level enables the amp
#define PA_OFF HIGH

#define ES8311_ADDR 0x18

// Flip these to 1 if the touch axes come out mirrored on your panel.
#define TOUCH_FLIP_X 0
#define TOUCH_FLIP_Y 0

// ---------------------------------------------------------------------------
// Objects / layout constants
// ---------------------------------------------------------------------------
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
Audio audio;

const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

// Radio stations (plain-HTTP MP3 streams play most reliably on the ESP32).
struct Station { const char* name; const char* url; };
const Station STATIONS[] = {
  { "SomaFM Groove Salad", "http://ice1.somafm.com/groovesalad-128-mp3" },
  { "SomaFM DEF CON",      "http://ice1.somafm.com/defcon-128-mp3"      },
  { "SomaFM Drone Zone",   "http://ice1.somafm.com/dronezone-128-mp3"   },
  { "SomaFM Indie Pop",    "http://ice1.somafm.com/indiepop-128-mp3"    },
  { "RT HK Radio 1",         "https://rthk.hk/live1.m3u" },
  { "RT HK Radio 2",         "https://rthk.hk/live2.m3u" },
  { "RT HK Radio 3",         "https://rthk.hk/live3.m3u" },
  { "RT HK Radio 4",         "https://rthk.hk/live4.m3u" }
};
const int NUM_STATIONS = sizeof(STATIONS) / sizeof(STATIONS[0]);

const uint8_t VOL_MAX = 21;   // number of steps on the on-screen volume slider

// The on-screen volume drives the ES8311 DAC volume register (0x32) directly.
// That register runs 0x00 (mute) .. 0xFF (0 dB, max) in 0.5 dB steps, so mapping
// the slider here gives far more output than the old fixed 0xBF, while the
// library's digital volume is held at full scale to preserve signal quality.
const uint8_t VOL_REG_MIN = 0x90;  // slider = 1   (~ -55 dB, quietest audible)
const uint8_t VOL_REG_MAX = 0xFF;  // slider = max ( 0 dB, loudest)

// State
int     curStation = 0;
uint8_t volume     = 10;
bool    playing    = false;

// Now-playing text, filled from the audio callback (same task as loop()).
char nowPlaying[160] = "";
volatile bool nowPlayingDirty = false;

// ---------------------------------------------------------------------------
// Simple touch button
// ---------------------------------------------------------------------------
struct Button { int16_t x, y, w, h; const char* label; uint16_t color; };

// Layout
Button btnPrev = {   4, 130, 112, 42, "< PREV", ILI9341_DARKGREY };
Button btnNext = { 124, 130, 112, 42, "NEXT >", ILI9341_DARKGREY };
Button btnVolDn= {   4, 180, 112, 42, "VOL -",  ILI9341_NAVY     };
Button btnVolUp= { 124, 180, 112, 42, "VOL +",  ILI9341_NAVY     };
const int16_t VOLBAR_X = 4,  VOLBAR_Y = 232, VOLBAR_W = 232, VOLBAR_H = 22;
Button btnPlay = {   4, 266, 232, 46, "PLAY",   ILI9341_DARKGREEN};

void drawButton(const Button& b) {
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 6, b.color);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 6, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  int16_t tw = (int16_t)strlen(b.label) * 12;     // ~12px per char at size 2
  tft.setCursor(b.x + (b.w - tw) / 2, b.y + (b.h - 16) / 2);
  tft.print(b.label);
}

bool hit(const Button& b, int16_t x, int16_t y) {
  return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
}

// ---------------------------------------------------------------------------
// FT6336 capacitive touch (direct I2C reads)
// ---------------------------------------------------------------------------
void touchReset() {
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);  delay(10);
  digitalWrite(TOUCH_RST, HIGH); delay(300);
}

bool getTouch(uint16_t *x, uint16_t *y) {
  uint8_t buf[5];
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)FT6336_ADDR, 5) != 5) return false;
  for (int i = 0; i < 5; i++) buf[i] = Wire.read();

  if ((buf[0] & 0x0F) == 0) return false;
  uint16_t rx = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
  uint16_t ry = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];
#if TOUCH_FLIP_X
  rx = SCREEN_W - 1 - rx;
#endif
#if TOUCH_FLIP_Y
  ry = SCREEN_H - 1 - ry;
#endif
  if (rx >= SCREEN_W) rx = SCREEN_W - 1;
  if (ry >= SCREEN_H) ry = SCREEN_H - 1;
  *x = rx; *y = ry;
  return true;
}

// ---------------------------------------------------------------------------
// ES8311 codec  -  known-good DAC-playback init for MCLK = 256 * sample_rate
// ---------------------------------------------------------------------------
void es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool es8311Init() {
  // Probe the codec first.
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ES8311 not found on I2C!");
    return false;
  }
  // reg, value pairs (DAC playback, 256x MCLK). reg 0x32 is the DAC volume.
  static const uint8_t init[][2] = {
    {0x00,0x80},{0x01,0x3F},{0x02,0x00},{0x03,0x10},{0x04,0x10},
    {0x05,0x00},{0x06,0x03},{0x07,0x00},{0x08,0xFF},{0x09,0x0C},
    {0x0A,0x4C},{0x0B,0x00},{0x0C,0x00},{0x0D,0x01},{0x0E,0x02},
    {0x0F,0x00},{0x10,0x1F},{0x11,0x7F},{0x12,0x00},{0x13,0x10},
    {0x14,0x1A},{0x15,0x40},{0x16,0x24},{0x17,0xBF},{0x18,0x00},
    {0x19,0x00},{0x1A,0x00},{0x1B,0x0A},{0x1C,0x6A},
    {0x32,0xBF},{0x37,0x08},{0x44,0x50},
  };
  for (auto &r : init) { es8311Write(r[0], r[1]); delay(1); }
  Serial.println("ES8311 initialised");
  return true;
}

// Apply the on-screen volume (0..VOL_MAX) to the ES8311 DAC volume register.
// Slider 0 mutes; 1..VOL_MAX maps linearly across VOL_REG_MIN..VOL_REG_MAX.
void applyVolume() {
  uint8_t reg;
  if (volume == 0) {
    reg = 0x00;                       // mute
  } else {
    reg = VOL_REG_MIN +
          (uint16_t)(VOL_REG_MAX - VOL_REG_MIN) * volume / VOL_MAX;
  }
  es8311Write(0x32, reg);
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
// Map the current WiFi RSSI to 0..4 signal bars. 0 means not connected.
int wifiBars() {
  if (WiFi.status() != WL_CONNECTED) return 0;
  int rssi = WiFi.RSSI();           // dBm, typically -30 (strong) .. -90 (weak)
  if (rssi >= -55) return 4;
  if (rssi >= -65) return 3;
  if (rssi >= -75) return 2;
  if (rssi >= -85) return 1;
  return 0;                          // connected but very weak
}

// Draw a 4-bar WiFi strength icon. Filled bars use a strength-based colour;
// empty bars are outlined so the user can see how many are missing.
void drawWifiBars() {
  const int16_t nBars   = 4;
  const int16_t barW    = 5;
  const int16_t gap     = 2;
  const int16_t baseY   = 26;        // bottom of the tallest bar
  const int16_t totalW  = nBars * barW + (nBars - 1) * gap;
  const int16_t x0      = SCREEN_W - 8 - totalW;

  int bars = wifiBars();
  uint16_t onColor = bars >= 3 ? ILI9341_GREEN
                   : bars == 2 ? ILI9341_YELLOW
                   : bars == 1 ? ILI9341_ORANGE
                               : ILI9341_RED;

  // clear the icon area first
  tft.fillRect(x0, 6, totalW, 22, ILI9341_MAROON);

  for (int i = 0; i < nBars; i++) {
    int16_t h = 6 + i * 4;           // 6, 10, 14, 18 px tall
    int16_t x = x0 + i * (barW + gap);
    int16_t y = baseY - h;
    if (i < bars)
      tft.fillRect(x, y, barW, h, onColor);
    else
      tft.drawRect(x, y, barW, h, ILI9341_DARKGREY);
  }
}

void drawTitleBar() {
  tft.fillRect(0, 0, SCREEN_W, 34, ILI9341_MAROON);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 9);
  tft.print("Net Radio");
  drawWifiBars();
}

void drawStation() {
  tft.fillRect(0, 40, SCREEN_W, 38, ILI9341_BLACK);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setTextSize(2);
  tft.setCursor(6, 44);
  // clip name to one line (~19 chars at size 2)
  char buf[24];
  strlcpy(buf, STATIONS[curStation].name, sizeof(buf));
  tft.print(buf);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(6, 66);
  tft.printf("Station %d/%d", curStation + 1, NUM_STATIONS);
}

void drawNowPlaying() {
  tft.fillRect(0, 84, SCREEN_W, 40, ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setTextWrap(false);
  // up to two ~39-char lines
  char line[40];
  const char* p = nowPlaying;
  for (int row = 0; row < 2 && *p; row++) {
    int n = 0;
    while (p[n] && n < 39) n++;
    strlcpy(line, p, n + 1);
    tft.setCursor(6, 88 + row * 12);
    tft.print(line);
    p += n;
  }
}

void drawVolumeBar() {
  tft.drawRect(VOLBAR_X, VOLBAR_Y, VOLBAR_W, VOLBAR_H, ILI9341_WHITE);
  int16_t inner = VOLBAR_W - 4;
  int16_t fill  = (int32_t)inner * volume / VOL_MAX;
  tft.fillRect(VOLBAR_X + 2, VOLBAR_Y + 2, fill, VOLBAR_H - 4, ILI9341_GREEN);
  tft.fillRect(VOLBAR_X + 2 + fill, VOLBAR_Y + 2, inner - fill, VOLBAR_H - 4, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  char v[16];
  snprintf(v, sizeof(v), "VOL %u/%u", volume, VOL_MAX);
  tft.setCursor(VOLBAR_X + VOLBAR_W / 2 - 24, VOLBAR_Y + VOLBAR_H + 4);
  tft.fillRect(VOLBAR_X, VOLBAR_Y + VOLBAR_H + 2, VOLBAR_W, 10, ILI9341_BLACK);
  tft.print(v);
}

void drawPlayButton() {
  btnPlay.label = playing ? "STOP" : "PLAY";
  btnPlay.color = playing ? ILI9341_RED : ILI9341_DARKGREEN;
  drawButton(btnPlay);
}

void drawUI() {
  tft.fillScreen(ILI9341_BLACK);
  drawTitleBar();
  drawStation();
  drawNowPlaying();
  drawButton(btnPrev);
  drawButton(btnNext);
  drawButton(btnVolDn);
  drawButton(btnVolUp);
  drawVolumeBar();
  drawPlayButton();
}

// ---------------------------------------------------------------------------
// Radio control
// ---------------------------------------------------------------------------
void startStation(int idx) {
  curStation = (idx + NUM_STATIONS) % NUM_STATIONS;
  nowPlaying[0] = '\0';
  audio.stopSong();
  Serial.printf("Connecting: %s\n", STATIONS[curStation].url);
  playing = audio.connecttohost(STATIONS[curStation].url);
  drawStation();
  drawNowPlaying();
  drawPlayButton();
}

void stopRadio() {
  audio.stopSong();
  playing = false;
  drawPlayButton();
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P internet radio");

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.print("Connecting WiFi");

  // I2C (touch + codec) and touch controller
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  touchReset();

  // Amplifier off until the codec + stream are up (avoids a pop)
  pinMode(PA_EN, OUTPUT);
  digitalWrite(PA_EN, PA_OFF);

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED)
    Serial.printf("WiFi OK  %s\n", WiFi.localIP().toString().c_str());
  else
    Serial.println("WiFi FAILED - check SSID/password");

  // I2S out -> ES8311. Start I2S first so MCLK is running for codec config.
  audio.setPinout(I2S_BCLK, I2S_LRCK, I2S_DOUT, I2S_MCLK);
  audio.setVolume(VOL_MAX);                // run the library at full scale;
                                           // the ES8311 register sets loudness
  es8311Init();
  applyVolume();                           // set codec DAC volume from `volume`
  digitalWrite(PA_EN, PA_ON);              // enable amplifier

  // Receive ICY metadata (stream titles, station name) for the display.
  Audio::audio_info_callback = [](Audio::msg_t m) {
    if (!m.msg) return;
    if (m.e == Audio::evt_streamtitle) {
      strlcpy(nowPlaying, m.msg, sizeof(nowPlaying));
      nowPlayingDirty = true;
    }
    Serial.printf("[audio] %s: %s\n", m.s ? m.s : "?", m.msg);
  };

  drawUI();
  startStation(curStation);                // auto-play the first station
}

void handleTouch() {
  static bool     wasTouched = false;
  static uint32_t lastPress  = 0;
  uint16_t x, y;
  bool touched = getTouch(&x, &y);

  // Edge-triggered: act once on the press-down, never while held. This stops a
  // single (or noisy) touch from being read as a stream of button presses.
  if (!touched || wasTouched || millis() - lastPress < 200) {
    wasTouched = touched;
    return;
  }
  wasTouched = true;
  lastPress  = millis();

  if (hit(btnPrev, x, y))      startStation(curStation - 1);
  else if (hit(btnNext, x, y)) startStation(curStation + 1);
  else if (hit(btnVolDn, x, y)) {
    if (volume > 0) volume--;
    applyVolume();
    drawVolumeBar();
  } else if (hit(btnVolUp, x, y)) {
    if (volume < VOL_MAX) volume++;
    applyVolume();
    drawVolumeBar();
  } else if (hit(btnPlay, x, y)) {
    if (playing) stopRadio();
    else         startStation(curStation);
  }
}

void loop() {
  audio.loop();          // keep the stream flowing + fire callbacks
  handleTouch();

  if (nowPlayingDirty) {
    nowPlayingDirty = false;
    drawNowPlaying();
  }

  // Refresh the WiFi strength bars every couple of seconds. Only redraw when
  // the bar count actually changes to avoid needless SPI traffic / flicker.
  static uint32_t lastWifiDraw = 0;
  static int      lastBars     = -1;
  if (millis() - lastWifiDraw > 2000) {
    lastWifiDraw = millis();
    int bars = wifiBars();
    if (bars != lastBars) {
      lastBars = bars;
      drawWifiBars();
    }
  }
}
