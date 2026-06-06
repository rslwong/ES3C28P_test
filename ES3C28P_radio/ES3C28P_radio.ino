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
#include <Preferences.h>    // NVS-backed storage for volume / last station
#include "Audio.h"          // ESP32-audioI2S

// WiFi credentials are now provisioned on-screen and stored in NVS (see the
// "WiFi provisioning" section below). secrets.h is therefore optional: if it is
// present it only seeds the very first boot, after which the saved credentials
// take over. Build still works with no secrets.h at all.
#if __has_include("secrets.h")
  #include "secrets.h"      // optional seed (gitignored; see secrets.h.example)
#endif
#ifndef WIFI_SSID
  #define WIFI_SSID ""
  #define WIFI_PASS ""
#endif

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
#define ES8311_REG_DACVOL 0x32   // DAC volume: 0x00 mute .. 0xFF 0dB, 0.5dB/step

// Screen dims (backlight off) after this long with no touch; any touch wakes it.
#define SCREEN_TIMEOUT_MS 60000*10  // 10 minutes - set to 0 to disable auto-dim

// Flip these to 1 if the touch axes come out mirrored on your panel.
#define TOUCH_FLIP_X 0
#define TOUCH_FLIP_Y 0

// ---------------------------------------------------------------------------
// Objects / layout constants
// ---------------------------------------------------------------------------
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
Audio audio;
Preferences prefs;

const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

// ---------------------------------------------------------------------------
// Theme  -  a modern dark palette in RGB565 (the panel is full 16-bit colour;
// the old ILI9341_* named colours are the harsh 8-colour CGA set). Tweak the
// hex below to reskin the whole UI.
// ---------------------------------------------------------------------------
#define RGB565(r,g,b) ((uint16_t)((((r)&0xF8)<<8)|(((g)&0xFC)<<3)|((b)>>3)))

const uint16_t C_BG         = RGB565(0x0D,0x11,0x17);  // app background, near-black
const uint16_t C_SURFACE    = RGB565(0x16,0x1B,0x22);  // header / panels
const uint16_t C_SURFACE_HI = RGB565(0x25,0x2C,0x38);  // raised (neutral) buttons
const uint16_t C_BORDER     = RGB565(0x32,0x3B,0x48);  // subtle outlines / track
const uint16_t C_TEXT       = RGB565(0xE6,0xED,0xF3);  // primary text
const uint16_t C_TEXT_DIM   = RGB565(0x8B,0x94,0x9E);  // secondary text
const uint16_t C_ACCENT     = RGB565(0x3F,0xB9,0x50);  // primary accent (green)
const uint16_t C_ACCENT_DK  = RGB565(0x23,0x86,0x36);  // accent pressed / PLAY
const uint16_t C_BLUE       = RGB565(0x58,0xA6,0xFF);  // info accent (now playing)
const uint16_t C_AMBER      = RGB565(0xD2,0x99,0x22);  // station / mid signal
const uint16_t C_RED        = RGB565(0xF8,0x51,0x49);  // STOP / weak signal

// Radio stations (plain-HTTP MP3 streams play most reliably on the ESP32).
struct Station { const char* name; const char* url; };
const Station STATIONS[] = {
  { "SomaFM Groove Salad", "http://ice1.somafm.com/groovesalad-128-mp3" },
  { "SomaFM DEF CON",      "http://ice1.somafm.com/defcon-128-mp3"      },
  { "SomaFM Drone Zone",   "http://ice1.somafm.com/dronezone-128-mp3"   },
  { "SomaFM Indie Pop",    "http://ice1.somafm.com/indiepop-128-mp3"    },
  { "RT HK Radio 1",         "https://stm.rthk.hk/radio1" },
  { "RT HK Radio 2",         "https://stm.rthk.hk/radio2" },
  { "RT HK Radio 3",         "https://stm.rthk.hk/radio3" },
  { "RT HK Radio 4",         "https://stm.rthk.hk/radio4" },
  { "57FM",                  "https://listen.57fm.com/lscafe" },
  { "Toronto Cast",          "https://maggie.torontocast.com:8022/stream" }

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

// Backlight / screen-dim state
bool     screenOn    = true;
uint32_t lastTouchMs = 0;

// Volume + station are persisted to NVS, but lazily: a change marks them dirty
// and the actual flash write happens a few seconds later (see loop()), so a
// flurry of VOL +/- taps costs one write, not one per tap.
bool     settingsDirty   = false;
uint32_t settingsDirtyAt = 0;

// Now-playing text, filled from the audio callback (same task as loop()).
char nowPlaying[160] = "";
volatile bool nowPlayingDirty = false;

// ---------------------------------------------------------------------------
// Simple touch button
// ---------------------------------------------------------------------------
struct Button { int16_t x, y, w, h; const char* label; uint16_t color; };

// Layout
Button btnPrev = {   4, 130, 112, 42, "< PREV", C_SURFACE_HI };
Button btnNext = { 124, 130, 112, 42, "NEXT >", C_SURFACE_HI };
Button btnVolDn= {   4, 180, 112, 42, "VOL -",  C_SURFACE_HI };
Button btnVolUp= { 124, 180, 112, 42, "VOL +",  C_SURFACE_HI };
const int16_t VOLBAR_X = 4,  VOLBAR_Y = 232, VOLBAR_W = 232, VOLBAR_H = 22;
Button btnPlay = {   4, 266, 232, 46, "PLAY",   C_ACCENT_DK };

void drawButton(const Button& b) {
  tft.fillRoundRect(b.x, b.y, b.w, b.h, 8, b.color);
  tft.drawRoundRect(b.x, b.y, b.w, b.h, 8, C_BORDER);
  tft.setTextColor(C_TEXT);
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
  es8311Write(ES8311_REG_DACVOL, reg);
}

// ---------------------------------------------------------------------------
// Persisted settings (NVS) and backlight control
// ---------------------------------------------------------------------------
void markSettingsDirty() {
  settingsDirty   = true;
  settingsDirtyAt = millis();
}

// putXxx() skips the write when the stored value is unchanged, so this is cheap
// to call even when nothing actually moved.
void saveSettings() {
  prefs.putUChar("vol", volume);
  prefs.putInt("station", curStation);
  settingsDirty = false;
}

void setBacklight(bool on) {
  digitalWrite(TFT_BL, on ? HIGH : LOW);
  screenOn = on;
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
  uint16_t onColor = bars >= 3 ? C_ACCENT
                   : bars == 2 ? C_AMBER
                   : bars == 1 ? C_AMBER
                               : C_RED;

  // clear the icon area first (must match the title-bar background)
  tft.fillRect(x0, 6, totalW, 22, C_SURFACE);

  for (int i = 0; i < nBars; i++) {
    int16_t h = 6 + i * 4;           // 6, 10, 14, 18 px tall
    int16_t x = x0 + i * (barW + gap);
    int16_t y = baseY - h;
    if (i < bars)
      tft.fillRect(x, y, barW, h, onColor);
    else
      tft.drawRect(x, y, barW, h, C_BORDER);
  }
}

void drawTitleBar() {
  tft.fillRect(0, 0, SCREEN_W, 34, C_SURFACE);
  tft.setTextColor(C_ACCENT);
  tft.setTextSize(2);
  tft.setCursor(8, 9);
  tft.print("Net Radio");
  drawWifiBars();
}

void drawStation() {
  tft.fillRect(4, 40, SCREEN_W - 8, 38, C_BG);  // inset: keep VU edge lanes
  tft.setTextColor(C_TEXT);
  tft.setTextSize(2);
  tft.setCursor(6, 44);
  // clip name to one line (~19 chars at size 2)
  char buf[24];
  strlcpy(buf, STATIONS[curStation].name, sizeof(buf));
  tft.print(buf);
  tft.setTextColor(C_TEXT_DIM);
  tft.setTextSize(1);
  tft.setCursor(6, 66);
  tft.printf("Station %d/%d", curStation + 1, NUM_STATIONS);
}

void drawNowPlaying() {
  tft.fillRect(4, 84, SCREEN_W - 8, 40, C_BG);  // inset: keep VU edge lanes
  tft.setTextColor(C_BLUE);
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
  tft.drawRoundRect(VOLBAR_X, VOLBAR_Y, VOLBAR_W, VOLBAR_H, 4, C_BORDER);
  int16_t inner = VOLBAR_W - 4;
  int16_t fill  = (int32_t)inner * volume / VOL_MAX;
  tft.fillRect(VOLBAR_X + 2, VOLBAR_Y + 2, fill, VOLBAR_H - 4, C_ACCENT);
  tft.fillRect(VOLBAR_X + 2 + fill, VOLBAR_Y + 2, inner - fill, VOLBAR_H - 4, C_SURFACE);
  tft.setTextColor(C_TEXT);
  tft.setTextSize(1);
  char v[16];
  snprintf(v, sizeof(v), "VOL %u/%u", volume, VOL_MAX);
  tft.setCursor(VOLBAR_X + VOLBAR_W / 2 - 24, VOLBAR_Y + VOLBAR_H + 4);
  tft.fillRect(VOLBAR_X, VOLBAR_Y + VOLBAR_H + 2, VOLBAR_W, 10, C_BG);
  tft.print(v);
}

void drawPlayButton() {
  btnPlay.label = playing ? "STOP" : "PLAY";
  btnPlay.color = playing ? C_RED : C_ACCENT_DK;
  drawButton(btnPlay);
}

void drawUI() {
  tft.fillScreen(C_BG);
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
// VU meter  -  two vertical bars in the 4px background margins either side of
// the controls. Left edge = left channel, right edge = right channel. The
// library's getVUlevel() gives an already-smoothed peak (0..255) per channel.
// ---------------------------------------------------------------------------
const int16_t VU_TOP  = 36;                 // just under the title bar
const int16_t VU_BOT  = 310;                // just above the PLAY button
const int16_t VU_SPAN = VU_BOT - VU_TOP;
const int16_t VU_W    = 4;
const int16_t VU_LX   = 0;                   // left-channel lane (x 0..3)
const int16_t VU_RX   = SCREEN_W - VU_W;     // right-channel lane (x 236..239)

// Per-lane drawn state, so each frame only paints the part that changed.
int16_t  vuLH = 0, vuRH = 0;                 // current bar heights, px
uint16_t vuLC = 0, vuRC = 0;                 // current bar colours (0 = none yet)

// Colour by how full the lane is: green low, yellow mid, red near peak.
uint16_t vuColor(int16_t h) {
  if (h > VU_SPAN * 85 / 100) return C_RED;
  if (h > VU_SPAN * 60 / 100) return C_AMBER;
  return C_ACCENT;
}

// Repaint one lane to height h, touching only the rows that moved.
void drawVUlane(int16_t x, int16_t h, int16_t &prevH, uint16_t &prevColor) {
  if (h < 0) h = 0; else if (h > VU_SPAN) h = VU_SPAN;
  uint16_t col = vuColor(h);
  if (col != prevColor) {                    // colour changed: repaint whole lane
    if (h > 0)        tft.fillRect(x, VU_BOT - h, VU_W, h, col);
    if (h < VU_SPAN)  tft.fillRect(x, VU_TOP, VU_W, VU_SPAN - h, C_BG);
    prevColor = col;
  } else if (h > prevH) {                     // grew: fill the new rows on top
    tft.fillRect(x, VU_BOT - h, VU_W, h - prevH, col);
  } else if (h < prevH) {                     // shrank: clear the freed rows
    tft.fillRect(x, VU_BOT - prevH, VU_W, prevH - h, C_BG);
  }
  prevH = h;
}

void updateVU() {
  uint16_t vu = audio.getVUlevel();          // rrrrrrrrllllllll
  int16_t lh = (int32_t)(vu & 0xFF)        * VU_SPAN / 255;
  int16_t rh = (int32_t)((vu >> 8) & 0xFF) * VU_SPAN / 255;
  drawVUlane(VU_LX, lh, vuLH, vuLC);
  drawVUlane(VU_RX, rh, vuRH, vuRC);
}

// Drop both bars to zero (e.g. on STOP or when the screen dims).
void clearVU() {
  drawVUlane(VU_LX, 0, vuLH, vuLC);
  drawVUlane(VU_RX, 0, vuRH, vuRC);
}

// ---------------------------------------------------------------------------
// Radio control
// ---------------------------------------------------------------------------
void startStation(int idx) {
  curStation = (idx + NUM_STATIONS) % NUM_STATIONS;
  markSettingsDirty();
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

// ===========================================================================
// WiFi provisioning
// ---------------------------------------------------------------------------
// The SSID + password are no longer hard-coded. They are entered on the panel
// (scan a list of networks, type the password on an on-screen keyboard) and
// persisted to NVS so the radio reconnects automatically after a reboot.
//
// Flow:
//   boot -> try saved creds (or a one-time secrets.h seed) -> on failure run
//   wifiSetupFlow(): pick network -> type password -> connect -> save.
// A long-press on the title bar re-opens wifiSetupFlow() at any time.
// ===========================================================================
String wifiSsid;     // active credentials; also what we persist to NVS
String wifiPass;

// --- credential storage (NVS, shared "radio" namespace) --------------------
void loadWifiCreds() {
  wifiSsid = prefs.getString("wifi_ssid", "");
  wifiPass = prefs.getString("wifi_pass", "");
}
void saveWifiCreds(const String& ssid, const String& pass) {
  prefs.putString("wifi_ssid", ssid);
  prefs.putString("wifi_pass", pass);
}

// --- edge-detected single tap (for the modal provisioning screens) ---------
// Mirrors handleTouch()'s debounce, but usable standalone inside a blocking
// loop. Returns true once per finger-down.
bool readTap(uint16_t* x, uint16_t* y) {
  static bool     was  = false;
  static uint32_t last = 0;
  if (digitalRead(TOUCH_INT) == HIGH) { was = false; return false; }
  uint16_t tx, ty;
  bool t = getTouch(&tx, &ty);
  if (!t || was || millis() - last < 180) { was = t; return false; }
  was = true; last = millis();
  if (x) *x = tx; if (y) *y = ty;
  return true;
}

// --- connect with on-screen progress ---------------------------------------
bool wifiConnect(const String& ssid, const String& pass, uint32_t timeoutMs) {
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT); tft.setTextSize(2);
  tft.setCursor(10, 110); tft.print("Connecting to:");
  tft.setTextColor(C_ACCENT);
  char b[22]; strlcpy(b, ssid.c_str(), sizeof(b));
  tft.setCursor(10, 140); tft.print(b);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(100);
  WiFi.begin(ssid.c_str(), pass.c_str());

  tft.setTextColor(C_TEXT_DIM); tft.setTextSize(2);
  uint32_t t0 = millis();
  int dots = 0;
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < timeoutMs) {
    delay(250);
    tft.setCursor(10 + dots * 12, 175); tft.print('.');
    if (++dots > 17) { dots = 0; tft.fillRect(10, 175, SCREEN_W - 20, 18, C_BG); }
    Serial.print('.');
  }
  Serial.println();

  bool ok = WiFi.status() == WL_CONNECTED;
  if (ok) {
    Serial.printf("WiFi OK  %s\n", WiFi.localIP().toString().c_str());
    tft.fillScreen(C_BG);
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    tft.setCursor(10, 140); tft.print("WiFi connected");
    delay(700);
  }
  return ok;
}

// --- network picker --------------------------------------------------------
// Scans and lists nearby networks. Returns the chosen SSID, or "" if cancelled.
String wifiPickNetwork() {
  const int16_t ROW_H = 30, LIST_Y = 44, VIS = 7;
  Button bUp  = {   0, 278, 46, 36, "UP",   C_SURFACE_HI };
  Button bDn  = {  48, 278, 46, 36, "DN",   C_SURFACE_HI };
  Button bRes = {  96, 278, 86, 36, "SCAN", C_SURFACE_HI };
  Button bX   = { 184, 278, 56, 36, "X",    C_RED        };

  int n = 0, top = 0;
  bool needScan = true;

  for (;;) {
    if (needScan) {
      needScan = false;
      tft.fillScreen(C_BG);
      tft.fillRect(0, 0, SCREEN_W, 34, C_SURFACE);
      tft.setTextColor(C_ACCENT); tft.setTextSize(2);
      tft.setCursor(8, 9); tft.print("Scanning...");
      WiFi.mode(WIFI_STA);
      WiFi.disconnect(false);
      n = WiFi.scanNetworks();
      if (n < 0) n = 0;
      top = 0;
    }

    // Draw the current page.
    tft.fillRect(0, 34, SCREEN_W, SCREEN_H - 34, C_BG);
    tft.fillRect(0, 0, SCREEN_W, 34, C_SURFACE);
    tft.setTextColor(C_ACCENT); tft.setTextSize(2);
    tft.setCursor(8, 9); tft.printf("WiFi (%d)", n);
    if (n == 0) {
      tft.setTextColor(C_TEXT_DIM); tft.setTextSize(2);
      tft.setCursor(20, 140); tft.print("No networks");
    }
    for (int i = 0; i < VIS; i++) {
      int idx = top + i;
      if (idx >= n) break;
      int16_t y = LIST_Y + i * ROW_H;
      tft.setTextColor(C_TEXT); tft.setTextSize(2);
      char b[16]; strlcpy(b, WiFi.SSID(idx).c_str(), sizeof(b));
      tft.setCursor(6, y + 6); tft.print(b);
      tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
      if (WiFi.encryptionType(idx) != WIFI_AUTH_OPEN) {
        tft.setCursor(SCREEN_W - 58, y + 9); tft.print("LOCK");
      }
      tft.setCursor(SCREEN_W - 28, y + 9); tft.printf("%d", WiFi.RSSI(idx));
      tft.drawFastHLine(0, y + ROW_H - 1, SCREEN_W, C_BORDER);
    }
    drawButton(bUp); drawButton(bDn); drawButton(bRes); drawButton(bX);

    // Wait for input; break out to redraw, or return on a selection.
    for (;;) {
      uint16_t x, y;
      if (readTap(&x, &y)) {
        if (hit(bX, x, y))   return String("");
        if (hit(bRes, x, y)) { needScan = true; break; }
        if (hit(bUp, x, y))  { if (top > 0)        top -= VIS; if (top < 0) top = 0; break; }
        if (hit(bDn, x, y))  { if (top + VIS < n)  top += VIS; break; }
        if (y >= LIST_Y && y < LIST_Y + VIS * ROW_H) {
          int idx = top + (y - LIST_Y) / ROW_H;
          if (idx < n) return WiFi.SSID(idx);
        }
      }
      delay(8);
    }
  }
}

// --- on-screen keyboard ----------------------------------------------------
enum { KB_LOWER, KB_UPPER, KB_SYM };
int kbMode = KB_LOWER;

// Special key codes (>0 means "insert this literal character", incl. ' ').
#define KC_SHIFT -1
#define KC_SYM   -2
#define KC_BKSP  -3
#define KC_OK    -4

struct LiveKey { int16_t x, y, w, h; char label[6]; int code; };
LiveKey kbKeys[48];
int     kbCount = 0;

// Alphabetical grid keyboard. Letters use 7 wide columns (~34px keys) so they
// are easy to hit; digits stay on a permanent 10-key top row. KB_Y0 sits just
// under the password field to give the keys as much height as the panel allows.
const int16_t KB_Y0 = 96, KEY_H = 33, KEY_G = 3;
const int16_t KB_STRIDE = KEY_H + KEY_G;

void kbAddKey(int16_t x, int16_t y, int16_t w, const char* label, int code) {
  LiveKey& k = kbKeys[kbCount++];
  k.x = x; k.y = y; k.w = w; k.h = KEY_H; k.code = code;
  strlcpy(k.label, label, sizeof(k.label));
}

// Lay out a row of single-character keys, centred across the screen.
static void kbCharRow(const char* s, int cols, int16_t y) {
  int n = strlen(s);
  int16_t cell = SCREEN_W / cols;
  int16_t x0   = (SCREEN_W - n * cell) / 2;
  for (int i = 0; i < n; i++) {
    char t[2] = { s[i], 0 };
    kbAddKey(x0 + i * cell, y, cell - KEY_G, t, (int)(unsigned char)s[i]);
  }
}

// Rebuild the live key list for the current kbMode.
void kbBuild() {
  kbCount = 0;
  int16_t y = KB_Y0;

  // Digits are always available on the top row (10 columns).
  kbCharRow("1234567890", 10, y); y += KB_STRIDE;

  if (kbMode == KB_SYM) {
    // Four symbol rows, 7 columns each.
    const char* sym[4] = { "!@#$%^&", "*()-_=+", "[]{};:'", ",.?/~|\\" };
    for (int r = 0; r < 4; r++) { kbCharRow(sym[r], 7, y); y += KB_STRIDE; }
  } else {
    // Letters A-Z (or a-z) in reading order, 7 columns: rows of 7,7,7,5.
    static const char* upper[4] = { "ABCDEFG", "HIJKLMN", "OPQRSTU", "VWXYZ" };
    static const char* lower[4] = { "abcdefg", "hijklmn", "opqrstu", "vwxyz" };
    const char** rows = (kbMode == KB_UPPER) ? upper : lower;
    for (int r = 0; r < 4; r++) { kbCharRow(rows[r], 7, y); y += KB_STRIDE; }
  }

  // Function row: SHIFT, symbol toggle, space, DEL, OK.
  kbAddKey(0,   y, 38, kbMode == KB_UPPER ? "v" : "^",        KC_SHIFT);
  kbAddKey(41,  y, 37, kbMode == KB_SYM   ? "ABC" : "123",    KC_SYM);
  kbAddKey(81,  y, 74, "space",                               ' ');
  kbAddKey(158, y, 40, "DEL",                                 KC_BKSP);
  kbAddKey(201, y, 38, "OK",                                  KC_OK);
}

void kbDraw() {
  // Clear the whole keyboard band first so a mode switch (ABC/123, shift) that
  // changes key positions can't leave ghost pixels of the previous layout.
  tft.fillRect(0, KB_Y0, SCREEN_W, SCREEN_H - KB_Y0, C_BG);
  for (int i = 0; i < kbCount; i++) {
    LiveKey& k = kbKeys[i];
    uint16_t bg = (k.code == KC_OK) ? C_ACCENT_DK
                : (k.code < 0)      ? C_SURFACE
                                    : C_SURFACE_HI;
    tft.fillRoundRect(k.x, k.y, k.w, k.h, 4, bg);
    tft.drawRoundRect(k.x, k.y, k.w, k.h, 4, C_BORDER);
    tft.setTextColor(C_TEXT);
    int len = strlen(k.label);
    if (len <= 1) {
      tft.setTextSize(2);
      tft.setCursor(k.x + (k.w - 12) / 2, k.y + (k.h - 16) / 2);
    } else {
      tft.setTextSize(1);
      tft.setCursor(k.x + (k.w - len * 6) / 2, k.y + (k.h - 8) / 2);
    }
    tft.print(k.label);
  }
}

int kbHit(uint16_t x, uint16_t y) {
  for (int i = 0; i < kbCount; i++) {
    LiveKey& k = kbKeys[i];
    if (x >= k.x && x < k.x + k.w && y >= k.y && y < k.y + k.h) return k.code;
  }
  return 0;
}

// --- password entry --------------------------------------------------------
// Drives the keyboard. Returns true if OK was pressed (pass filled), false if
// the user cancelled (X in the title bar).
bool wifiEnterPassword(const String& ssid, String& pass) {
  pass = "";
  kbMode = KB_LOWER;

  auto drawField = [&]() {
    tft.fillRect(10, 58, SCREEN_W - 20, 26, C_SURFACE);
    tft.setTextColor(C_TEXT); tft.setTextSize(2);
    const char* p = pass.c_str();
    int L = pass.length(), maxc = 18;            // show the trailing chars that fit
    char shown[20];
    strlcpy(shown, L > maxc ? p + (L - maxc) : p, sizeof(shown));
    tft.setCursor(12, 61); tft.print(shown);
  };

  // Header: title, cancel X, network name, password field.
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, SCREEN_W, 34, C_SURFACE);
  tft.setTextColor(C_ACCENT); tft.setTextSize(2);
  tft.setCursor(8, 9); tft.print("Password");
  tft.setTextColor(C_RED);
  tft.setCursor(SCREEN_W - 22, 9); tft.print("X");
  tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
  tft.setCursor(8, 40); tft.print("Network: ");
  char b[24]; strlcpy(b, ssid.c_str(), sizeof(b)); tft.print(b);
  tft.drawRoundRect(8, 56, SCREEN_W - 16, 30, 4, C_BORDER);
  drawField();
  kbBuild(); kbDraw();

  for (;;) {
    uint16_t x, y;
    if (readTap(&x, &y)) {
      if (y < 34 && x > SCREEN_W - 30) return false;      // cancel X
      int c = kbHit(x, y);
      if (c == 0) { /* miss */ }
      else if (c == KC_OK)    return true;
      else if (c == KC_BKSP)  { if (pass.length()) pass.remove(pass.length() - 1); drawField(); }
      else if (c == KC_SHIFT) { kbMode = (kbMode == KB_UPPER) ? KB_LOWER : KB_UPPER; kbBuild(); kbDraw(); }
      else if (c == KC_SYM)   { kbMode = (kbMode == KB_SYM)   ? KB_LOWER : KB_SYM;   kbBuild(); kbDraw(); }
      else if (c > 0) {
        if (pass.length() < 63) pass += (char)c;
        drawField();
        if (kbMode == KB_UPPER) { kbMode = KB_LOWER; kbBuild(); kbDraw(); }  // auto-unshift
      }
    }
    delay(8);
  }
}

// --- top-level interactive provisioning ------------------------------------
// Loops until connected, or returns false if the user cancels the picker.
bool wifiSetupFlow() {
  for (;;) {
    String ssid = wifiPickNetwork();
    if (ssid.length() == 0) return WiFi.status() == WL_CONNECTED;  // cancelled

    String pass;
    if (!wifiEnterPassword(ssid, pass)) continue;                 // back to list

    if (wifiConnect(ssid, pass, 15000)) {
      wifiSsid = ssid; wifiPass = pass;
      saveWifiCreds(ssid, pass);
      return true;
    }
    // Failed: let the user retry.
    tft.fillScreen(C_BG);
    tft.setTextColor(C_RED); tft.setTextSize(2);
    tft.setCursor(10, 120); tft.print("Connect failed");
    tft.setTextColor(C_TEXT_DIM); tft.setTextSize(1);
    tft.setCursor(10, 150); tft.print("Tap to try again");
    uint16_t tx, ty;
    while (!readTap(&tx, &ty)) delay(10);
  }
}

// Re-open provisioning from the running radio (long-press the title bar).
void openWifiSetup() {
  stopRadio();
  digitalWrite(PA_EN, PA_OFF);
  setBacklight(true);
  bool ok = wifiSetupFlow();
  if (!ok && wifiSsid.length())               // cancelled: restore prior link
    ok = wifiConnect(wifiSsid, wifiPass, 15000);
  drawUI();
  lastTouchMs = millis();
  if (ok) {
    digitalWrite(PA_EN, PA_ON);
    startStation(curStation);
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P internet radio");

  // Restore persisted volume + last station (defaults if never saved).
  prefs.begin("radio", false);
  volume     = prefs.getUChar("vol", volume);
  curStation = prefs.getInt("station", curStation);
  if (volume > VOL_MAX)                       volume     = VOL_MAX;
  if (curStation < 0 || curStation >= NUM_STATIONS) curStation = 0;

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(0);
  // This IPS ILI9341V glass needs colour inversion ON, otherwise everything
  // shows as its complement (dark background comes out white). Flip to false
  // if a future panel looks inverted the other way.
  tft.invertDisplay(true);
  tft.fillScreen(C_BG);
  tft.setTextColor(C_TEXT);
  tft.setTextSize(2);
  tft.setCursor(10, 140);
  tft.print("Connecting WiFi");

  // I2C (touch + codec) and touch controller
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  touchReset();
  // Put the FT6336 in polling mode (G_MODE reg 0xA4 = 0) so its INT line stays
  // low for the whole time a finger is down -- we gate the I2C reads on it.
  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0xA4);
  Wire.write(0x00);
  Wire.endTransmission();
  pinMode(TOUCH_INT, INPUT_PULLUP);

  // Amplifier off until the codec + stream are up (avoids a pop)
  pinMode(PA_EN, OUTPUT);
  digitalWrite(PA_EN, PA_OFF);

  // WiFi: try the saved credentials (NVS), falling back to a one-time seed from
  // secrets.h on first boot, and finally to the on-screen scan + keyboard UI.
  WiFi.mode(WIFI_STA);
  loadWifiCreds();
  if (wifiSsid.length() == 0 && strlen(WIFI_SSID)) {
    wifiSsid = WIFI_SSID;                 // seed from secrets.h, not yet saved
    wifiPass = WIFI_PASS;
  }
  bool wifiOk = false;
  if (wifiSsid.length())
    wifiOk = wifiConnect(wifiSsid, wifiPass, 15000);
  if (wifiOk)
    saveWifiCreds(wifiSsid, wifiPass);    // persist confirmed credentials
  else
    wifiOk = wifiSetupFlow();             // scan + type the password on-panel
  if (!wifiOk)
    Serial.println("WiFi not configured");

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
  lastTouchMs = millis();                   // start the screen-dim timer
  startStation(curStation);                // auto-play the first/last station
}

void handleTouch() {
  static bool     wasTouched = false;
  static uint32_t lastPress  = 0;

  // Long-press the title bar (top 34px) for ~1.5s to re-open WiFi setup. This
  // is the only way to change networks once credentials are saved.
  static bool     holding    = false;
  static uint32_t holdStart  = 0;
  {
    uint16_t hx, hy;
    bool inTitle = digitalRead(TOUCH_INT) == LOW && getTouch(&hx, &hy) && hy < 34;
    if (inTitle) {
      if (!holding) { holding = true; holdStart = millis(); }
      else if (millis() - holdStart > 1500) {
        holding = wasTouched = false;
        openWifiSetup();
        return;
      }
    } else holding = false;
  }

  // The FT6336 holds INT low only while a finger is on the panel. When it is
  // high there is nothing to report, so skip the I2C transaction entirely.
  if (digitalRead(TOUCH_INT) == HIGH) {
    wasTouched = false;
    return;
  }

  uint16_t x, y;
  bool touched = getTouch(&x, &y);

  // Edge-triggered: act once on the press-down, never while held. This stops a
  // single (or noisy) touch from being read as a stream of button presses.
  if (!touched || wasTouched || millis() - lastPress < 200) {
    wasTouched = touched;
    return;
  }
  wasTouched  = true;
  lastPress   = millis();
  lastTouchMs = lastPress;

  // First touch after the screen dimmed just wakes it -- don't also fire a button.
  if (!screenOn) {
    setBacklight(true);
    return;
  }

  if (hit(btnPrev, x, y))      startStation(curStation - 1);
  else if (hit(btnNext, x, y)) startStation(curStation + 1);
  else if (hit(btnVolDn, x, y)) {
    if (volume > 0) volume--;
    applyVolume();
    drawVolumeBar();
    markSettingsDirty();
  } else if (hit(btnVolUp, x, y)) {
    if (volume < VOL_MAX) volume++;
    applyVolume();
    drawVolumeBar();
    markSettingsDirty();
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

  // VU meter: animate while playing on a lit screen (~25 fps); otherwise make
  // sure the bars are cleared so they don't linger after STOP / dim.
  static uint32_t lastVU = 0;
  if (playing && screenOn) {
    if (millis() - lastVU >= 40) {
      lastVU = millis();
      updateVU();
    }
  } else if (vuLH || vuRH) {
    clearVU();
  }

  // Dim the backlight after a spell of no touches; handleTouch() wakes it again.
  if (screenOn && millis() - lastTouchMs > SCREEN_TIMEOUT_MS) setBacklight(false);

  // Flush pending volume/station changes to NVS once they've settled.
  if (settingsDirty && millis() - settingsDirtyAt > 3000) saveSettings();

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
