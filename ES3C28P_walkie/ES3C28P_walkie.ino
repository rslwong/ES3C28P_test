/*
 * ES3C28P_walkie  -  Two-board push-to-talk walkie-talkie over ESP-NOW.
 *
 * Board : ES3C28P  (ESP32-S3-WROOM-1, N16R8 - 16MB flash / 8MB PSRAM, USB-C)
 * Screen: 2.8" IPS 240x320, ILI9341V, SPI
 * Touch : FT6336G capacitive, I2C (addr 0x38)   <- used as the PTT button
 * Audio : analog MIC -> ES8311 ADC -> I2S DIN(6)   (capture / TX)
 *         I2S DOUT(8) -> ES8311 DAC -> FM8002E amp -> speaker  (playback / RX)
 *
 * What it does
 * ------------
 * Hold the on-screen PTT button to TRANSMIT: the mic is captured at 16 kHz mono
 * and streamed as raw 16-bit PCM in ESP-NOW packets. Release to RECEIVE: incoming
 * packets are queued in a small jitter buffer and played out the speaker. Flash
 * the SAME binary to both boards - no pairing/config needed, because packets go
 * to the ESP-NOW broadcast address (see PEER_MAC below to lock to one peer).
 *
 * Why ESP-NOW (vs WiFi/UDP)
 * -------------------------
 * Connectionless, no router/AP, ~1-5 ms latency - exactly the walkie-talkie
 * model. The only constraint is the 250-byte payload cap, so audio is chunked
 * into 120-sample packets (244 bytes, ~7.5 ms of audio each, ~133 packets/sec).
 * 16 kHz x 16-bit = 256 kbps raw, well within ESP-NOW - no codec needed.
 *
 * Signal chain / codec
 * --------------------
 * The ESP32 is the I2S master (MCLK4/BCLK5/LRCK7), full-duplex: it reads ADC data
 * on DIN(6) and writes DAC data on DOUT(8) on the same bus. The ES8311 has no
 * Arduino driver, so it is configured with a raw register sequence that powers up
 * BOTH the ADC/analog-mic path (from ES3C28P_mic) AND the DAC playback path (from
 * ES3C28P_radio) at once - 16-bit I2S, MCLK = 256*fs. The codec is mono but uses
 * one slot of a stereo I2S frame, so we capture stereo + keep the mic slot, and
 * duplicate mono into both slots on playback.
 *
 * NOTE (untested): full-duplex relies on ESP_I2S allocating both TX and RX
 * channels when setPins() is given both DOUT and DIN. If playback is silent but
 * capture works, that channel allocation is the first place to look.
 *
 * Libraries (installed locally under ./.arduino/user/libraries by the Makefile):
 *   - Adafruit GFX / ILI9341 / BusIO   display
 *   - ESP_I2S, WiFi, esp_now           bundled with the ESP32 core
 *
 * Build:  make flash SKETCH=ES3C28P_walkie
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ESP_I2S.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Pin map  (identical to ES3C28P_radio / ES3C28P_mic - see board BSP)
// ---------------------------------------------------------------------------
// Display - SPI
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_CS   10
#define TFT_DC   46
#define TFT_RST  -1          // tied to ESP32-S3 reset
#define TFT_BL   45          // backlight, active HIGH

// Shared I2C bus (touch FT6336 @0x38, codec ES8311 @0x18)
#define I2C_SDA  16
#define I2C_SCL  15
#define TOUCH_INT 17
#define TOUCH_RST 18
#define FT6336_ADDR 0x38
#define ES8311_ADDR 0x18

// Audio I2S  (BSP: MCLK=4, BCLK=5, LRCK=7, DOUT/playback=8, DIN/mic=6)
#define I2S_MCLK 4
#define I2S_BCLK 5
#define I2S_LRCK 7
#define I2S_DIN  6           // ADC data from codec into ESP32 (TX/capture)
#define I2S_DOUT 8           // DAC data from ESP32 into codec  (RX/playback)
#define PA_EN    1           // FM8002E amplifier enable
#define PA_ON  LOW           // datasheet: low level enables the amp
#define PA_OFF HIGH

// Flip these to 1 if the touch axes come out mirrored on your panel.
#define TOUCH_FLIP_X 0
#define TOUCH_FLIP_Y 0

// ---------------------------------------------------------------------------
// Audio / transport settings
// ---------------------------------------------------------------------------
#define SAMPLE_RATE      8000    // Hz, mono. Telephone quality, 128 kbps raw.
                                 // Lowered from 16 kHz to fit the Long-Range PHY's
                                 // airtime budget (see USE_LONG_RANGE below).
#define SAMPLES_PER_PKT  120     // 120 * 2 bytes = 240B payload (<=250 ESP-NOW)
#define PKT_MAGIC        0xA5

// ES8311 mic gain knobs (carried over from ES3C28P_mic - raise if mic is quiet).
#define MIC_PGA_GAIN 0x24        // reg 0x16 - analog PGA stage
#define ADC_VOLUME   0xE6        // reg 0x17 - digital ADC volume (0..255)

// DAC playback volume (reg 0x32, 0..255, ~0.5 dB/step). This is the speaker
// loudness knob and is now adjustable at runtime with the on-screen +/- buttons.
// Default raised from the old 0xBF for a noticeably louder starting level.
#define DAC_VOLUME_DEFAULT 0xF0  // ~94% - loud default (was 0xBF)
#define DAC_VOLUME_MAX     0xFF
#define DAC_VOLUME_MIN     0x40  // floor so "min" still produces audible output
#define DAC_VOLUME_STEP    0x10  // per +/- tap

// ESP-NOW peering. Default: broadcast, so the identical binary works on both
// boards with zero configuration. To lock to a single partner, set USE_BROADCAST
// to 0 and put the OTHER board's STA MAC (printed on serial at boot) in PEER_MAC.
#define USE_BROADCAST 1
#define WIFI_CHANNEL  1

// --- Range tuning ---------------------------------------------------------
// TX power: crank the radio to its maximum (~20 dBm). Units are 0.25 dBm, so
// 84 = 21 dBm (driver clamps to the legal max). Pure win, no downside.
#define MAX_TX_POWER  84
// Long-Range PHY (Espressif proprietary): trades data rate for a much lower,
// more robust modulation - can multiply usable range several-fold. BOTH boards
// must run with this enabled. It lowers throughput, so it is paired with a
// lower SAMPLE_RATE above; if you bump the rate back up while LR is on you may
// hear dropouts (watch the "ovf"/"drop" stats). Set to 0 for normal mode.
#define USE_LONG_RANGE 1
//static uint8_t PEER_MAC[6]   = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static uint8_t PEER_MAC[6]   = { 0x14, 0xc1, 0x9f, 0xd0, 0x57, 0x20 };     // old


//static uint8_t PEER_MAC[6]   = { 0x14, 0xc1, 0x9f, 0xd0, 0x58, 0xb8 };      // new



static uint8_t BCAST_MAC[6]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

// Jitter buffer: queue a little audio before playback so dropped/late packets
// don't click. RING_SIZE must be a power of two. ~0.25 s at 16 kHz.
#define RING_SIZE        4096            // int16 samples (must be power of 2)
#define RING_MASK        (RING_SIZE - 1)
#define PREBUFFER        (SAMPLES_PER_PKT * 4)   // ~60 ms before we start playing

// ---------------------------------------------------------------------------
// Packet layout (244 bytes on the wire)
// ---------------------------------------------------------------------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;                   // PKT_MAGIC sanity byte
  uint8_t  seq;                     // sequence number (loss detection)
  uint16_t count;                   // number of valid samples in pcm[]
  int16_t  pcm[SAMPLES_PER_PKT];    // raw 16-bit mono PCM
} AudioPkt;

// ---------------------------------------------------------------------------
// Objects / state
// ---------------------------------------------------------------------------
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
I2SClass i2s;

const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

// TX scratch (stereo capture -> mono) and RX scratch (mono -> stereo playback)
int16_t txStereo[SAMPLES_PER_PKT * 2];
int16_t txMono[SAMPLES_PER_PKT];
int16_t rxStereo[SAMPLES_PER_PKT * 2];
int16_t rxMono[SAMPLES_PER_PKT];

// Jitter ring buffer. Single-producer (recv callback) / single-consumer (loop),
// so free-running head/tail indices need no lock.
volatile uint32_t ringHead = 0;       // written by recv callback
volatile uint32_t ringTail = 0;       // read by loop()
int16_t           ring[RING_SIZE];

// Modes & counters
volatile bool txMode  = false;        // true while PTT held (transmitting)
bool          playing = false;        // RX: past prebuffer, actively playing
uint8_t       txSeq   = 0;

volatile uint32_t pktsTx = 0, pktsRx = 0, pktsDrop = 0, ringOverflow = 0;
volatile uint8_t  lastRxSeq = 0;
volatile bool     haveLastRx = false;

// Identity / link info shown on screen.
char              myMacStr[18] = "??:??:??:??:??:??";   // this board's STA MAC
volatile uint8_t  peerMac[6]   = {0};                   // last sender we heard from
volatile bool     havePeer     = false;
volatile uint32_t lastRxMillis = 0;     // when we last received a voice packet

// DAC playback volume, adjustable at runtime via the on-screen +/- buttons.
uint8_t           dacVolume    = DAC_VOLUME_DEFAULT;

// Cached "receiving voice" indicator state (-1 = needs redraw) so we only repaint
// the blinking sign when it actually changes.
int8_t            rxIndState   = -1;
bool              peerDrawn    = false;

// ---------------------------------------------------------------------------
// PTT button
// ---------------------------------------------------------------------------
struct Button { int16_t x, y, w, h; };

// Vertical layout bands (rotation 0, 240x320).
#define Y_STATE   36     // TX/RX banner
#define H_STATE   50
#define Y_MAC     90     // two lines: own MAC + peer MAC
#define Y_STATS   120    // packet counters
#define Y_VOL     138    // volume row (- bar +)
#define H_VOL     54

Button btnPtt   = { 10, 200, 220, 112 };   // hold-to-talk
Button btnVolDn = { 10,  Y_VOL,  60, H_VOL };
Button btnVolUp = { 170, Y_VOL,  60, H_VOL };

bool hit(const Button& b, int16_t x, int16_t y) {
  return x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h;
}

// ---------------------------------------------------------------------------
// FT6336 capacitive touch (direct I2C reads) - same as ES3C28P_radio
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
// ES8311 codec  -  combined full-duplex init (ADC mic path + DAC playback path)
// 16-bit I2S, MCLK = 256 * sample_rate. Built from the known-good register sets
// in ES3C28P_mic (ADC) and ES3C28P_radio (DAC).
// ---------------------------------------------------------------------------
void es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool es8311InitDuplex() {
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ES8311 not found on I2C!");
    return false;
  }

  es8311Write(0x00, 0x1F);   // reset
  delay(20);
  es8311Write(0x45, 0x00);

  // Clock manager: 0x3F enables MCLK + BCLK + DAC clock + ADC clock (bit3 is the
  // ADC clock - needed for capture). Dividers are the 256*fs set shared by both
  // existing sketches.
  es8311Write(0x01, 0x3F);
  es8311Write(0x02, 0x00);
  es8311Write(0x03, 0x10);
  es8311Write(0x04, 0x10);
  es8311Write(0x05, 0x00);
  es8311Write(0x06, 0x03);
  es8311Write(0x07, 0x00);
  es8311Write(0x08, 0xFF);

  // Serial data ports, both 16-bit I2S, both unmuted (full duplex).
  es8311Write(0x09, 0x0C);   // SDP IN  (ESP -> DAC)   16-bit I2S
  es8311Write(0x0A, 0x0C);   // SDP OUT (ADC -> ESP)   16-bit I2S, not muted

  es8311Write(0x0B, 0x00);
  es8311Write(0x0C, 0x00);
  es8311Write(0x0D, 0x01);   // power up analog circuits
  es8311Write(0x0E, 0x02);   // enable ADC modulator + analog PGA
  es8311Write(0x0F, 0x00);
  es8311Write(0x10, 0x1F);   // system: power up VMID / analog references
  es8311Write(0x11, 0x7F);   // system: power up analog
  es8311Write(0x12, 0x00);   // enable DAC
  es8311Write(0x13, 0x10);   // DAC output enable

  // ADC / analog mic path
  es8311Write(0x14, 0x1A);            // select analog MIC1, enable PGA
  es8311Write(0x15, 0x40);            // ADC soft-ramp
  es8311Write(0x16, MIC_PGA_GAIN);    // ADC analog PGA gain  (tunable)
  es8311Write(0x17, ADC_VOLUME);      // ADC digital volume   (tunable)
  es8311Write(0x18, 0x00);
  es8311Write(0x19, 0x00);
  es8311Write(0x1A, 0x00);
  es8311Write(0x1B, 0x0A);            // ADC HPF / settings
  es8311Write(0x1C, 0x6A);            // ADC HPF / EQ bypass

  // DAC playback path
  es8311Write(0x32, dacVolume);       // DAC volume (runtime-adjustable, see +/-)
  es8311Write(0x37, 0x08);            // ramp rate (shared by both paths)
  es8311Write(0x44, 0x50);            // data path: DAC fed from SDP IN (no loopback)

  es8311Write(0x00, 0x80);            // power up: slave mode, normal operation
  Serial.println("ES8311 full-duplex (ADC+DAC) initialised");
  return true;
}

// ---------------------------------------------------------------------------
// ESP-NOW receive callback. Runs in the WiFi task: keep it short. Push the
// packet's samples into the jitter ring (drop oldest on overflow). Ignored
// while we are transmitting (half-duplex).
// ---------------------------------------------------------------------------
void onEspNowRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (txMode) return;
  if (len < (int)sizeof(uint32_t)) return;
  const AudioPkt *p = (const AudioPkt *)data;
  if (p->magic != PKT_MAGIC) return;

  uint16_t n = p->count;
  if (n > SAMPLES_PER_PKT) n = SAMPLES_PER_PKT;

  // Loss detection (best-effort, for serial stats only).
  if (haveLastRx) {
    uint8_t expected = (uint8_t)(lastRxSeq + 1);
    if (p->seq != expected) pktsDrop += (uint8_t)(p->seq - expected);
  }
  lastRxSeq  = p->seq;
  haveLastRx = true;
  pktsRx++;

  // Remember who we are hearing (for the on-screen peer MAC) and when, so the UI
  // can light up a "receiving voice" sign while packets are arriving.
  memcpy((void*)peerMac, info->src_addr, 6);
  havePeer     = true;
  lastRxMillis = millis();

  uint32_t head = ringHead;
  uint32_t tail = ringTail;
  for (uint16_t i = 0; i < n; i++) {
    if ((uint32_t)(head - tail) >= RING_SIZE) {   // full: drop this sample
      ringOverflow++;
      break;
    }
    ring[head & RING_MASK] = p->pcm[i];
    head++;
  }
  ringHead = head;
}

// ---------------------------------------------------------------------------
// Display
// ---------------------------------------------------------------------------
void drawTitleBar() {
  tft.fillRect(0, 0, SCREEN_W, 34, ILI9341_MAROON);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 9);
  tft.print("Walkie-Talkie");
}

// "Receiving voice" sign: a green dot at the right of the RX banner that blinks
// while voice packets are arriving from the peer, hollow grey when idle. This is
// the cue the listening side gets while the other side is transmitting.
void drawRxIndicator() {
  const int16_t cx = 200, cy = Y_STATE + H_STATE / 2;
  bool active = !txMode && (millis() - lastRxMillis < 300);

  int8_t want;                      // 0 = idle dot, 1/2 = blink phases, 3 = hidden (TX)
  if (txMode)        want = 3;
  else if (!active)  want = 0;
  else               want = ((millis() / 250) & 1) ? 1 : 2;

  if (want == rxIndState) return;
  rxIndState = want;

  uint16_t bg = txMode ? ILI9341_RED : ILI9341_DARKGREEN;
  tft.fillRect(cx - 26, Y_STATE + 2, 52, H_STATE - 4, bg);   // clear the dot area
  if (want == 3) return;                                     // TX: no RX sign

  if (want == 0) {                  // idle: hollow outline
    tft.drawCircle(cx, cy, 13, ILI9341_DARKGREY);
  } else {                          // receiving: filled, blinking bright/dim green
    tft.fillCircle(cx, cy, 13, want == 1 ? ILI9341_GREEN : 0x05A0);
    tft.setTextColor(ILI9341_WHITE);
    tft.setTextSize(1);
    tft.setCursor(cx - 27, cy - 3);
    tft.print(" RX");
  }
}

void drawState() {
  // Big state banner: TX (red) / RX (green) on the left, RX-voice sign on right.
  tft.fillRect(0, Y_STATE, SCREEN_W, H_STATE,
               txMode ? ILI9341_RED : ILI9341_DARKGREEN);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(16, Y_STATE + 9);
  tft.print(txMode ? "TX" : "RX");
  tft.setTextSize(1);
  tft.setCursor(16, Y_STATE + H_STATE - 11);
  tft.print(txMode ? "transmitting..." : "listening");

  rxIndState = -1;                  // force the indicator to repaint
  drawRxIndicator();
}

// Own MAC (cyan) and the peer we are linked with (yellow). Peer is the source of
// the most recent voice packet, so it reflects who we are actually talking to.
void drawMacInfo() {
  tft.fillRect(0, Y_MAC, SCREEN_W, 28, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_CYAN);
  tft.setCursor(6, Y_MAC);
  tft.printf("Me:   %s", myMacStr);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(6, Y_MAC + 14);
  if (havePeer)
    tft.printf("Peer: %02X:%02X:%02X:%02X:%02X:%02X",
               peerMac[0], peerMac[1], peerMac[2],
               peerMac[3], peerMac[4], peerMac[5]);
  else
    tft.print("Peer: (waiting for audio...)");
}

// Volume row: [-]  NN%  [+]  with a small level bar.
void drawVolume() {
  // Minus / plus buttons.
  tft.fillRoundRect(btnVolDn.x, btnVolDn.y, btnVolDn.w, btnVolDn.h, 8, ILI9341_NAVY);
  tft.drawRoundRect(btnVolDn.x, btnVolDn.y, btnVolDn.w, btnVolDn.h, 8, ILI9341_WHITE);
  tft.fillRoundRect(btnVolUp.x, btnVolUp.y, btnVolUp.w, btnVolUp.h, 8, ILI9341_NAVY);
  tft.drawRoundRect(btnVolUp.x, btnVolUp.y, btnVolUp.w, btnVolUp.h, 8, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(4);
  tft.setCursor(btnVolDn.x + btnVolDn.w / 2 - 8, btnVolDn.y + H_VOL / 2 - 16);
  tft.print("-");
  tft.setCursor(btnVolUp.x + btnVolUp.w / 2 - 12, btnVolUp.y + H_VOL / 2 - 16);
  tft.print("+");

  // Middle: percentage + level bar.
  const int16_t mx = btnVolDn.x + btnVolDn.w + 8;          // 78
  const int16_t mw = btnVolUp.x - 8 - mx;                  // ~84
  int pct = (int)dacVolume * 100 / 255;
  tft.fillRect(mx, Y_VOL, mw, H_VOL, ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d%%", pct);
  int16_t tw = (int16_t)strlen(buf) * 12;
  tft.setCursor(mx + (mw - tw) / 2, Y_VOL + 6);
  tft.print(buf);
  int16_t barY = Y_VOL + H_VOL - 16;
  tft.drawRect(mx, barY, mw, 10, ILI9341_WHITE);
  tft.fillRect(mx + 1, barY + 1, (mw - 2) * pct / 100, 8, ILI9341_GREEN);
}

// Clamp + apply a new DAC volume, repaint the row only on a real change.
void setDacVolume(int v) {
  if (v < DAC_VOLUME_MIN) v = DAC_VOLUME_MIN;
  if (v > DAC_VOLUME_MAX) v = DAC_VOLUME_MAX;
  if ((uint8_t)v == dacVolume) return;
  dacVolume = (uint8_t)v;
  es8311Write(0x32, dacVolume);
  drawVolume();
}

void drawPttButton() {
  uint16_t col = txMode ? ILI9341_RED : ILI9341_NAVY;
  tft.fillRoundRect(btnPtt.x, btnPtt.y, btnPtt.w, btnPtt.h, 12, col);
  tft.drawRoundRect(btnPtt.x, btnPtt.y, btnPtt.w, btnPtt.h, 12, ILI9341_WHITE);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(3);
  const char* t = "PTT";
  int16_t tw = (int16_t)strlen(t) * 18;            // ~18px per char at size 3
  tft.setCursor(btnPtt.x + (btnPtt.w - tw) / 2, btnPtt.y + btnPtt.h / 2 - 22);
  tft.print(t);
  tft.setTextSize(1);
  const char* h = "hold to talk";
  tft.setCursor(btnPtt.x + (btnPtt.w - (int16_t)strlen(h) * 6) / 2,
                btnPtt.y + btnPtt.h / 2 + 12);
  tft.print(h);
}

void drawStats() {
  tft.fillRect(0, Y_STATS, SCREEN_W, 14, ILI9341_BLACK);
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(6, Y_STATS + 2);
  tft.printf("TX %lu  RX %lu  drop %lu  ovf %lu",
             (unsigned long)pktsTx, (unsigned long)pktsRx,
             (unsigned long)pktsDrop, (unsigned long)ringOverflow);
}

void redrawAll() {
  tft.fillScreen(ILI9341_BLACK);
  drawTitleBar();
  drawState();
  drawMacInfo();
  drawStats();
  drawVolume();
  drawPttButton();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P walkie-talkie (ESP-NOW push-to-talk)");

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(ILI9341_BLACK);

  // I2C: touch + codec
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  touchReset();

  // Amplifier off until we are in RX with audio to play (avoids a pop / feedback)
  pinMode(PA_EN, OUTPUT);
  digitalWrite(PA_EN, PA_OFF);

  es8311InitDuplex();

  // I2S master, full-duplex: both DOUT (playback) and DIN (capture) wired so the
  // peripheral runs MCLK/BCLK/LRCK and gives us both read() and write().
  i2s.setPins(I2S_BCLK, I2S_LRCK, I2S_DOUT, I2S_DIN, I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE,
                 I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO)) {
    Serial.println("I2S init failed!");
    tft.setTextColor(ILI9341_RED);
    tft.setTextSize(2);
    tft.setCursor(10, 150);
    tft.print("I2S init failed");
    while (true) delay(1000);
  }
  Serial.println("I2S full-duplex started");

  // WiFi in station mode (not connected) so ESP-NOW has a radio. Fix the channel
  // so both boards agree without an AP to negotiate it.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  // Push the radio to maximum transmit power for the best range.
  esp_wifi_set_max_tx_power(MAX_TX_POWER);
#if USE_LONG_RANGE
  // Enable the proprietary Long-Range PHY in addition to the standard rates.
  // ESP-NOW frames are then sent with LR modulation, greatly extending range.
  esp_wifi_set_protocol(WIFI_IF_STA,
      WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR);
  Serial.println("Long-Range PHY enabled");
#endif

  // Read the STA MAC from eFuse directly - WiFi.macAddress() can return all-zeros
  // right after WiFi.mode() before the interface is fully up. esp_read_mac() does
  // not depend on the WiFi driver being started, so it is always valid here.
  uint8_t mac[6] = {0};
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(myMacStr, sizeof(myMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.print("This board STA MAC: ");
  Serial.println(myMacStr);            // also shown on screen; put in PEER_MAC on peer

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onEspNowRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, USE_BROADCAST ? BCAST_MAC : PEER_MAC, 6);
  peer.channel = WIFI_CHANNEL;
  peer.ifidx   = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK)
    Serial.println("ESP-NOW add_peer failed!");

  redrawAll();
  Serial.println("Ready. Hold PTT to transmit.");
}

// ---------------------------------------------------------------------------
// TX: capture one packet's worth of mic audio and broadcast it.
// i2s.readBytes() blocks until the frame is full, which paces TX at SAMPLE_RATE.
// ---------------------------------------------------------------------------
void doTransmit() {
  size_t want = sizeof(txStereo);
  size_t got  = i2s.readBytes((char*)txStereo, want);
  if (got < want) return;

  // Codec is mono on one slot of the stereo frame - keep the even slot (mic).
  for (int i = 0; i < SAMPLES_PER_PKT; i++) txMono[i] = txStereo[2 * i];

  AudioPkt pkt;
  pkt.magic = PKT_MAGIC;
  pkt.seq   = txSeq++;
  pkt.count = SAMPLES_PER_PKT;
  memcpy(pkt.pcm, txMono, sizeof(txMono));

  esp_now_send(USE_BROADCAST ? BCAST_MAC : PEER_MAC, (uint8_t*)&pkt, sizeof(pkt));
  pktsTx++;
}

// ---------------------------------------------------------------------------
// RX: once past the prebuffer, pop a chunk from the jitter ring, duplicate mono
// into both I2S slots, and write it. i2s.write() blocks on DMA, pacing playback.
// On underrun we stop and re-accumulate the prebuffer to avoid choppy output.
// ---------------------------------------------------------------------------
void doReceive() {
  uint32_t avail = ringHead - ringTail;

  if (!playing) {
    if (avail < PREBUFFER) return;     // wait for a cushion before starting
    playing = true;
  }
  if (avail < SAMPLES_PER_PKT) {       // underrun - rebuffer
    playing = false;
    return;
  }

  uint32_t tail = ringTail;
  for (int i = 0; i < SAMPLES_PER_PKT; i++) {
    int16_t s = ring[(tail + i) & RING_MASK];
    rxStereo[2 * i]     = s;           // duplicate mono into L and R slots
    rxStereo[2 * i + 1] = s;
  }
  ringTail = tail + SAMPLES_PER_PKT;

  i2s.write((const uint8_t*)rxStereo, sizeof(rxStereo));
}

// ---------------------------------------------------------------------------
// Loop: PTT state machine + audio pump + periodic stats.
// ---------------------------------------------------------------------------
void loop() {
  static uint32_t lastStats   = 0;
  static uint32_t lastVolStep = 0;

  // --- PTT: enter TX on a button press, stay until the finger lifts ---
  uint16_t tx, ty;
  bool touched = getTouch(&tx, &ty);

  // --- Volume +/- (RX only). Auto-repeats while held, throttled. ---
  if (!txMode && touched && (millis() - lastVolStep > 160)) {
    if (hit(btnVolDn, tx, ty)) {
      setDacVolume((int)dacVolume - DAC_VOLUME_STEP);
      lastVolStep = millis();
    } else if (hit(btnVolUp, tx, ty)) {
      setDacVolume((int)dacVolume + DAC_VOLUME_STEP);
      lastVolStep = millis();
    }
  }

  if (!txMode) {
    if (touched && hit(btnPtt, tx, ty)) {
      // RX -> TX
      txMode = true;
      digitalWrite(PA_EN, PA_OFF);     // amp off while talking (no feedback)
      // Drop any queued/stale RX audio so the next listen starts clean.
      ringTail   = ringHead;
      playing    = false;
      haveLastRx = false;
      drawState();
      drawPttButton();
    }
  } else {
    if (!touched) {
      // TX -> RX
      txMode = false;
      digitalWrite(PA_EN, PA_ON);      // amp on to hear the other side
      drawState();
      drawPttButton();
    }
  }

  if (txMode) doTransmit();
  else        doReceive();

  // "Receiving voice" sign - cheap, refresh every loop so it blinks promptly.
  drawRxIndicator();

  // Fill in the peer MAC the first time we hear from someone.
  if (havePeer && !peerDrawn) {
    peerDrawn = true;
    drawMacInfo();
  }

  // Periodic stats (cheap; doesn't disturb the audio pump much).
  if (millis() - lastStats > 500) {
    lastStats = millis();
    drawStats();
  }
}
