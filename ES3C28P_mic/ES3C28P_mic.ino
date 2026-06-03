/*
 * ES3C28P_mic  -  Microphone test + real-time spectrum analyzer on the LCD.
 *
 * Board : ES3C28P  (ESP32-S3-WROOM-1, N16R8 - 16MB flash / 8MB PSRAM, USB-C)
 * Screen: 2.8" IPS 240x320, ILI9341V, SPI
 * Audio : analog MIC -> ES8311 codec ADC (I2C 0x18) -> I2S DIN (GPIO6) -> ESP32
 *
 * What it does
 * ------------
 * Captures audio from the on-board microphone, runs a 256-point FFT, and draws
 * a bar-graph spectrum (log-spaced bins) on the display. A level meter and a
 * "LISTENING / quiet" indicator show when the mic actually picks up sound; the
 * bars only rise above the floor once the input crosses a noise gate, and the
 * dominant frequency is printed while sound is present.
 *
 * Signal chain
 * ------------
 * The ESP32 is the I2S *master*: it generates MCLK(4)/BCLK(5)/LRCK(7) and reads
 * the ADC data on DIN(6). The ES8311 runs as an I2S slave with its ADC fed from
 * the analog MIC1 input through the internal PGA. The codec has no Arduino
 * driver, so it is configured with a raw register sequence (16-bit I2S,
 * MCLK = 256*fs) - the playback init in ES3C28P_radio is reused and extended to
 * power up the ADC / analog-mic path. Two details that were needed to make the
 * mic actually produce data (verified on hardware): clock-manager reg 0x01 must
 * enable the ADC clock (0x3F, not 0x30), and the mono mic lands on one slot of
 * the I2S frame - we capture STEREO and keep the slot that carries it.
 *
 * Libraries (installed locally under ./.arduino/user/libraries by the Makefile):
 *   - Adafruit GFX / ILI9341 / BusIO   display
 *   - ESP_I2S                          (bundled with the ESP32 core) I2S RX
 * The FFT is implemented in this file (small radix-2), so no extra library is
 * required.
 *
 * Tuning: if the bars are too quiet/too hot, adjust MIC_PGA_GAIN (analog) and
 * ADC_VOLUME (digital) below, or the SPEC_MIN_DB / NOISE_GATE_DB thresholds.
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <ESP_I2S.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Pin map  (identical to ES3C28P_radio - see board datasheet / BSP)
// ---------------------------------------------------------------------------
// Display - SPI
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_CS   10
#define TFT_DC   46
#define TFT_RST  -1          // tied to ESP32-S3 reset
#define TFT_BL   45          // backlight, active HIGH

// Shared I2C bus (codec ES8311 @0x18; touch FT6336 also lives here but unused)
#define I2C_SDA  16
#define I2C_SCL  15
#define ES8311_ADDR 0x18

// Audio I2S  (BSP: MCLK=4, BCLK=5, LRCK=7, DOUT/playback=8, DIN/mic=6)
#define I2S_MCLK 4
#define I2S_BCLK 5
#define I2S_LRCK 7
#define I2S_DIN  6           // ADC data from the codec into the ESP32
#define I2S_DOUT 8           // DAC data (unused here; keeps I2S full-duplex)
#define PA_EN    1           // FM8002E amplifier enable
#define PA_OFF HIGH          // keep the speaker amp off; this sketch only listens

// ---------------------------------------------------------------------------
// Capture / FFT settings
// ---------------------------------------------------------------------------
#define SAMPLE_RATE  16000   // Hz  -> Nyquist 8 kHz, FFT bin width 62.5 Hz
#define FFT_N        256     // power of 2
#define HALF_N       (FFT_N / 2)

// ES8311 mic gain knobs (see datasheet). Raise if the spectrum looks dead.
#define MIC_PGA_GAIN 0x24    // reg 0x16 - analog PGA stage
#define ADC_VOLUME   0xE6    // reg 0x17 - digital ADC volume (0..255)

// Display scaling (tuned against measured levels: quiet floor ~-67 dBFS,
// ordinary speech/sound ~-50 dBFS on this board's mic + PGA setting).
#define SPEC_MIN_DB  (-64.0f) // bottom of the bars (dBFS)
#define SPEC_MAX_DB  (-12.0f) // top of the bars
#define NOISE_GATE_DB (-54.0f)// input RMS below this -> treated as "quiet"

// ---------------------------------------------------------------------------
// Objects / layout
// ---------------------------------------------------------------------------
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);
I2SClass i2s;

const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

// Spectrum drawing area
const int16_t SPEC_X   = 4;
const int16_t SPEC_W   = 232;
const int16_t SPEC_TOP = 78;
const int16_t SPEC_BOT = 300;
const int16_t SPEC_H   = SPEC_BOT - SPEC_TOP;

const int NUM_BARS = 24;
const int16_t BAR_SLOT = SPEC_W / NUM_BARS;   // 9 px
const int16_t BAR_W    = BAR_SLOT - 2;        // 7 px wide, 2 px gap

// Capture + FFT buffers. The codec is mono but drives one slot of a stereo
// I2S frame; we read stereo and keep the slot that actually carries the mic.
int16_t  stereoBuf[FFT_N * 2];                 // interleaved L,R as captured
int16_t  sampleBuf[FFT_N];                     // de-interleaved mic channel
float    fftRe[FFT_N];
float    fftIm[FFT_N];
float    hann[FFT_N];                          // precomputed window

// Per-bar render state (for flicker-free delta drawing)
int16_t  barH[NUM_BARS];                       // current drawn height
int16_t  peakH[NUM_BARS];                      // peak-hold height
uint16_t barColor[NUM_BARS];                   // current bar color category

// Log-spaced bin edges: bar b spans FFT bins [binEdge[b], binEdge[b+1])
int      binEdge[NUM_BARS + 1];

// ---------------------------------------------------------------------------
// ES8311 codec  -  power up the ADC / analog-mic path (16-bit I2S, 256*fs MCLK)
// ---------------------------------------------------------------------------
void es8311Write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(ES8311_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool es8311InitMic() {
  Wire.beginTransmission(ES8311_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("ES8311 not found on I2C!");
    return false;
  }

  es8311Write(0x00, 0x1F);   // reset
  delay(20);
  es8311Write(0x45, 0x00);

  // Clock manager: 0x3F enables MCLK + BCLK + DAC + *ADC* clocks (bit3 is the
  // ADC clock - without it the ADC produces no data). Divider values below are
  // the known-good 256*fs set from the radio sketch (same MCLK relationship).
  es8311Write(0x01, 0x3F);
  es8311Write(0x02, 0x00);
  es8311Write(0x03, 0x10);
  es8311Write(0x04, 0x10);
  es8311Write(0x05, 0x00);
  es8311Write(0x06, 0x03);
  es8311Write(0x07, 0x00);
  es8311Write(0x08, 0xFF);

  // Serial data ports: 16-bit I2S. 0x0A (SDP-OUT) is the ADC->ESP path we read.
  es8311Write(0x09, 0x0C);   // SDP IN  (DAC, unused) 16-bit I2S
  es8311Write(0x0A, 0x0C);   // SDP OUT (ADC -> ESP)  16-bit I2S, not muted

  es8311Write(0x0B, 0x00);
  es8311Write(0x0C, 0x00);
  es8311Write(0x10, 0x1F);   // system: power up VMID / analog references
  es8311Write(0x11, 0x7F);   // system: power up analog
  es8311Write(0x0D, 0x01);   // power up analog circuits
  es8311Write(0x0E, 0x02);   // enable ADC modulator + analog PGA
  es8311Write(0x12, 0x00);
  es8311Write(0x13, 0x10);

  es8311Write(0x14, 0x1A);          // select analog MIC1, enable PGA
  es8311Write(0x15, 0x40);          // ADC: soft-ramp
  es8311Write(0x16, MIC_PGA_GAIN);  // ADC: analog PGA gain  (tunable)
  es8311Write(0x17, ADC_VOLUME);    // ADC: digital volume   (tunable)
  es8311Write(0x1B, 0x0A);          // ADC: HPF / settings
  es8311Write(0x1C, 0x6A);          // ADC: HPF / EQ bypass
  es8311Write(0x37, 0x08);          // ADC ramp rate

  es8311Write(0x00, 0x80);          // power up: slave mode, normal operation
  Serial.println("ES8311 ADC/mic initialised");
  return true;
}

// ---------------------------------------------------------------------------
// Radix-2 iterative FFT (in place). Input in fftRe/fftIm, length FFT_N.
// ---------------------------------------------------------------------------
void fft() {
  // Bit-reversal permutation
  for (int i = 1, j = 0; i < FFT_N; i++) {
    int bit = FFT_N >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float tr = fftRe[i]; fftRe[i] = fftRe[j]; fftRe[j] = tr;
      float ti = fftIm[i]; fftIm[i] = fftIm[j]; fftIm[j] = ti;
    }
  }
  // Butterflies
  for (int len = 2; len <= FFT_N; len <<= 1) {
    float ang = -2.0f * (float)M_PI / len;
    float wr = cosf(ang), wi = sinf(ang);
    for (int i = 0; i < FFT_N; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (int k = 0; k < len / 2; k++) {
        int a = i + k, b = i + k + len / 2;
        float tr = cr * fftRe[b] - ci * fftIm[b];
        float ti = cr * fftIm[b] + ci * fftRe[b];
        fftRe[b] = fftRe[a] - tr;
        fftIm[b] = fftIm[a] - ti;
        fftRe[a] += tr;
        fftIm[a] += ti;
        float ncr = cr * wr - ci * wi;   // advance twiddle
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
void drawTitleBar() {
  tft.fillRect(0, 0, SCREEN_W, 34, ILI9341_MAROON);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(8, 9);
  tft.print("Mic Spectrum");
}

void drawStaticUI() {
  tft.fillScreen(ILI9341_BLACK);
  drawTitleBar();
  // Frame around the spectrum region
  tft.drawRect(SPEC_X - 1, SPEC_TOP - 1, SPEC_W + 2, SPEC_H + 2, ILI9341_DARKGREY);
  // Frequency axis hint
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setTextSize(1);
  tft.setCursor(SPEC_X, SPEC_BOT + 4);
  tft.print("0");
  tft.setCursor(SPEC_X + SPEC_W / 2 - 12, SPEC_BOT + 4);
  tft.print("freq");
  tft.setCursor(SPEC_X + SPEC_W - 30, SPEC_BOT + 4);
  tft.print("8kHz");
}

// Color a bar by how tall it is: green -> yellow -> red.
uint16_t heightColor(int16_t h) {
  float f = (float)h / SPEC_H;
  if (f < 0.5f) return ILI9341_GREEN;
  if (f < 0.8f) return ILI9341_YELLOW;
  return ILI9341_RED;
}

// Flicker-free per-bar update with peak-hold marker.
void drawBar(int b, int16_t newH) {
  int16_t x = SPEC_X + b * BAR_SLOT;
  int16_t old = barH[b];
  uint16_t col = heightColor(newH);

  if (col != barColor[b]) {
    // Color category changed - repaint the whole solid column.
    if (newH > 0)
      tft.fillRect(x, SPEC_BOT - newH, BAR_W, newH, col);
    if (newH < old)
      tft.fillRect(x, SPEC_BOT - old, BAR_W, old - newH, ILI9341_BLACK);
    barColor[b] = col;
  } else if (newH > old) {
    tft.fillRect(x, SPEC_BOT - newH, BAR_W, newH - old, col);   // grow up
  } else if (newH < old) {
    tft.fillRect(x, SPEC_BOT - old, BAR_W, old - newH, ILI9341_BLACK); // shrink
  }
  barH[b] = newH;

  // Peak-hold marker (2 px white line that slowly falls).
  int16_t oldPeakY = SPEC_BOT - peakH[b];
  if (newH >= peakH[b]) peakH[b] = newH;
  else if (peakH[b] > 0) peakH[b] -= 2;            // decay
  if (peakH[b] < newH) peakH[b] = newH;

  int16_t newPeakY = SPEC_BOT - peakH[b];
  if (newPeakY != oldPeakY) {
    // Erase old marker: restore bar color if still inside the bar, else black.
    uint16_t bg = (SPEC_BOT - oldPeakY) <= newH ? col : ILI9341_BLACK;
    tft.fillRect(x, oldPeakY - 1, BAR_W, 2, bg);
  }
  if (peakH[b] > 0)
    tft.fillRect(x, newPeakY - 1, BAR_W, 2, ILI9341_WHITE);
}

// Status line: input level bar + LISTENING/quiet + dominant frequency.
void drawStatus(float levelDb, bool active, float peakFreq) {
  const int16_t Y = 40, H = 14;
  // Level bar (maps SPEC_MIN_DB..SPEC_MAX_DB across the width)
  tft.drawRect(SPEC_X, Y, SPEC_W, H, ILI9341_DARKGREY);
  float f = (levelDb - SPEC_MIN_DB) / (SPEC_MAX_DB - SPEC_MIN_DB);
  if (f < 0) f = 0; if (f > 1) f = 1;
  int16_t fill = (int16_t)((SPEC_W - 2) * f);
  uint16_t lc = active ? ILI9341_GREEN : ILI9341_NAVY;
  tft.fillRect(SPEC_X + 1, Y + 1, fill, H - 2, lc);
  tft.fillRect(SPEC_X + 1 + fill, Y + 1, (SPEC_W - 2) - fill, H - 2, ILI9341_BLACK);

  // Text line below the level bar
  tft.fillRect(SPEC_X, Y + H + 3, SPEC_W, 10, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setCursor(SPEC_X, Y + H + 3);
  if (active) {
    tft.setTextColor(ILI9341_GREEN);
    tft.printf("LISTENING   peak %4.0f Hz", peakFreq);
  } else {
    tft.setTextColor(ILI9341_DARKGREY);
    tft.print("quiet - make some noise");
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P microphone spectrum analyzer");

  // Display
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(0);
  drawStaticUI();

  // Keep the speaker amplifier disabled - we only listen.
  pinMode(PA_EN, OUTPUT);
  digitalWrite(PA_EN, PA_OFF);

  // I2C + codec (ADC / analog-mic path)
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);
  es8311InitMic();

  // I2S master, full-duplex (DOUT provided too) so the peripheral reliably
  // generates MCLK/BCLK/LRCK at 256*fs even though we only consume RX.
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
  Serial.println("I2S RX started");

  // Precompute the Hann window and the log-spaced bar bin edges.
  for (int i = 0; i < FFT_N; i++)
    hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_N - 1)));

  const float lo = 1.0f, hi = (float)HALF_N;     // bins 1..128
  for (int b = 0; b <= NUM_BARS; b++) {
    float e = lo * powf(hi / lo, (float)b / NUM_BARS);
    binEdge[b] = (int)(e + 0.5f);
  }
  for (int b = 0; b < NUM_BARS; b++)
    if (binEdge[b + 1] <= binEdge[b]) binEdge[b + 1] = binEdge[b] + 1;

  for (int b = 0; b < NUM_BARS; b++) { barColor[b] = 0xFFFF; barH[b] = peakH[b] = 0; }
}

// ---------------------------------------------------------------------------
// Loop:  capture -> window+FFT -> bars
// ---------------------------------------------------------------------------
void loop() {
  // Block until a full stereo frame is captured, then keep the mic slot.
  size_t want = sizeof(stereoBuf);
  size_t got  = i2s.readBytes((char*)stereoBuf, want);
  if (got < want) return;
  for (int i = 0; i < FFT_N; i++) sampleBuf[i] = stereoBuf[2 * i];   // even slot = mic

  // DC removal + RMS (raw counts) for the noise gate / level meter.
  long sum = 0;
  for (int i = 0; i < FFT_N; i++) sum += sampleBuf[i];
  float mean = (float)sum / FFT_N;

  float energy = 0.0f;
  for (int i = 0; i < FFT_N; i++) {
    float s = sampleBuf[i] - mean;
    energy += s * s;
    fftRe[i] = s * hann[i];     // windowed input
    fftIm[i] = 0.0f;
  }
  float rms     = sqrtf(energy / FFT_N);
  float levelDb = 20.0f * log10f(rms / 32768.0f + 1e-7f);
  bool  active  = levelDb > NOISE_GATE_DB;

  fft();

  // Per-bin magnitude -> dBFS, grouped (max) into log-spaced bars.
  float peakMag = 0.0f; int peakBin = 1;
  for (int b = 0; b < NUM_BARS; b++) {
    float maxMag = 0.0f;
    for (int k = binEdge[b]; k < binEdge[b + 1] && k < HALF_N; k++) {
      float mag = sqrtf(fftRe[k] * fftRe[k] + fftIm[k] * fftIm[k]);
      if (mag > maxMag) maxMag = mag;
      if (mag > peakMag) { peakMag = mag; peakBin = k; }
    }
    int16_t h = 0;
    if (active) {
      float norm = (2.0f / FFT_N) * maxMag / 32768.0f;   // 0..1 full scale
      float db   = 20.0f * log10f(norm + 1e-7f);
      float f    = (db - SPEC_MIN_DB) / (SPEC_MAX_DB - SPEC_MIN_DB);
      if (f < 0) f = 0; if (f > 1) f = 1;
      h = (int16_t)(f * SPEC_H);
    }
    drawBar(b, h);
  }

  float peakFreq = (float)peakBin * SAMPLE_RATE / FFT_N;
  drawStatus(levelDb, active, peakFreq);
}
