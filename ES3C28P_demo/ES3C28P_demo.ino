/*
 * ES3C28P_demo  -  Graphics + capacitive-touch demo for the ES3C28P board.
 *
 * Board : ES3C28P  (ESP32-S3-WROOM-1, N16R8 - 16MB flash / 8MB PSRAM, USB-C)
 * Screen: 2.8" IPS, 240x320, ILI9341V driver, 4-wire SPI
 * Touch : FT6336G capacitive controller, I2C (5-point), addr 0x38
 *
 * Libraries (installed locally under ./.arduino/user/libraries by the Makefile):
 *   - Adafruit GFX Library
 *   - Adafruit ILI9341
 *   - Adafruit BusIO
 *
 * The FT6336 touch controller is read directly over I2C (its register map is
 * trivial and many ready-made libraries reject the "G" silicon on a chip-ID
 * check), so no extra touch library is required.
 *
 * Demo: on boot it runs a short graphics showcase, then drops into a simple
 * finger-paint app with a color palette and a CLEAR button.
 */

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

// ---------------------------------------------------------------------------
// Pin map (see board datasheet / lcdwiki E32C28P page)
// ---------------------------------------------------------------------------
// Display - SPI
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_CS   10
#define TFT_DC   46
#define TFT_RST  -1   // RST is tied to the ESP32-S3 reset line; no GPIO control
#define TFT_BL   45   // backlight, active HIGH

// Touch - I2C (FT6336G)
#define TOUCH_SDA  16
#define TOUCH_SCL  15
#define TOUCH_INT  17
#define TOUCH_RST  18
#define FT6336_ADDR 0x38

// If touch axes come out mirrored on your panel, flip these to 1.
#define TOUCH_FLIP_X 0
#define TOUCH_FLIP_Y 0

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
Adafruit_ILI9341 tft = Adafruit_ILI9341(&SPI, TFT_DC, TFT_CS, TFT_RST);

// Screen is used in portrait (rotation 0): 240 wide x 320 tall.
const int16_t SCREEN_W = 240;
const int16_t SCREEN_H = 320;

// UI layout
const int16_t TITLE_H   = 30;   // title bar height
const int16_t PALETTE_H = 30;   // palette row height
const int16_t CANVAS_Y  = TITLE_H + PALETTE_H;   // canvas starts here (60)
const int16_t CLEAR_W   = 60;   // CLEAR button width (top-right of title bar)

// Color palette (6 swatches across the 240px-wide row = 40px each)
const uint16_t PALETTE[6] = {
  ILI9341_RED, ILI9341_ORANGE, ILI9341_YELLOW,
  ILI9341_GREEN, ILI9341_CYAN, ILI9341_WHITE
};
const int16_t SWATCH_W = SCREEN_W / 6;   // 40px

uint16_t brushColor = ILI9341_GREEN;
const int16_t BRUSH_R = 4;

// ---------------------------------------------------------------------------
// FT6336 capacitive touch  (direct I2C reads)
// ---------------------------------------------------------------------------
void touchReset() {
  pinMode(TOUCH_RST, OUTPUT);
  digitalWrite(TOUCH_RST, LOW);
  delay(10);
  digitalWrite(TOUCH_RST, HIGH);
  delay(300);                 // controller boot time
}

// Returns true and fills *x,*y (in screen pixels) when a finger is present.
bool getTouch(uint16_t *x, uint16_t *y) {
  uint8_t buf[5];

  Wire.beginTransmission(FT6336_ADDR);
  Wire.write(0x02);                              // start at TD_STATUS
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)FT6336_ADDR, 5) != 5) return false;
  for (int i = 0; i < 5; i++) buf[i] = Wire.read();

  uint8_t touches = buf[0] & 0x0F;               // number of active points
  if (touches == 0 || touches > 5) return false;

  uint16_t rawX = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];   // P1 X (0..239)
  uint16_t rawY = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];   // P1 Y (0..319)

#if TOUCH_FLIP_X
  rawX = SCREEN_W - 1 - rawX;
#endif
#if TOUCH_FLIP_Y
  rawY = SCREEN_H - 1 - rawY;
#endif

  if (rawX >= SCREEN_W) rawX = SCREEN_W - 1;
  if (rawY >= SCREEN_H) rawY = SCREEN_H - 1;
  *x = rawX;
  *y = rawY;
  return true;
}

// ---------------------------------------------------------------------------
// Graphics showcase (runs once at boot)
// ---------------------------------------------------------------------------
void graphicsShowcase() {
  tft.fillScreen(ILI9341_BLACK);

  // Title
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("ES3C28P");
  tft.setTextSize(1);
  tft.setCursor(10, 34);
  tft.setTextColor(ILI9341_CYAN);
  tft.println("ILI9341 + FT6336 demo");

  // Lines fanning out
  for (int16_t x = 0; x < SCREEN_W; x += 12)
    tft.drawLine(0, 60, x, 150, tft.color565(x % 256, 128, 255 - (x % 256)));

  // Shapes
  tft.drawRect(10, 160, 60, 50, ILI9341_RED);
  tft.fillRect(80, 160, 60, 50, ILI9341_BLUE);
  tft.drawRoundRect(150, 160, 70, 50, 10, ILI9341_YELLOW);

  tft.drawCircle(40, 250, 25, ILI9341_GREEN);
  tft.fillCircle(110, 250, 25, ILI9341_MAGENTA);
  tft.fillTriangle(160, 275, 185, 225, 210, 275, ILI9341_CYAN);

  // Color bars
  for (int i = 0; i < 6; i++)
    tft.fillRect(i * SWATCH_W, 290, SWATCH_W, 30, PALETTE[i]);

  delay(2500);
}

// ---------------------------------------------------------------------------
// Paint UI
// ---------------------------------------------------------------------------
void drawUI() {
  tft.fillScreen(ILI9341_BLACK);

  // Title bar
  tft.fillRect(0, 0, SCREEN_W, TITLE_H, ILI9341_NAVY);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(1);
  tft.setCursor(6, 11);
  tft.print("Touch to paint");

  // CLEAR button (top-right)
  tft.fillRect(SCREEN_W - CLEAR_W, 0, CLEAR_W, TITLE_H, ILI9341_RED);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(SCREEN_W - CLEAR_W + 10, 11);
  tft.print("CLEAR");

  // Palette row
  for (int i = 0; i < 6; i++) {
    tft.fillRect(i * SWATCH_W, TITLE_H, SWATCH_W, PALETTE_H, PALETTE[i]);
  }
  highlightBrush();
}

// Draw a marker around the swatch matching the current brush color.
void highlightBrush() {
  for (int i = 0; i < 6; i++) {
    uint16_t border = (PALETTE[i] == brushColor) ? ILI9341_BLACK : PALETTE[i];
    tft.drawRect(i * SWATCH_W, TITLE_H, SWATCH_W, PALETTE_H, border);
    tft.drawRect(i * SWATCH_W + 1, TITLE_H + 1, SWATCH_W - 2, PALETTE_H - 2, border);
  }
}

void clearCanvas() {
  tft.fillRect(0, CANVAS_Y, SCREEN_W, SCREEN_H - CANVAS_Y, ILI9341_BLACK);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P graphics + touch demo");

  // Backlight on
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // SPI with the board's custom pins. Calling SPI.begin() with explicit pins
  // first means Adafruit's later no-arg begin() is a no-op and keeps our pins.
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);

  tft.begin();
  tft.setRotation(0);                 // portrait, 240x320

  // I2C + touch controller
  Wire.begin(TOUCH_SDA, TOUCH_SCL);
  Wire.setClock(400000);
  touchReset();

  graphicsShowcase();
  drawUI();
}

void loop() {
  uint16_t x, y;
  if (getTouch(&x, &y)) {
    Serial.printf("touch  x=%u  y=%u\n", x, y);

    if (y < TITLE_H) {
      // Title bar: only the CLEAR button is active
      if (x >= SCREEN_W - CLEAR_W) {
        clearCanvas();
        delay(200);                   // simple debounce so one tap = one clear
      }
    } else if (y < CANVAS_Y) {
      // Palette row: pick a brush color
      int idx = x / SWATCH_W;
      if (idx >= 0 && idx < 6) {
        brushColor = PALETTE[idx];
        highlightBrush();
        delay(150);
      }
    } else {
      // Canvas: paint
      tft.fillCircle(x, y, BRUSH_R, brushColor);
    }
  }
  delay(5);
}
