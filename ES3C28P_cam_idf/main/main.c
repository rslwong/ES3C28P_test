/*
 * ES3C28P_cam_idf  -  Show a Tapo camera's MJPEG RTSP stream (stream8) on the panel.
 *                     ESP-IDF port of ../ES3C28P_cam (Arduino).
 *
 * Board : ES3C28P (ESP32-S3 N16R8), 2.8" ILI9341 240x320 over SPI.
 *
 * Why ESP-IDF: the Arduino core ships lwIP precompiled with a 5.7 KB TCP
 * receive window. With the camera's ~28 ms WiFi round trip that caps RTSP at
 * ~2 Mbit/s, while stream8 pushes ~10 Mbit/s. Here sdkconfig.defaults sets a
 * 64 KB window (plus bigger WiFi/lwIP queues), so the whole stream fits.
 *
 * How it works:
 *   RTSP over TCP (Basic auth) -> DESCRIBE / SETUP / PLAY rtsp://CAM/stream8,
 *   RTP packets arrive interleaved on the same socket ("$" framing). Tapo puts
 *   raw JPEG bytes (FFD8..FFD9) behind each RFC 2435 header, so we just strip the
 *   8-byte header and join fragments. esp_new_jpeg scales 640x360 to 320x176
 *   (multiples of 8) straight into a DMA buffer that goes out over SPI.
 *
 * Tapo stream8 quirks (measured; the camera, not the network):
 *   - It re-sends one buffer ~15-25 times/s while its encoder overwrites it, so
 *     an RTP frame is rarely exactly one JPEG. We take the complete SOI..EOI
 *     inside it, skip repeats by hash, and drop "torn" ones (head of one image +
 *     tail of another) with jpegcheck.c. Net: every clean image the camera
 *     actually delivers, ~2-3/s.
 *   - It writes RTSP replies into the middle of an RTP packet, so after each
 *     GET_PARAMETER keepalive (needed: session timeout 15 s) we resync on "$".
 *
 *   core 0  readerTask : owns the socket - RTSP setup, keepalive, reconnect, and
 *                        frame reassembly.
 *   core 1  displayTask: decodes the newest frame, draws it, plus the status line.
 *   Three JPEG buffers (triple buffering) so neither side ever waits on the other.
 *
 * WiFi credentials are read from NVS (saved by ES3C28P_radio's on-screen setup;
 * partitions.csv keeps the Arduino NVS location). Camera address/login live in
 * main/secrets.h (gitignored) - see secrets.h.example.
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "mbedtls/base64.h"
#include "esp_jpeg_dec.h"   // components/esp_new_jpeg
#include "jpegcheck.h"      // torn-image detector
#include "glcdfont.h"       // Adafruit GFX classic 5x7 font
#include "secrets.h"        // CAM_HOST, CAM_USER, CAM_PASS

#ifndef WIFI_SSID
  #define WIFI_SSID ""
  #define WIFI_PASS ""
#endif

// Display pins (same as ES3C28P_radio)
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_CS   10
#define TFT_DC   46
#define TFT_BL   45

#define CAM_PATH     "stream8"      // Tapo's MJPEG profile (ONVIF profile_3)
#define JPG_MAX      (128 * 1024)   // one 640x360 frame is ~42 KB
#define LCD_W        320            // landscape
#define LCD_H        240
#define IMG_W        320            // 640x360 scaled; decoder wants multiples of 8
#define IMG_H        176
#define IMG_Y        32             // below the status line
#define BAR_H        30             // status line height
#define KEEPALIVE_MS 10000          // session timeout is 15 s
#define STALL_MS     8000           // no valid frame this long -> reconnect

// RGB565 colors as the panel wants them on the wire (big-endian)
#define BE(c)  ((uint16_t)(((c) >> 8) | ((c) << 8)))
#define WHITE  0xFFFF
#define GREEN  0x07E0
#define YELLOW 0xFFE0
#define RED    0xF800

static const char* TAG = "cam";
static inline uint32_t millis(void) { return esp_timer_get_time() / 1000; }

// --- reader task (core 0) state --------------------------------------------
static int      sock = -1;
static uint8_t  rx[16 * 1024];          // socket read buffer
static size_t   rxPos, rxLen;
static char     rtspUrl[96], authHdr[160], session[64];
static int      cseq;
static uint32_t lastKeepalive, lastFrameMs;
static size_t   jpgLen;                 // bytes of the current RTP frame so far
static size_t   segOff, segLen;         // where the JPEG sits inside it
static bool     jpgOk;                  // current frame has had no gaps so far
static volatile bool streaming;

// --- triple buffer shared by both cores ------------------------------------
static uint8_t*     bufs[3];            // JPEG buffers in PSRAM
static size_t       offs[3], lens[3];   // JPEG position inside each buffer
static int          wIdx = 0;           // reader writes here           (reader only)
static int          readyIdx = 1;       // newest complete frame        (shared, under mux)
static int          rIdx = 2;           // display decodes from here    (display only)
static bool         readyNew;           // readyIdx holds a frame display hasn't taken
static portMUX_TYPE bufMux = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t showTask;

// --- display (core 1) state -------------------------------------------------
static esp_lcd_panel_io_handle_t io;
static SemaphoreHandle_t lcdDone;
static uint16_t* rgb;                   // decoded frame, internal DMA RAM, 16-byte aligned
static uint16_t* bar;                   // status line, internal DMA RAM
static uint32_t  lastHash;              // last image checked (Tapo re-sends each one)

// Stats, logged every 5 s. Updated from both cores; a lost count here or there
// on reset is fine for diagnostics.
static volatile uint32_t statFrames, statJpegs, statBytes;
static uint32_t statShown, statTorn, statCheckMs, statDecodeMs;

// ---------------------------------------------------------------------------
// Display: ILI9341 over esp_lcd's SPI panel IO
// ---------------------------------------------------------------------------
// Adafruit_ILI9341's init sequence: cmd, argc (0x80 = wait 150 ms after), args...
static const uint8_t initCmds[] = {
  0xEF, 3, 0x03, 0x80, 0x02,
  0xCF, 3, 0x00, 0xC1, 0x30,
  0xED, 4, 0x64, 0x03, 0x12, 0x81,
  0xE8, 3, 0x85, 0x00, 0x78,
  0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
  0xF7, 1, 0x20,
  0xEA, 2, 0x00, 0x00,
  0xC0, 1, 0x23,                         // PWCTR1
  0xC1, 1, 0x10,                         // PWCTR2
  0xC5, 2, 0x3e, 0x28,                   // VMCTR1
  0xC7, 1, 0x86,                         // VMCTR2
  0x36, 1, 0x28,                         // MADCTL: MV|BGR = landscape (Adafruit rotation 1)
  0x37, 1, 0x00,                         // VSCRSADD
  0x3A, 1, 0x55,                         // PIXFMT 16 bpp
  0xB1, 2, 0x00, 0x18,                   // FRMCTR1
  0xB6, 3, 0x08, 0x82, 0x27,             // DFUNCTR
  0xF2, 1, 0x00,                         // 3Gamma off
  0x26, 1, 0x01,                         // GAMMASET
  0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
  0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
  0x21, 0,                               // INVON: this IPS panel needs inversion (see radio sketch)
  0x11, 0x80,                            // SLPOUT
  0x29, 0x80,                            // DISPON
  0x00
};

static bool IRAM_ATTR onColorDone(esp_lcd_panel_io_handle_t h, esp_lcd_panel_io_event_data_t* e, void* ctx) {
  BaseType_t woken = pdFALSE;
  xSemaphoreGiveFromISR(lcdDone, &woken);
  return woken == pdTRUE;
}

// Send w x h big-endian RGB565 pixels to (x, y). Returns once the DMA is done,
// so the caller may reuse the buffer.
// ponytail: decode waits ~22 ms for the SPI DMA; double-buffer rgb if >15 fps is ever needed
static void lcdBlit(int x, int y, int w, int h, const void* px) {
  uint8_t ca[4] = { x >> 8, x & 0xFF, (x + w - 1) >> 8, (x + w - 1) & 0xFF };
  uint8_t ra[4] = { y >> 8, y & 0xFF, (y + h - 1) >> 8, (y + h - 1) & 0xFF };
  esp_lcd_panel_io_tx_param(io, 0x2A, ca, 4);   // CASET
  esp_lcd_panel_io_tx_param(io, 0x2B, ra, 4);   // RASET
  esp_lcd_panel_io_tx_color(io, 0x2C, px, w * h * 2);
  xSemaphoreTake(lcdDone, portMAX_DELAY);
}

static void lcdInit(void) {
  gpio_reset_pin(TFT_BL);
  gpio_set_direction(TFT_BL, GPIO_MODE_OUTPUT);
  gpio_set_level(TFT_BL, 1);

  spi_bus_config_t bus = {
    .sclk_io_num = TFT_SCLK, .mosi_io_num = TFT_MOSI, .miso_io_num = -1,
    .quadwp_io_num = -1, .quadhd_io_num = -1,
    .max_transfer_sz = IMG_W * IMG_H * 2,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));
  esp_lcd_panel_io_spi_config_t cfg = {
    .cs_gpio_num = TFT_CS, .dc_gpio_num = TFT_DC,
    .spi_mode = 0, .pclk_hz = 40 * 1000 * 1000,   // same 40 MHz Adafruit uses
    .trans_queue_depth = 4, .on_color_trans_done = onColorDone,
    .lcd_cmd_bits = 8, .lcd_param_bits = 8,
  };
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &cfg, &io));

  esp_lcd_panel_io_tx_param(io, 0x01, NULL, 0);     // SWRESET
  vTaskDelay(pdMS_TO_TICKS(150));
  for (const uint8_t* p = initCmds; *p;) {
    uint8_t cmd = *p++, argc = *p++;
    esp_lcd_panel_io_tx_param(io, cmd, p, argc & 0x7F);
    p += argc & 0x7F;
    if (argc & 0x80) vTaskDelay(pdMS_TO_TICKS(150));
  }

  memset(bar, 0, LCD_W * BAR_H * 2);                // clear the screen
  for (int y = 0; y < LCD_H; y += BAR_H) lcdBlit(0, y, LCD_W, BAR_H, bar);
}

// Status line: text size 2 (12x16 px glyphs) at (4, 6), like the Arduino sketch.
static void status(const char* msg, uint16_t color) {
  uint16_t c = BE(color);
  memset(bar, 0, LCD_W * BAR_H * 2);
  int x = 4;
  for (const char* p = msg; *p && x + 12 <= LCD_W; p++, x += 12)
    for (int col = 0; col < 5; col++) {
      uint8_t bits = font[(uint8_t)*p * 5 + col];
      for (int row = 0; row < 8; row++, bits >>= 1)
        if (bits & 1)
          for (int dy = 0; dy < 2; dy++) {
            uint16_t* d = bar + (6 + row * 2 + dy) * LCD_W + x + col * 2;
            d[0] = d[1] = c;
          }
    }
  lcdBlit(0, 0, LCD_W, BAR_H, bar);
}

// ---------------------------------------------------------------------------
// Socket helpers (reader task only). Any failure = drop the connection.
// ---------------------------------------------------------------------------
static bool fill(void) {
  int n = recv(sock, rx, sizeof rx, 0);           // SO_RCVTIMEO = 3 s
  if (n <= 0) { ESP_LOGW(TAG, "recv -> %d, errno %d", n, errno); return false; }   // 0 = camera closed it
  rxPos = 0; rxLen = n; statBytes += n;
  return true;
}

// Read exactly n bytes (or drop them if buf is NULL).
static bool readFully(uint8_t* buf, size_t n) {
  while (n) {
    if (rxPos == rxLen && !fill()) return false;
    size_t k = rxLen - rxPos < n ? rxLen - rxPos : n;
    if (buf) { memcpy(buf, rx + rxPos, k); buf += k; }
    rxPos += k; n -= k;
  }
  return true;
}

static int readByte(void) {
  if (rxPos == rxLen && !fill()) return -1;
  return rx[rxPos++];
}

static bool readLine(char* line, size_t cap) {
  size_t n = 0;
  for (int c; (c = readByte()) != '\n';) {
    if (c < 0) return false;
    if (c != '\r' && n + 1 < cap) line[n++] = c;
  }
  line[n] = 0;
  return true;
}

// Read an RTSP text response (status line + headers + optional body).
// Returns the status code, or -1. Copies the Session id to sessionOut if given.
static int readResponse(char* sessionOut) {
  char line[256];
  int code = -1, contentLen = 0;
  for (bool first = true;; first = false) {
    if (!readLine(line, sizeof line)) return -1;
    if (!line[0]) break;
    if (first) {
      const char* sp = strchr(line, ' ');
      code = sp ? atoi(sp + 1) : -1;
    } else if (!strncasecmp(line, "Content-Length:", 15)) {
      contentLen = atoi(line + 15);
    } else if (sessionOut && !strncasecmp(line, "Session:", 8)) {
      const char* s = line + 8;
      while (*s == ' ') s++;
      size_t n = strcspn(s, ";");                   // drop ";timeout=15"
      if (n >= sizeof session) n = sizeof session - 1;
      memcpy(sessionOut, s, n);
      sessionOut[n] = 0;
    }
  }
  if (contentLen > 0 && !readFully(NULL, contentLen)) return -1;   // SDP etc: not needed
  return code;
}

static bool sendRequest(const char* method, const char* url, const char* extra) {
  char req[512];
  int n = snprintf(req, sizeof req, "%s %s RTSP/1.0\r\nCSeq: %d\r\n%s%s\r\n",
                   method, url, ++cseq, authHdr, extra);
  return n < (int)sizeof req && send(sock, req, n, 0) == n;
}

// ---------------------------------------------------------------------------
// RTSP session (reader task only)
// ---------------------------------------------------------------------------
static bool rtspStart(void) {
  if (sock >= 0) close(sock);
  sock = -1;
  rxPos = rxLen = 0;
  session[0] = 0;
  jpgOk = false;

  struct addrinfo hints = { .ai_family = AF_INET, .ai_socktype = SOCK_STREAM }, *ai;
  if (getaddrinfo(CAM_HOST, "554", &hints, &ai) != 0) { ESP_LOGE(TAG, "Can't resolve " CAM_HOST); return false; }
  sock = socket(AF_INET, SOCK_STREAM, 0);
  struct timeval tv = { .tv_sec = 3 };
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
  int ok = connect(sock, ai->ai_addr, ai->ai_addrlen) == 0;
  freeaddrinfo(ai);
  if (!ok) { ESP_LOGE(TAG, "RTSP connect failed"); return false; }

  char cred[128], b64[128];
  size_t b64len;
  int credLen = snprintf(cred, sizeof cred, "%s:%s", CAM_USER, CAM_PASS);
  mbedtls_base64_encode((unsigned char*)b64, sizeof b64, &b64len, (unsigned char*)cred, credLen);
  snprintf(authHdr, sizeof authHdr, "Authorization: Basic %s\r\n", b64);
  snprintf(rtspUrl, sizeof rtspUrl, "rtsp://%s:554/%s", CAM_HOST, CAM_PATH);

  sendRequest("DESCRIBE", rtspUrl, "Accept: application/sdp\r\n");
  int st = readResponse(NULL);
  if (st != 200) { ESP_LOGE(TAG, "DESCRIBE -> %d", st); return false; }

  // ponytail: Tapo's video track is always "track1"; parse the SDP if another camera needs it
  char url[128], extra[128];
  snprintf(url, sizeof url, "%s/track1", rtspUrl);
  sendRequest("SETUP", url, "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
  st = readResponse(session);
  if (st != 200 || !session[0]) { ESP_LOGE(TAG, "SETUP -> %d", st); return false; }

  snprintf(extra, sizeof extra, "Session: %s\r\nRange: npt=0.000-\r\n", session);
  sendRequest("PLAY", rtspUrl, extra);
  st = readResponse(NULL);
  if (st != 200) { ESP_LOGE(TAG, "PLAY -> %d", st); return false; }

  ESP_LOGI(TAG, "RTSP playing, session %s", session);
  lastKeepalive = lastFrameMs = millis();
  return true;
}

// Consume one interleaved item from the socket. Returns 1 when a complete,
// plausible JPEG is at bufs[wIdx] + segOff (segLen bytes), 0 if not (yet), -1 if
// the connection broke.
//
// Tapo writes its reply to GET_PARAMETER into the MIDDLE of an RTP packet, so
// after a keepalive the "$" framing is off by the reply's length. Anything that
// isn't a sane "$ ch len RTP-v2" header is treated as junk and we scan on for
// the next "$". Replies from servers that send them between packets are text
// without a "$", so they're skipped the same way.
static int pumpRtsp(void) {
  int c = readByte();
  if (c < 0) return -1;
  if (c != '$') { jpgOk = false; return 0; }               // junk: resync

  uint8_t ih[3];
  if (!readFully(ih, 3)) return -1;
  int ch = ih[0], len = ih[1] << 8 | ih[2];
  if (ch > 3) { jpgOk = false; return 0; }                 // stray '$' inside junk
  if (ch != 0) return readFully(NULL, len) ? 0 : -1;       // RTCP / audio

  uint8_t r[12];
  if (len < 20 || !readFully(r, 12)) return len < 20 ? 0 : -1;
  int left = len - 12;
  int csrc = (r[0] & 0x0F) * 4;                            // CSRC list
  int ext  = r[0] & 0x10;                                  // header extension
  if ((r[0] & 0xC0) != 0x80 || csrc + (ext ? 4 : 0) + 8 > left) { jpgOk = false; return 0; }
  bool marker = r[1] & 0x80;
  if (!readFully(NULL, csrc)) return -1;
  left -= csrc;
  if (ext) {
    uint8_t e[4];
    if (!readFully(e, 4)) return -1;
    int n = (e[2] << 8 | e[3]) * 4;
    left -= 4 + n;
    if (left < 8) { jpgOk = false; return 0; }
    if (!readFully(NULL, n)) return -1;
  }

  uint8_t jh[8];                                           // RFC 2435 JPEG header
  if (!readFully(jh, 8)) return -1;
  left -= 8;
  size_t off = (size_t)jh[1] << 16 | jh[2] << 8 | jh[3];
  if (off == 0) { jpgLen = 0; jpgOk = true; }

  uint8_t* jpg = bufs[wIdx];
  if (jpgOk && off == jpgLen && off + left <= JPG_MAX) {
    if (!readFully(jpg + off, left)) return -1;
    jpgLen += left;
  } else {
    jpgOk = false;                                         // gap: drop this frame
    if (!readFully(NULL, left)) return -1;
  }
  if (!marker) return 0;

  statFrames++;
  bool ok = jpgOk;
  jpgOk = false;
  if (!ok) return 0;
  // Tapo re-sends one buffer that its encoder keeps overwriting, so an RTP frame
  // is rarely exactly one JPEG: it may start or end mid-image. Take the first
  // complete SOI..EOI inside it (there's never more than one).
  const uint8_t* s = memmem(jpg, jpgLen, "\xFF\xD8\xFF", 3);
  const uint8_t* e = s ? memmem(s, jpg + jpgLen - s, "\xFF\xD9", 2) : NULL;
  if (!e) return 0;
  segOff = s - jpg;
  segLen = e + 2 - s;
  statJpegs++;
  return 1;
}

// Hand the frame just completed in bufs[wIdx] to the display task, replacing
// any older frame it hasn't taken yet. The reader gets that older buffer next.
static void publishFrame(void) {
  offs[wIdx] = segOff;
  lens[wIdx] = segLen;
  portENTER_CRITICAL(&bufMux);
  int t = readyIdx; readyIdx = wIdx; wIdx = t;
  readyNew = true;
  portEXIT_CRITICAL(&bufMux);
  xTaskNotifyGive(showTask);
}

static void readerTask(void* arg) {
  for (;;) {
    streaming = false;
    while (!rtspStart()) vTaskDelay(pdMS_TO_TICKS(2000));
    streaming = true;

    for (;;) {
      int r = pumpRtsp();                                  // blocks in recv() when idle
      if (r < 0) break;
      if (r) { publishFrame(); lastFrameMs = millis(); }
      if (millis() - lastFrameMs > STALL_MS) break;
      if (millis() - lastKeepalive > KEEPALIVE_MS) {
        char extra[96];
        snprintf(extra, sizeof extra, "Session: %s\r\n", session);
        lastKeepalive = millis();
        sendRequest("GET_PARAMETER", rtspUrl, extra);
      }
    }
    ESP_LOGW(TAG, "Stream lost, reconnecting");
  }
}

// ---------------------------------------------------------------------------
// Decode + draw (display task, core 1)
// ---------------------------------------------------------------------------
// Take the newest frame if there is one. Returns its buffer index, or -1.
static int takeFrame(void) {
  int idx = -1;
  portENTER_CRITICAL(&bufMux);
  if (readyNew) {
    int t = rIdx; rIdx = readyIdx; readyIdx = t;
    readyNew = false;
    idx = rIdx;
  }
  portEXIT_CRITICAL(&bufMux);
  return idx;
}

static uint32_t fnv1a(const uint8_t* p, size_t n) {
  uint32_t h = 2166136261u;
  while (n--) h = (h ^ *p++) * 16777619u;
  return h;
}

static void showFrame(const uint8_t* jpg, size_t len) {
  uint32_t h = fnv1a(jpg, len);
  if (h == lastHash) return;                               // already shown or rejected
  lastHash = h;

  // A torn image (head of one JPEG + tail of another) has the wrong amount of
  // compressed data for its blocks; the decoder doesn't always notice, so walk
  // the Huffman codes first. Matches libjpeg's verdict on 144/144 Tapo samples.
  uint32_t t0 = millis();
  bool whole = jpeg_complete(jpg, len);
  statCheckMs += millis() - t0;
  if (!whole) { statTorn++; return; }

  t0 = millis();
  jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
  cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_BE;           // wire order for the panel
  cfg.scale.width  = IMG_W;
  cfg.scale.height = IMG_H;
  jpeg_dec_handle_t dec;
  if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) return;
  jpeg_dec_io_t jio = { .inbuf = (uint8_t*)jpg, .inbuf_len = len, .outbuf = (uint8_t*)rgb };
  jpeg_dec_header_info_t info;
  int outLen = 0;
  bool ok = jpeg_dec_parse_header(dec, &jio, &info) == JPEG_ERR_OK &&
            jpeg_dec_get_outbuf_len(dec, &outLen) == JPEG_ERR_OK &&
            outLen <= IMG_W * IMG_H * 2 &&
            jpeg_dec_process(dec, &jio) == JPEG_ERR_OK;
  jpeg_dec_close(dec);
  if (!ok) { statTorn++; return; }
  statDecodeMs += millis() - t0;

  lcdBlit(0, IMG_Y, IMG_W, IMG_H, rgb);
  statShown++;
}

static int rssi(void) {
  wifi_ap_record_t ap;
  return esp_wifi_sta_get_ap_info(&ap) == ESP_OK ? ap.rssi : 0;
}

static void displayTask(void* arg) {
  uint32_t lastStat = millis();
  bool wasStreaming = false;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));          // woken by publishFrame()
    int idx = takeFrame();
    if (idx >= 0) showFrame(bufs[idx] + offs[idx], lens[idx]);

    if (streaming != wasStreaming) {
      wasStreaming = streaming;
      if (streaming) status(CAM_HOST, GREEN);
      else           status("Reconnecting...", YELLOW);
    }
    uint32_t now = millis();
    if (now - lastStat >= 5000) {
      float secs = (now - lastStat) / 1000.0f;
      uint32_t checked = statShown + statTorn;               // each new image is checked once
      ESP_LOGI(TAG, "rx %.1f Mbit/s | RTP frames %.1f/s, JPEGs %.1f/s, torn %.1f/s, shown %.1f/s | check %lu ms, decode %lu ms | RSSI %d",
               statBytes * 8 / secs / 1e6, statFrames / secs, statJpegs / secs, statTorn / secs,
               statShown / secs, checked ? statCheckMs / checked : 0,
               statShown ? statDecodeMs / statShown : 0, rssi());
      if (streaming) {
        char s[40];
        snprintf(s, sizeof s, "%s  %.1f fps", CAM_HOST, statShown / secs);
        status(s, GREEN);
      }
      statBytes = statFrames = statJpegs = 0;
      statShown = statTorn = statCheckMs = statDecodeMs = 0;
      lastStat = now;
    }
  }
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
static EventGroupHandle_t wifiEv;

static void onWifi(void* arg, esp_event_base_t base, int32_t id, void* data) {
  if (base == WIFI_EVENT && (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED)) {
    esp_wifi_connect();                                    // (re)connect forever
  } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* e = data;
    ESP_LOGI(TAG, "WiFi OK " IPSTR ", RSSI %d", IP2STR(&e->ip_info.ip), rssi());
    xEventGroupSetBits(wifiEv, 1);
  }
}

// Credentials saved by ES3C28P_radio's on-screen setup, else secrets.h.
static bool wifiStart(void) {
  char ssid[33] = WIFI_SSID, pass[65] = WIFI_PASS;
  nvs_handle_t h;
  if (nvs_open("radio", NVS_READONLY, &h) == ESP_OK) {
    size_t n = sizeof ssid;
    nvs_get_str(h, "wifi_ssid", ssid, &n);
    n = sizeof pass;
    nvs_get_str(h, "wifi_pass", pass, &n);
    nvs_close(h);
  }
  if (!ssid[0]) return false;

  wifiEv = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t ic = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&ic));
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onWifi, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, onWifi, NULL);

  wifi_config_t wc = {0};
  memcpy(wc.sta.ssid, ssid, strnlen(ssid, sizeof wc.sta.ssid));
  memcpy(wc.sta.password, pass, strnlen(pass, sizeof wc.sta.password));
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());
  esp_wifi_set_ps(WIFI_PS_NONE);          // modem sleep would throttle a ~10 Mbit/s stream
  xEventGroupWaitBits(wifiEv, 1, pdFALSE, pdTRUE, portMAX_DELAY);
  return true;
}

// ---------------------------------------------------------------------------
void app_main(void) {
  ESP_LOGI(TAG, "ES3C28P camera viewer (ESP-IDF)");
  lcdDone = xSemaphoreCreateBinary();
  bar = heap_caps_malloc(LCD_W * BAR_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  rgb = heap_caps_aligned_alloc(16, IMG_W * IMG_H * 2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  for (int i = 0; i < 3; i++) bufs[i] = heap_caps_malloc(JPG_MAX, MALLOC_CAP_SPIRAM);
  if (!bar) { ESP_LOGE(TAG, "Out of memory"); return; }
  lcdInit();
  if (!rgb || !bufs[0] || !bufs[1] || !bufs[2]) { status("Out of memory", RED); return; }

  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);

  status("WiFi...", WHITE);
  if (!wifiStart()) { status("No WiFi saved", RED); return; }
  ESP_LOGI(TAG, "free internal %u KB, PSRAM %u KB",
           heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024, heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024);

  status("Connecting camera...", WHITE);
  xTaskCreatePinnedToCore(displayTask, "show", 8192, NULL, 4, &showTask, 1);
  xTaskCreatePinnedToCore(readerTask, "rtsp", 6144, NULL, 5, NULL, 0);
}
