/*
 * ES3C28P_cam  -  Show a Tapo camera's MJPEG RTSP stream (stream8) on the panel.
 *
 * Board : ES3C28P (ESP32-S3, 8MB PSRAM), 2.8" ILI9341 240x320 over SPI.
 *
 * How it works:
 *   RTSP over TCP (Basic auth) -> DESCRIBE / SETUP / PLAY rtsp://CAM/stream8,
 *   RTP packets arrive interleaved on the same socket ("$" framing). Tapo puts a
 *   COMPLETE JPEG file (FFD8..FFD9) behind each RFC 2435 header, so we just strip
 *   the 8-byte header and join fragments - no JPEG header reconstruction needed.
 *   The 640x360 frame is scaled to 320x176 (the decoder needs multiples of 8) by
 *   esp_new_jpeg, Espressif's SIMD decoder that ships with the ESP32 Arduino
 *   core, and drawn in landscape.
 *
 * Two cores:
 *   core 0  readerTask: owns the socket - RTSP setup, keepalive, reconnect, and
 *           frame reassembly - so the TCP stream never stalls while we decode.
 *   core 1  loop(): decodes the newest frame and draws it, plus the status line.
 *   They share three JPEG buffers (triple buffering): the reader always has one
 *   to write, the newest complete frame waits in "ready", and loop() decodes
 *   from a third. Neither side ever waits on the other.
 *
 * Tapo quirks handled:
 *   - It pushes ~28 frames/s (~10 Mbit/s) but the image only changes a few
 *     times a second, and many frames arrive torn. We keep frames that start
 *     FFD8 and end FFD9, and skip ones identical to what's on screen.
 *   - Session timeout is 15 s, so we send GET_PARAMETER every 10 s.
 *
 * WiFi credentials are read from NVS (saved by ES3C28P_radio's on-screen setup).
 * Camera address/login live in secrets.h (gitignored) - see secrets.h.example.
 */

#include <WiFi.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Preferences.h>
#include <base64.h>
#include "esp_jpeg_dec.h"   // esp_new_jpeg, bundled with the ESP32 core
#include "secrets.h"        // CAM_HOST, CAM_USER, CAM_PASS

#ifndef WIFI_SSID
  #define WIFI_SSID ""
  #define WIFI_PASS ""
#endif

// Display pins (same as ES3C28P_radio)
#define TFT_SCLK 12
#define TFT_MOSI 11
#define TFT_MISO 13
#define TFT_CS   10
#define TFT_DC   46
#define TFT_RST  -1
#define TFT_BL   45

Adafruit_ILI9341 tft(&SPI, TFT_DC, TFT_CS, TFT_RST);

const char*    CAM_PATH = "stream8";           // Tapo's MJPEG profile (ONVIF profile_3)
const size_t   JPG_MAX  = 128 * 1024;          // one 640x360 frame is ~42 KB
const int16_t  IMG_W = 320, IMG_H = 176;       // 640x360 scaled; decoder wants multiples of 8
const int16_t  IMG_Y = 32;                     // below the status line
const uint32_t KEEPALIVE_MS = 10000;           // session timeout is 15 s
const uint32_t STALL_MS     = 8000;            // no valid frame this long -> reconnect

// --- reader task (core 0) state --------------------------------------------
WiFiClient rtsp;
String     rtspUrl, authHdr, session;
int        cseq = 0;
uint32_t   lastKeepalive = 0, lastFrameMs = 0;
size_t     jpgLen = 0;
bool       jpgOk  = false;     // current frame has had no gaps so far
volatile bool streaming = false;

// --- triple buffer shared by both cores ------------------------------------
uint8_t*     bufs[3];          // JPEG buffers in PSRAM
size_t       lens[3];
int          wIdx = 0;         // reader writes here           (reader only)
int          readyIdx = 1;     // newest complete frame        (shared, under mux)
int          rIdx = 2;         // loop() decodes from here     (loop only)
bool         readyNew = false; // readyIdx holds a frame loop() hasn't taken
portMUX_TYPE bufMux = portMUX_INITIALIZER_UNLOCKED;

// --- loop (core 1) state ----------------------------------------------------
uint16_t* rgb;                 // decoded 320x176 RGB565 (16-byte aligned)
uint32_t  shownHash = 0;

// Stats, printed every 5 s. Updated from both cores; a lost count here or
// there on reset is fine for diagnostics.
volatile uint32_t statFrames = 0, statValid = 0, statBytes = 0;
uint32_t statShown = 0, statDecodeMs = 0;

// ---------------------------------------------------------------------------
// Socket helpers (reader task only)
// ---------------------------------------------------------------------------
// Read exactly n bytes (or drop them if buf is null). False on timeout/disconnect.
bool readFully(uint8_t* buf, size_t n) {
  static uint8_t sink[512];
  uint32_t t0 = millis();
  while (n) {
    size_t want = buf ? n : min(n, sizeof(sink));
    int got = rtsp.read(buf ? buf : sink, want);
    if (got > 0) {
      n -= got; statBytes += got;
      if (buf) buf += got;
      t0 = millis();
    } else {
      if (!rtsp.connected() || millis() - t0 > 3000) return false;
      vTaskDelay(1);
    }
  }
  return true;
}

// Read an RTSP text response (status line + headers + optional body).
// Returns the status code, or -1 on failure. Headers are an optional output.
int readResponse(String* headers = nullptr) {
  String h;
  int contentLen = 0;
  for (;;) {
    String line = rtsp.readStringUntil('\n');
    if (!rtsp.connected() && line.length() == 0) return -1;
    line.trim();
    if (line.length() == 0) break;
    if (line.startsWith("Content-Length:")) contentLen = line.substring(15).toInt();
    h += line + "\n";
  }
  if (contentLen) readFully(nullptr, contentLen);   // SDP etc: not needed
  if (headers) *headers = h;
  int sp = h.indexOf(' ');
  return sp > 0 ? h.substring(sp + 1, sp + 4).toInt() : -1;
}

void sendRequest(const char* method, const String& url, const String& extra) {
  rtsp.printf("%s %s RTSP/1.0\r\nCSeq: %d\r\n%s%s\r\n",
              method, url.c_str(), ++cseq, authHdr.c_str(), extra.c_str());
}

// ---------------------------------------------------------------------------
// RTSP session (reader task only)
// ---------------------------------------------------------------------------
bool rtspStart() {
  rtsp.stop();
  session = "";
  rtspUrl = String("rtsp://") + CAM_HOST + ":554/" + CAM_PATH;
  authHdr = "Authorization: Basic " + base64::encode(String(CAM_USER) + ":" + CAM_PASS) + "\r\n";

  rtsp.setTimeout(3000);   // affects readStringUntil
  if (!rtsp.connect(CAM_HOST, 554, 3000)) { Serial.println("RTSP connect failed"); return false; }

  String h;
  sendRequest("DESCRIBE", rtspUrl, "Accept: application/sdp\r\n");
  int st = readResponse();
  if (st != 200) { Serial.printf("DESCRIBE -> %d\n", st); return false; }

  // ponytail: Tapo's video track is always "track1"; parse the SDP if another camera needs it
  sendRequest("SETUP", rtspUrl + "/track1", "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n");
  st = readResponse(&h);
  int s = h.indexOf("Session:");
  if (st != 200 || s < 0) { Serial.printf("SETUP -> %d\n", st); return false; }
  session = h.substring(s + 8, h.indexOf('\n', s));
  session.trim();
  int semi = session.indexOf(';');                  // drop ";timeout=15"
  if (semi > 0) session = session.substring(0, semi);

  sendRequest("PLAY", rtspUrl, "Session: " + session + "\r\nRange: npt=0.000-\r\n");
  st = readResponse();
  if (st != 200) { Serial.printf("PLAY -> %d\n", st); return false; }

  Serial.printf("RTSP playing, session %s\n", session.c_str());
  lastKeepalive = lastFrameMs = millis();
  jpgOk = false;
  return true;
}

// Consume one interleaved item from the socket. Returns true when a complete,
// plausible JPEG is sitting in bufs[wIdx][0..jpgLen).
bool pumpRtsp() {
  if (rtsp.available() <= 0) return false;
  int c = rtsp.read();
  if (c == 'R') { readResponse(); return false; }   // reply to our GET_PARAMETER
  if (c != '$') return false;                       // resync

  uint8_t ih[3];
  if (!readFully(ih, 3)) return false;
  uint8_t  ch  = ih[0];
  uint16_t len = (ih[1] << 8) | ih[2];
  if (ch > 3) return false;                         // stray '$' inside data: keep scanning
  if (ch != 0 || len < 20) { readFully(nullptr, len); return false; }  // RTCP / audio

  uint8_t r[12];
  if (!readFully(r, 12)) return false;
  len -= 12;
  bool marker = r[1] & 0x80;
  size_t extra = (r[0] & 0x0F) * 4;                 // CSRC list
  if (extra) { readFully(nullptr, extra); len -= extra; }
  if (r[0] & 0x10) {                                // RTP header extension
    uint8_t e[4];
    readFully(e, 4);
    size_t n = ((e[2] << 8) | e[3]) * 4;
    readFully(nullptr, n);
    len -= 4 + n;
  }

  uint8_t jh[8];                                    // RFC 2435 JPEG header
  readFully(jh, 8);
  len -= 8;
  uint32_t off = ((uint32_t)jh[1] << 16) | (jh[2] << 8) | jh[3];
  if (off == 0) { jpgLen = 0; jpgOk = true; }

  uint8_t* jpg = bufs[wIdx];
  if (jpgOk && off == jpgLen && off + len <= JPG_MAX) {
    readFully(jpg + off, len);
    jpgLen += len;
  } else {
    jpgOk = false;                                  // gap: drop this frame
    readFully(nullptr, len);
  }
  if (!marker) return false;

  statFrames++;
  bool good = jpgOk && jpgLen > 4 &&
              jpg[0] == 0xFF && jpg[1] == 0xD8 &&
              jpg[jpgLen - 2] == 0xFF && jpg[jpgLen - 1] == 0xD9;
  jpgOk = false;
  if (good) statValid++;
  return good;
}

// Hand the frame just completed in bufs[wIdx] to loop(), replacing any older
// frame it hasn't taken yet. The reader gets that older buffer to write next.
void publishFrame() {
  lens[wIdx] = jpgLen;
  portENTER_CRITICAL(&bufMux);
  int t = readyIdx; readyIdx = wIdx; wIdx = t;
  readyNew = true;
  portEXIT_CRITICAL(&bufMux);
}

void readerTask(void*) {
  for (;;) {
    streaming = false;
    while (!rtspStart()) vTaskDelay(pdMS_TO_TICKS(2000));
    streaming = true;

    uint32_t lastYield = millis();
    while (rtsp.connected() && millis() - lastFrameMs < STALL_MS) {
      if (pumpRtsp()) {
        publishFrame();
        lastFrameMs = millis();
      }
      // Sleep a tick when idle, and at least every 20 ms while data keeps
      // flowing, so core 0's idle task can feed the task watchdog.
      if (rtsp.available() <= 0 || millis() - lastYield >= 20) {
        vTaskDelay(1);
        lastYield = millis();
      }
      if (millis() - lastKeepalive > KEEPALIVE_MS) {
        lastKeepalive = millis();
        sendRequest("GET_PARAMETER", rtspUrl, "Session: " + session + "\r\n");
      }
    }
    Serial.println("Stream lost, reconnecting");
  }
}

// ---------------------------------------------------------------------------
// Decode + draw (loop, core 1)
// ---------------------------------------------------------------------------
// Take the newest frame if there is one. Returns its buffer index, or -1.
int takeFrame() {
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

uint32_t fnv1a(const uint8_t* p, size_t n) {
  uint32_t h = 2166136261u;
  while (n--) h = (h ^ *p++) * 16777619u;
  return h;
}

void showFrame(const uint8_t* jpg, size_t len) {
  uint32_t h = fnv1a(jpg, len);
  if (h == shownHash) return;                       // same image as on screen

  uint32_t t0 = millis();
  jpeg_dec_config_t cfg = DEFAULT_JPEG_DEC_CONFIG();
  cfg.output_type = JPEG_PIXEL_FORMAT_RGB565_LE;   // native order for drawRGBBitmap
  cfg.scale.width  = IMG_W;
  cfg.scale.height = IMG_H;
  jpeg_dec_handle_t dec;
  if (jpeg_dec_open(&cfg, &dec) != JPEG_ERR_OK) return;
  jpeg_dec_io_t io = {};
  io.inbuf     = (uint8_t*)jpg;
  io.inbuf_len = len;
  io.outbuf    = (uint8_t*)rgb;
  jpeg_dec_header_info_t info;
  int outLen = 0;
  bool ok = jpeg_dec_parse_header(dec, &io, &info) == JPEG_ERR_OK &&
            jpeg_dec_get_outbuf_len(dec, &outLen) == JPEG_ERR_OK &&
            outLen <= IMG_W * IMG_H * 2 &&
            jpeg_dec_process(dec, &io) == JPEG_ERR_OK;   // fails on a torn frame
  jpeg_dec_close(dec);
  if (!ok) return;
  statDecodeMs += millis() - t0;

  tft.drawRGBBitmap(0, IMG_Y, rgb, IMG_W, IMG_H);
  shownHash = h;
  statShown++;
}

void status(const char* msg, uint16_t color = ILI9341_WHITE) {
  tft.fillRect(0, 0, 320, IMG_Y - 2, ILI9341_BLACK);
  tft.setTextColor(color);
  tft.setTextSize(2);
  tft.setCursor(4, 6);
  tft.print(msg);
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\nES3C28P camera viewer");

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  SPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(1);           // landscape 320x240
  tft.invertDisplay(true);      // this IPS panel needs inversion (see radio sketch)
  tft.fillScreen(ILI9341_BLACK);

  for (auto& b : bufs) b = (uint8_t*)ps_malloc(JPG_MAX);
  rgb = (uint16_t*)jpeg_calloc_align(IMG_W * IMG_H * 2, 16);   // decoder needs 16-byte alignment
  if (!bufs[0] || !bufs[1] || !bufs[2] || !rgb) { status("Out of memory", ILI9341_RED); for (;;) delay(1000); }

  // WiFi: credentials saved by ES3C28P_radio's on-screen setup, else secrets.h.
  Preferences prefs;
  prefs.begin("radio", true);
  String ssid = prefs.getString("wifi_ssid", WIFI_SSID);
  String pass = prefs.getString("wifi_pass", WIFI_PASS);
  prefs.end();
  if (ssid.length() == 0) { status("No WiFi saved", ILI9341_RED); for (;;) delay(1000); }

  status("WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);         // modem sleep would throttle a ~10 Mbit/s stream
  WiFi.begin(ssid.c_str(), pass.c_str());
  while (WiFi.status() != WL_CONNECTED) delay(250);
  Serial.printf("WiFi OK %s, RSSI %d\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  status("Connecting camera...");
  xTaskCreatePinnedToCore(readerTask, "rtsp", 8192, nullptr, 2, nullptr, 0);
}

void loop() {
  int idx = takeFrame();
  if (idx >= 0) showFrame(bufs[idx], lens[idx]);
  else delay(2);

  uint32_t now = millis();
  static uint32_t lastStat = 0;
  static bool wasStreaming = false;
  if (streaming != wasStreaming) {
    wasStreaming = streaming;
    if (streaming) status(CAM_HOST, ILI9341_GREEN);
    else           status("Reconnecting...", ILI9341_YELLOW);
  }
  if (now - lastStat >= 5000) {
    float secs = (now - lastStat) / 1000.0f;
    Serial.printf("rx %.1f Mbit/s | frames %.1f/s, valid %.1f/s, shown %.1f/s | decode %lu ms avg | RSSI %d\n",
                  statBytes * 8 / secs / 1e6, statFrames / secs, statValid / secs, statShown / secs,
                  statShown ? statDecodeMs / statShown : 0, WiFi.RSSI());
    if (streaming) {
      char s[40];
      snprintf(s, sizeof(s), "%s  %.1f fps", CAM_HOST, statShown / secs);
      status(s, ILI9341_GREEN);
    }
    statBytes = statFrames = statValid = 0;
    statShown = statDecodeMs = 0;
    lastStat = now;
  }
}
