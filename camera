/*
 * WATT City -- VISION NODE firmware (Seeed XIAO ESP32S3 SENSE)
 *
 * Role: grab camera frames, find the nearest AprilTag, flag near obstacles,
 *       and stream the result to the main board over UART.
 *
 * Board setup in Arduino IDE:
 *   Board: "XIAO_ESP32S3"
 *   PSRAM: "OPI PSRAM"  (REQUIRED -- frame buffer + any tag detector need it)
 *
 * Libraries: "esp32" core by Espressif (provides esp_camera).
 *
 * >>> HONEST NOTE ON APRILTAGS <<<
 * detectAprilTag() below is a HOOK, not a finished detector. A real AprilTag
 * decode (the apriltag C library) is heavy on an ESP32-S3: expect a small tag
 * family (tag16h5 / tag25h9), low resolution, and single-digit FPS. Two paths:
 *   A) On-board: drop a ported apriltag detector into detectAprilTag() and run
 *      it on the GRAYSCALE buffer. Keep FRAMESIZE small (QQVGA/QVGA).
 *   B) Off-board: stream frames over WiFi to a base PC running the standard
 *      apriltag lib, and have the PC send tag ids back (fast + robust).
 * The camera capture, obstacle heuristic, and UART link are real and ready.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "watt_link.h"

// ---- Seeed XIAO ESP32S3 Sense camera pin map (verify against current Seeed docs) ----
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   10
#define SIOD_GPIO_NUM   40
#define SIOC_GPIO_NUM   39
#define Y9_GPIO_NUM     48
#define Y8_GPIO_NUM     11
#define Y7_GPIO_NUM     12
#define Y6_GPIO_NUM     14
#define Y5_GPIO_NUM     16
#define Y4_GPIO_NUM     18
#define Y3_GPIO_NUM     17
#define Y2_GPIO_NUM     15
#define VSYNC_GPIO_NUM  38
#define HREF_GPIO_NUM   47
#define PCLK_GPIO_NUM   13

// ---- UART to main board (XIAO Serial1 on silkscreen TX=D6/GPIO43, RX=D7/GPIO44) ----
static const int PIN_MAIN_TX = 43;   // XIAO TX -> main RX
static const int PIN_MAIN_RX = 44;   // XIAO RX <- main TX (optional)
static const uint32_t LINK_BAUD = 115200;

uint32_t frameSeq = 0;

// ---- one detected tag; extend as needed ----
struct TagResult { int16_t id; float range_m; float bearing_deg; };

bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.frame_size   = FRAMESIZE_QVGA;      // 320x240 -- drop to QQVGA if tag FPS is too low
  c.pixel_format = PIXFORMAT_GRAYSCALE; // tag detection wants grayscale
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.fb_count     = 2;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

// ---- Camera obstacle heuristic (REAL but crude): a large dark blob low-center ----
// Replace with your own logic when you have the map lighting dialed in.
bool cameraObstacle(camera_fb_t* fb) {
  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) return false;
  int w = fb->width, h = fb->height;
  int x0 = w/4, x1 = 3*w/4, y0 = 2*h/3, y1 = h;   // bottom-center region
  long dark = 0, total = 0;
  for (int y = y0; y < y1; y += 2)
    for (int x = x0; x < x1; x += 2) {
      if (fb->buf[y*w + x] < 50) dark++;
      total++;
    }
  return total > 0 && (dark * 100 / total) > 60;   // >60% very-dark pixels
}

// ---- AprilTag HOOK: returns true and fills 'out' when a tag is found ----
bool detectAprilTag(camera_fb_t* fb, TagResult& out) {
  // TODO: run your apriltag detector on fb->buf (grayscale, fb->width x fb->height).
  //   - pick tag16h5 or tag25h9 for speed
  //   - map the detected tag center-x to bearing_deg (see below)
  //   - estimate range from tag pixel size + known physical tag size
  //
  // Placeholder so the link is testable end-to-end:
  (void)fb;
  out.id = -1; out.range_m = 0; out.bearing_deg = 0;
  return false;
}

// ---- UART framer: send VisionPacket to main board ----
void sendVision(const VisionPacket& p) {
  uint8_t len = sizeof(VisionPacket);
  uint8_t chk = watt_checksum(MSG_VISION, (const uint8_t*)&p, len);
  Serial1.write(WATT_SYNC0); Serial1.write(WATT_SYNC1);
  Serial1.write(MSG_VISION); Serial1.write(len);
  Serial1.write((const uint8_t*)&p, len);
  Serial1.write(chk);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(LINK_BAUD, SERIAL_8N1, PIN_MAIN_RX, PIN_MAIN_TX);
  delay(300);
  if (!cameraInit()) { Serial.println("[cam] init FAILED (check PSRAM=OPI)"); }
  else               { Serial.println("[cam] init OK"); }
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(5); return; }

  VisionPacket p = {};
  p.frameSeq = ++frameSeq;
  p.obstacle = cameraObstacle(fb) ? 1 : 0;

  TagResult t;
  if (detectAprilTag(fb, t)) {
    p.tagCount = 1;
    p.tagId = t.id; p.range_m = t.range_m; p.bearing_deg = t.bearing_deg;
  } else {
    p.tagCount = 0; p.tagId = -1;
  }

  esp_camera_fb_return(fb);
  sendVision(p);

  // debug to USB serial
  Serial.printf("seq %u  tag %d  range %.2f  bear %.1f  obs %d\n",
                p.frameSeq, p.tagId, p.range_m, p.bearing_deg, p.obstacle);
}

/*
 * bearing_deg from tag center-x (fill in inside detectAprilTag):
 *   float cx = <tag center x in pixels>;
 *   float halfFOV = 35.0f;              // ~half the OV2640 horizontal FOV, deg
 *   out.bearing_deg = ((cx / fb->width) - 0.5f) * 2.0f * halfFOV;
 *
 * range_m estimate from known tag size:
 *   float f_px = <focal length in pixels, from calibration>;
 *   float tagSize_m = 0.08f;            // your printed tag edge length
 *   out.range_m = f_px * tagSize_m / <tag edge length in pixels>;
 */
