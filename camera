/*
 * WATT City -- VISION NODE firmware (Seeed XIAO ESP32S3 SENSE)
 *
 * Role: grab camera frames, find the nearest AprilTag (intersection id) + a
 *       rough range, flag near obstacles, and stream the result to the main
 *       board over UART.  Intersection-only: NO camera calibration needed.
 *
 * Board setup (Arduino IDE):
 *   Board: "XIAO_ESP32S3"     PSRAM: "OPI PSRAM" (REQUIRED)
 *
 * Libraries:
 *   esp32 core (esp_camera)
 *   raspiduino/apriltag-esp32   <-- Arduino port of AprilTag 3.0 for ESP32-cam
 *       https://github.com/raspiduino/apriltag-esp32
 *       Open ITS example sketch and confirm the exact header/family-create
 *       names on your installed version -- the calls below follow the standard
 *       AprilTag 3 C API the port exposes, but verify before trusting.
 *
 * Perf budget (from the library's docs): ~150 ms/frame single-thread VGA,
 *   <100 ms HVGA; ~6-10 FPS. Plenty for reading intersections on approach.
 */

#include <Arduino.h>
#include "esp_camera.h"
#include "watt_link.h"

// ---- AprilTag (raspiduino/apriltag-esp32) ----
extern "C" {
  #include "apriltag.h"
  #include "tag36h11.h"      // family: 36h11 (best false-positive rejection)
}

// ---- Seeed XIAO ESP32S3 Sense camera pin map (verify vs current Seeed docs) ----
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

// ---- UART to main board (XIAO Serial1: TX=D6/GPIO43, RX=D7/GPIO44) ----
static const int PIN_MAIN_TX = 43;
static const int PIN_MAIN_RX = 44;
static const uint32_t LINK_BAUD = 115200;

// ---- Tag / lens constants (EDIT to match your build) ----
static const float TAG_SIZE_M = 0.05f;   // printed tag edge length (100 mm)
// Focal length in pixels at the resolution you run. Get it WITHOUT calibration:
//   f_px = (imgWidth/2) / tan(HFOV_deg/2 * PI/180)
// For VGA (640 wide) and ~66 deg HFOV: 320 / tan(33 deg) ~= 493.
// This ONLY affects the range *estimate*, never the id or whether a tag is seen.
static const float FOCAL_PX = 493.0f;
static const float HFOV_DEG = 66.0f;

uint32_t frameSeq = 0;
apriltag_family_t*   tf = nullptr;
apriltag_detector_t* td = nullptr;

struct TagResult { int16_t id; float range_m; float bearing_deg; float sizePx; };

bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.frame_size   = FRAMESIZE_VGA;        // 640x480; drop to HVGA for more FPS
  c.pixel_format = PIXFORMAT_GRAYSCALE;  // detector wants grayscale
  c.fb_location  = CAMERA_FB_IN_PSRAM;
  c.fb_count     = 2;
  c.grab_mode    = CAMERA_GRAB_LATEST;
  return esp_camera_init(&c) == ESP_OK;
}

void apriltagInit() {
  tf = tag36h11_create();
  td = apriltag_detector_create();
  apriltag_detector_add_family(td, tf);
  td->quad_decimate = 2.0f;   // downsample for speed; big tags absorb the range hit
  td->quad_sigma    = 0.0f;
  td->nthreads      = 2;      // ESP32-S3 is dual-core
  td->refine_edges  = 1;
  td->decode_sharpening = 0.25f;
}

// ---- Camera obstacle heuristic (crude but real): dark blob low-center ----
bool cameraObstacle(camera_fb_t* fb) {
  if (!fb || fb->format != PIXFORMAT_GRAYSCALE) return false;
  int w = fb->width, h = fb->height;
  long dark = 0, total = 0;
  for (int y = 2*h/3; y < h; y += 2)
    for (int x = w/4; x < 3*w/4; x += 2) { if (fb->buf[y*w + x] < 50) dark++; total++; }
  return total > 0 && (dark * 100 / total) > 60;
}

// ---- Real AprilTag detection: returns the NEAREST (largest) tag, if any ----
bool detectAprilTag(camera_fb_t* fb, TagResult& out) {
  if (!td || !fb || fb->format != PIXFORMAT_GRAYSCALE) return false;

  image_u8_t im = { (int32_t)fb->width, (int32_t)fb->height, (int32_t)fb->width, fb->buf };
  zarray_t* dets = apriltag_detector_detect(td, &im);

  bool found = false;
  float bestSize = 0;
  for (int i = 0; i < zarray_size(dets); i++) {
    apriltag_detection_t* d;
    zarray_get(dets, i, &d);
    if (d->hamming > 0) continue;           // reject error-corrected (stricter)

    // apparent tag edge length in px = average of the 4 side lengths
    float per = 0;
    for (int k = 0; k < 4; k++) {
      float dx = d->p[(k+1)%4][0] - d->p[k][0];
      float dy = d->p[(k+1)%4][1] - d->p[k][1];
      per += sqrtf(dx*dx + dy*dy);
    }
    float edgePx = per / 4.0f;

    if (edgePx > bestSize) {                 // nearest = biggest in frame
      bestSize = edgePx;
      out.id = (int16_t)d->id;
      out.sizePx = edgePx;
      out.range_m = (edgePx > 1) ? (FOCAL_PX * TAG_SIZE_M / edgePx) : 0.0f;
      out.bearing_deg = (((float)d->c[0] / fb->width) - 0.5f) * HFOV_DEG; // + = right
      found = true;
    }
  }
  apriltag_detections_destroy(dets);
  return found;
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
  Serial.println(cameraInit() ? "[cam] init OK" : "[cam] init FAILED (check PSRAM=OPI)");
  apriltagInit();
  Serial.println("[tag] 36h11 detector ready");
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { delay(5); return; }

  VisionPacket p = {};
  p.frameSeq = ++frameSeq;
  p.obstacle = cameraObstacle(fb) ? 1 : 0;

  TagResult t;
  if (detectAprilTag(fb, t)) {
    p.tagCount = 1; p.tagId = t.id; p.range_m = t.range_m; p.bearing_deg = t.bearing_deg;
  } else {
    p.tagCount = 0; p.tagId = -1;
  }

  esp_camera_fb_return(fb);
  sendVision(p);

  Serial.printf("seq %u  tag %d  range %.2f m  bear %.1f  obs %d\n",
                p.frameSeq, p.tagId, p.range_m, p.bearing_deg, p.obstacle);
}
