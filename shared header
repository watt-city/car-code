// watt_link.h  --  shared message definitions for WATT City
// Include this identical file on BOTH the main board and the XIAO vision node.
//
// Two links exist:
//   1) XIAO  --UART-->  main board      (VisionPacket)   -- AprilTag + camera obstacle
//   2) roadside/other cars --ESP-NOW--> main board (V2XPacket) -- traffic-light state
//
// UART framing (binary, robust to line noise):
//   [0xAA][0x55][TYPE][LEN][ ...payload (LEN bytes)... ][XOR checksum over TYPE..payload]

#pragma once
#include <stdint.h>

// ---- UART frame constants ----
static const uint8_t WATT_SYNC0 = 0xAA;
static const uint8_t WATT_SYNC1 = 0x55;

enum WattMsgType : uint8_t {
  MSG_VISION = 0x01,   // XIAO -> main
};

// ---- Vision packet: XIAO -> main board, over UART ----
// Pass the *nearest / most confident* tag; tagId = -1 when no tag this frame.
#pragma pack(push, 1)
struct VisionPacket {
  uint8_t  tagCount;      // total tags seen this frame
  int16_t  tagId;         // primary tag id, or -1
  float    range_m;       // estimated range to primary tag (m), 0 if unknown
  float    bearing_deg;   // + = tag is to the right of image center
  uint8_t  obstacle;      // 1 = camera sees a near obstacle, else 0
  uint32_t frameSeq;      // increments per frame; lets main board detect stalls
};
#pragma pack(pop)

// ---- V2X packet: roadside unit / other car -> main board, over ESP-NOW ----
enum LightState : uint8_t { LIGHT_RED = 0, LIGHT_YELLOW = 1, LIGHT_GREEN = 2, LIGHT_UNKNOWN = 3 };

#pragma pack(push, 1)
struct V2XPacket {
  uint8_t  intersectionId;   // must match the AprilTag id at that intersection
  uint8_t  lightState;       // LightState
  uint8_t  secondsToChange;  // countdown hint, 0 if not provided
};
#pragma pack(pop)

// ---- tiny XOR checksum used by the UART framer ----
static inline uint8_t watt_checksum(uint8_t type, const uint8_t* payload, uint8_t len) {
  uint8_t c = type ^ len;
  for (uint8_t i = 0; i < len; i++) c ^= payload[i];
  return c;
}
