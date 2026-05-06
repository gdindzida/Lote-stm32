#ifndef __APP_TYPES_H__
#define __APP_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define MAGIC (0xABCD)
#define PACKET_SIZE (64)
#define APP_RX_BUFFER_SIZE (9216)
#define APP_RX_DATA_SIZE (2 * APP_RX_BUFFER_SIZE)
#define APP_TX_DATA_SIZE (1024)
#define CENTER_COL (47)
#define CENTER_ROW (47)
#define WORK_QUEUE_SIZE (2)
#define SAD_BLOCK_SIZE (8)
#define SEARCH_SIZE (4)
#define STRIDE_HEIGHT (SAD_BLOCK_SIZE + 2 * SEARCH_SIZE)
#define K_FACTOR (2)
#define SAD_CEILING (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 10)
#define SAD_MAX (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 255)
#define VAR_MIN (30)

// NOLINTNEXTLINE
typedef enum {
  NO_WORK,
  PROCESS_RX_1,
  PROCESS_RX_2,
} WorkPackageType;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  int16_t row;
  int16_t col;
  bool valid;
} Coordinate;

// PacketHeader received FROM host (sent with each frame)
typedef struct __attribute__((packed)) {
  uint16_t magic;  // Magic number (0xABCD)
  uint16_t length; // Payload length
  // Input metadata: pose and timing (28 bytes)
  float dt;  // Time since previous frame (seconds)
  float h;   // Height
  float a_x; //
  float a_y; //
  float a_z; //
  float w_x; // Roll angle (radians)
  float w_y; // Pitch angle (radians)
  float w_z; // Yaw angle (radians)
  // Camera calibration parameters (24 bytes)
  float fx; // Focal length x (pixels)
  float fy; // Focal length y (pixels)
  float cx; // Principal point x (pixels)
  float cy; // Principal point y (pixels)
  float k1; // Radial distortion coefficient 1
  float k2; // Radial distortion coefficient 2
} RecvPacketHeader;

typedef struct __attribute__((packed)) {
  uint16_t magic;  // Magic number (0xABCD)
  uint16_t length; // Payload length
} SendPacketHeader;

// Output metadata sent TO host (processing results)
// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint32_t elapsed_time_ms;
  int16_t sum_u;
  int16_t sum_v;
  uint16_t num_points;
  float stack_mem_usage;
  float heap_mem_usage;
  float vx;
  float vy;
  float omega;
} Metadata;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  SendPacketHeader header;
  Metadata metadata;
  Coordinate coordinates[121];
} Payload;

#ifdef __cplusplus
}
#endif

#endif /* __APP_TYPES_H__ */
