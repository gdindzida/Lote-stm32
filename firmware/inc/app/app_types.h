#ifndef __APP_TYPES_H__
#define __APP_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app/image.h"
#include <stdbool.h>
#include <stdint.h>

#define MAGIC (0xABCD)
#define APP_RX_BUFFER_SIZE (9216)
#define APP_RX_DATA_SIZE (2 * APP_RX_BUFFER_SIZE)
#define APP_TX_DATA_SIZE (1024)
#define SAD_BLOCK_SIZE (8)
#define SEARCH_SIZE (4)
#define SEARCH_PATCH_SIZE (SAD_BLOCK_SIZE + (2 * SEARCH_SIZE))
#define SAD_CEILING (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 10)
#define SAD_MAX (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 255)
#define VAR_MIN (30)
#define NUMBER_OF_STRIDES (((IMG_H) / (SAD_BLOCK_SIZE)) - 1)
#define NUMBER_OF_BLOCKS_PER_STRIDE (((IMG_W) / (SAD_BLOCK_SIZE)) - 1)

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

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint16_t magic;  // Magic number (0xABCD)
  uint16_t length; // Payload length
  // Input metadata: pose and timing (28 bytes)
  float dt; // Time since previous frame (seconds)
  float h;  // Height
  float ax; //
  float ay; //
  float az; //
  float wx; // Roll angle (radians)
  float wy; // Pitch angle (radians)
  float wz; // Yaw angle (radians)
  // Camera calibration parameters (24 bytes)
  float fx; // Focal length x (pixels)
  float fy; // Focal length y (pixels)
  float cx; // Principal point x (pixels)
  float cy; // Principal point y (pixels)
  float k1; // Radial distortion coefficient 1
  float k2; // Radial distortion coefficient 2
} RecvPacketHeader;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint16_t magic;  // Magic number (0xABCD)
  uint16_t length; // Payload length
} SendPacketHeader;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint32_t elapsedTotalTimeMs;
  uint32_t elapsedStrideTimeMs;
  uint16_t numPoints;
  float stackMemUsage;
  float heapMemUsage;
  float vx;
  float vy;
  float debug;
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
