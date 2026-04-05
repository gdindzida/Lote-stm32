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
#define SAD_CEILING (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 5)
#define SAD_MAX (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE * 255)

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
  uint16_t magic;
  uint16_t length;
} PacketHeader;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint32_t elapsed_time_ms;
  int16_t sum_u;
  int16_t sum_v;
  uint16_t num_points;
  float stack_mem_usage;
  float heap_mem_usage;
  float tx;
  float ty;
  float theta;
} Metadata;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  PacketHeader header;
  Metadata metadata;
  Coordinate coordinates[121];
} Payload;

#ifdef __cplusplus
}
#endif

#endif /* __APP_TYPES_H__ */
