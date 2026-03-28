#ifndef __APP_TYPES_H__
#define __APP_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define MAGIC (0xABCD)
#define PACKET_SIZE (64)
#define APP_RX_BUFFER_SIZE (8192)
#define NUM_OF_PACKETS (APP_RX_BUFFER_SIZE / PACKET_SIZE)
#define APP_RX_DATA_SIZE (2 * APP_RX_BUFFER_SIZE)
#define APP_TX_DATA_SIZE (1024)
#define WORK_QUEUE_SIZE (2)

// NOLINTNEXTLINE
typedef enum {
  NO_WORK,
  PROCESS_RX_1,
  PROCESS_RX_2,
} WorkPackageType;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint8_t row;
  uint8_t col;
} Coordinate;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t length;
} PacketHeader;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  uint32_t elapsed_time_ms;
  uint32_t sum;
  uint16_t num_points;
  float stack_mem_usage;
  float heap_mem_usage;
} Metadata;

// NOLINTNEXTLINE
typedef struct __attribute__((packed)) {
  PacketHeader header;
  Metadata metadata;
  Coordinate coordinates[32];
} Payload;

#ifdef __cplusplus
}
#endif

#endif /* __APP_TYPES_H__ */
