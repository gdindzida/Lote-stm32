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

typedef enum {
  WAITING_INPUT,
  PROCESS_DATA,
  SEND_HEADER,
  WAITING_OUT_HEADER,
  SEND_DATA,
  WAITING_OUT_DATA,
} McuState;

typedef struct __attribute__((packed)) {
  uint8_t x;
  uint8_t y;
} Coordinate;

typedef struct __attribute__((packed)) {
  uint32_t elapsed_time_ms;
  uint32_t sum;
  uint16_t num_points;
  float stack_mem_usage;
} Metadata;

typedef struct __attribute__((packed)) {
  Metadata metadata;
  Coordinate coordinates[100];
} Payload;

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t length;
} PacketHeader;

#ifdef __cplusplus
}
#endif

#endif /* __APP_TYPES_H__ */
