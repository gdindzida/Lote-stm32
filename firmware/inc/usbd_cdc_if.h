#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

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
} Metadata;

typedef struct __attribute__((packed)) {
  Metadata metadata;
  Coordinate coordinates[100];
} Payload;

typedef struct __attribute__((packed)) {
  uint16_t magic;
  uint16_t length;
} PacketHeader;

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern volatile uint32_t rxBufferOffset;

uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
