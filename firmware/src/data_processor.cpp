#include "data_processor.h"
#include "dwt.h"
#include "stack_monitor.h"
#include "stm32g4xx_hal.h"
#include "usbd_cdc_if.h"

extern "C" void process_data(PacketHeader *header, Metadata *metadata,
                             Coordinate *coordinates) {
  metadata->elapsed_time_ms = DWT_GetMs();

  header->magic = MAGIC;
  header->length = 0;

  uint32_t currentRxBufferOffset = rxBufferOffset + APP_RX_BUFFER_SIZE;
  currentRxBufferOffset %= APP_RX_DATA_SIZE;
  uint8_t *bufferView = UserRxBufferFS + currentRxBufferOffset;

  uint32_t sum = 0;
  for (int i = 0; i < APP_RX_BUFFER_SIZE; i++) {
    sum += bufferView[i];
  }

  metadata->sum = sum;

  if (sum % 2 == 0) {
    header->length = sizeof(Metadata) + sizeof(Coordinate);
    metadata->num_points = 1;
    coordinates->x = 17;
    coordinates->y = 43;
  } else {
    header->length = sizeof(Metadata);
    metadata->num_points = 0;
  }

  metadata->elapsed_time_ms = DWT_GetMs() - metadata->elapsed_time_ms;
  metadata->stack_mem_usage = stack_get_peak_usage();
}
