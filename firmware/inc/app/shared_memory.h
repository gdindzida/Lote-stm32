#ifndef __SHARED_MEMORY_H__
#define __SHARED_MEMORY_H__

#include "app_types.h"
#include <cstdint>

extern volatile WorkPackageType currentWorkType;
// NOLINTNEXTLINE
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
// NOLINTNEXTLINE
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern volatile uint32_t rxBufferOffset;

extern volatile RecvPacketHeader current_packet_header;
extern volatile RecvPacketHeader previous_packet_header;

#endif
