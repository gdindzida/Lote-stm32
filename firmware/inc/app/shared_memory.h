#ifndef __SHARED_MEMORY_H__
#define __SHARED_MEMORY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app/app_types.h"
#include <stdint.h>

extern volatile WorkPackageType g_currentWorkType;
// NOLINTNEXTLINE
extern uint8_t g_rxBuffer[APP_RX_DATA_SIZE];
// NOLINTNEXTLINE
extern uint8_t g_txBuffer[APP_TX_DATA_SIZE];
extern volatile uint32_t g_rxBufferOffset;

extern volatile RecvPacketHeader g_currentPacketHeader;

#ifdef __cplusplus
}
#endif

#endif /* __SHARED_MEMORY_H__ */
