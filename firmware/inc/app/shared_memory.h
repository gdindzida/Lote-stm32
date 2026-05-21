#ifndef __SHARED_MEMORY_H__
#define __SHARED_MEMORY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "app/app_types.h"
#include <stdint.h>

extern volatile WorkPackageType currentWorkType;
// NOLINTNEXTLINE
extern uint8_t rxBuffer[APP_RX_DATA_SIZE];
// NOLINTNEXTLINE
extern uint8_t txBuffer[APP_TX_DATA_SIZE];
extern volatile uint32_t rxBufferOffset;

extern volatile RecvPacketHeader currentPacketHeader;

#ifdef __cplusplus
}
#endif

#endif /* __SHARED_MEMORY_H__ */
