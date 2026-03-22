#ifndef __DATA_PROCESSOR_H__
#define __DATA_PROCESSOR_H__

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void process_data(PacketHeader *header, Metadata *metadata,
                  Coordinate *coordinates);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_PROCESSOR_H__ */
