#ifndef __DATA_PROCESSOR_H__
#define __DATA_PROCESSOR_H__

#include "app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

struct Corner {
  uint8_t row;
  uint8_t col;
  uint32_t score;
};

void process_data(Payload *payload);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_PROCESSOR_H__ */
