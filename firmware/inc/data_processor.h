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

struct LSE_data {
  uint8_t N;
  int16_t u_sum;
  int16_t v_sum;
  int32_t rx_sum;
  int32_t ry_sum;
  uint32_t rx2_sum;
  uint32_t ry2_sum;
  int32_t rxv_sum;
  int32_t ryu_sum;
};

struct LSE_solution {
  float tx;
  float ty;
  float theta;
  float u;
  float v;
};

void process_data(Payload *payload, WorkPackageType work_package_type);

#ifdef __cplusplus
}
#endif

#endif /* __DATA_PROCESSOR_H__ */
