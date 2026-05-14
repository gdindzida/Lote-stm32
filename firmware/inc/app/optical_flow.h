#ifndef __OPTICAL_FLOW_H__
#define __OPTICAL_FLOW_H__

#include "app/app_types.h"

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
  float u_sum;
  float v_sum;
  float rx_sum;
  float ry_sum;
  float rx2_sum;
  float ry2_sum;
  float rxv_sum;
  float ryu_sum;
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

#endif /* __OPTICAL_FLOW_H__ */
