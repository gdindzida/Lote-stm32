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

#define HISTOGRAM_SIZE ((SEARCH_SIZE * 2) + 1)
struct Histogram {
  uint8_t colOffsets[HISTOGRAM_SIZE];
  uint8_t rowOffsets[HISTOGRAM_SIZE];
};

struct HistogramUV {
  float u;
  float v;
  int N;
};

void process_data(Payload *payload, WorkPackageType work_package_type);

#ifdef __cplusplus
}
#endif

#endif /* __OPTICAL_FLOW_H__ */
