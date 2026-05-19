#ifndef __OPTICAL_FLOW_H__
#define __OPTICAL_FLOW_H__

#include "app/app_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HISTOGRAM_SIZE ((SEARCH_SIZE * 2) + 1)
#define TAU (0.2F) // seconds

struct Corner {
  uint8_t row;
  uint8_t col;
  uint32_t score;
};

struct Histogram {
  uint8_t colOffsets[HISTOGRAM_SIZE];
  uint8_t rowOffsets[HISTOGRAM_SIZE];
};

struct HistogramUV {
  float u;
  float v;
  int N;
  float quality;
};

struct LinearKalmanFilter {
  // State
  float vx;
  float vy;

  // Covariance matrix
  float P00;
  float P01;
  float P10;
  float P11;

  // Process noise
  float Q;
};

void process_data(Payload *payload, WorkPackageType work_package_type,
                  RecvPacketHeader packetHeader);

#ifdef __cplusplus
}
#endif

#endif /* __OPTICAL_FLOW_H__ */
