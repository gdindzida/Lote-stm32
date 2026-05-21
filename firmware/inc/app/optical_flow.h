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
  // State: [vx, vy, ax, ay]
  float vx;
  float vy;
  float ax; // modeled acceleration x (not from IMU)
  float ay; // modeled acceleration y (not from IMU)

  // Covariance for x-subsystem [vx, ax]: symmetric 2x2
  float P00; // cov(vx, vx)
  float P02; // cov(vx, ax)
  float P22; // cov(ax, ax)

  // Covariance for y-subsystem [vy, ay]: symmetric 2x2
  float P11; // cov(vy, vy)
  float P13; // cov(vy, ay)
  float P33; // cov(ay, ay)

  // Process noise
  float Qv; // velocity process noise variance
  float Qa; // acceleration process noise variance
};

void process_data(Payload *payload, WorkPackageType work_package_type,
                  RecvPacketHeader packetHeader);

#ifdef __cplusplus
}
#endif

#endif /* __OPTICAL_FLOW_H__ */
