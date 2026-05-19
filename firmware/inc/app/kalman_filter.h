#ifndef __KALMAN_FILTER_H__
#define __KALMAN_FILTER_H__

#include "app/app_types.h"
#include "kalman_filter.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _LinearKalmanFilter {
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
} LinearKalmanFilter;

LinearKalmanFilter getKfInstance() {
  LinearKalmanFilter lkf = {0.F, 0.F, 1.F, 0.F, 0.F, 1.F, 0.05F};
  return lkf;
}

void predict(LinearKalmanFilter lkf, float ax, float ay, float dt) {
  // State prediction
  lkf.vx += ax * dt;
  lkf.vy += ay * dt;

  // Covariance prediction
  lkf.P00 += lkf.Q;
  lkf.P11 += lkf.Q;
}

void update(LinearKalmanFilter lkf, float vx_meas, float vy_meas,
            float quality) {
  quality = std::clamp(quality, 0.0f, 1.0f);

  // Adaptive measurement noise
  float R = 0.01f + (1.0f - quality) * 1.0f;

  // Innovation
  float yx = vx_meas - vx;
  float yy = vy_meas - vy;

  // Innovation covariance
  float S00 = P00 + R;
  float S11 = P11 + R;

  // Kalman gain
  float K00 = P00 / S00;
  float K11 = P11 / S11;

  // State update
  vx += K00 * yx;
  vy += K11 * yy;

  // Covariance update
  P00 *= (1.0f - K00);
  P11 *= (1.0f - K11);
}
#ifdef __cplusplus
}
#endif

#endif /* __KALMAN_FILTER_H__ */
