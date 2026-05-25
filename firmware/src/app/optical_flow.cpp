#include "app/optical_flow.h"
#include "app/app_types.h"
#include "app/image.h"
#include "app/shared_memory.h"
#include "bsp/dwt.h"
#include "system/sysmem.h"
#include "usbd_def.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>

namespace {

// debug measuring
volatile uint32_t startOfScope;
volatile uint32_t durationOfScope;
volatile uint32_t sumOfScope = 0;

constexpr std::array<Coordinate, (SEARCH_SIZE * 2 + 1) * (SEARCH_SIZE * 2 + 1)>
make_search_indices() {
  std::array<Coordinate, (SEARCH_SIZE * 2 + 1) * (SEARCH_SIZE * 2 + 1)> ret{};

  int16_t index = 0;
  for (int16_t i = -SEARCH_SIZE; i <= SEARCH_SIZE; i++) {
    for (int16_t j = -SEARCH_SIZE; j <= SEARCH_SIZE; j++) {
      ret[index++] = {i, j, false};
    }
  }

  return ret;
}

constexpr auto search_indices = make_search_indices();

inline uint32_t coord_to_index(uint8_t row, uint8_t col, uint8_t width) {

  return (row * width) + col;
}

void process_stride(const uint8_t *currImgStride, const uint8_t *prevImgStride,
                    Histogram &hist, Payload *payload, int32_t &index) {

  for (int blockIndex = 0; blockIndex < NUMBER_OF_BLOCKS_PER_STRIDE;
       ++blockIndex) {

    uint32_t min_sad = SAD_MAX;

    int16_t colOffset = 0;
    int16_t rowOffset = 0;

    Coordinate previousStart = {
        static_cast<int16_t>(SEARCH_SIZE),
        static_cast<int16_t>(SEARCH_SIZE + (SAD_BLOCK_SIZE * blockIndex)),
        false};

    int psum = 0;
    int psum2 = 0;
    for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
      for (int col = 0; col < SAD_BLOCK_SIZE; ++col) {
        int currentVal = static_cast<int>(prevImgStride[coord_to_index(
            previousStart.row + row, previousStart.col + col, IMG_W)]);
        psum += currentVal;
        psum2 += currentVal * currentVal;
      }
    }
    float mean = static_cast<float>(psum) / (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE);
    float variance =
        (static_cast<float>(psum2) / (SAD_BLOCK_SIZE * SAD_BLOCK_SIZE)) -
        (mean * mean);

    payload->coordinates[index] = {rowOffset, colOffset, false};

    if (variance < VAR_MIN) {
      index++;
      continue;
    }

    for (const Coordinate &search_index : search_indices) {
      Coordinate candidateStart = {
          static_cast<int16_t>(previousStart.row + search_index.row),
          static_cast<int16_t>(previousStart.col + search_index.col), false};

      uint32_t sad = 0;
      for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
        uint32_t candidateIndex =
            coord_to_index(candidateStart.row + row, candidateStart.col, IMG_W);
        uint32_t prevIndex =
            coord_to_index(previousStart.row + row, previousStart.col, IMG_W);
        // sad =
        //     __USADA8(static_cast<uint32_t>(prevImgStride[prevIndex]),
        //              static_cast<uint32_t>(currImgStride[candidateIndex]),
        //              sad);
        // sad =
        //     __USADA8(static_cast<uint32_t>(
        //                  prevImgStride[prevIndex + (SAD_BLOCK_SIZE / 2)]),
        //              static_cast<uint32_t>(
        //                  currImgStride[candidateIndex + (SAD_BLOCK_SIZE /
        //                  2)]),
        //              sad);
        sad = __USADA8(
            *reinterpret_cast<const uint32_t *>(&prevImgStride[prevIndex]),
            *reinterpret_cast<const uint32_t *>(&currImgStride[candidateIndex]),
            sad);
        sad =
            __USADA8(*reinterpret_cast<const uint32_t *>(
                         &prevImgStride[prevIndex + (SAD_BLOCK_SIZE / 2)]),
                     *reinterpret_cast<const uint32_t *>(
                         &currImgStride[candidateIndex + (SAD_BLOCK_SIZE / 2)]),
                     sad);
      }

      if (sad < min_sad) {
        min_sad = sad;
        colOffset = search_index.col;
        rowOffset = search_index.row;
      }
    }

    payload->coordinates[index] = {rowOffset, colOffset, false};
    index++;

    if (min_sad < SAD_CEILING) {
      hist.colOffsets[colOffset + SEARCH_SIZE]++;
      hist.rowOffsets[rowOffset + SEARCH_SIZE]++;
      payload->coordinates[index - 1].valid = true;
    }
  }
}

HistogramUV findMax(Histogram &hist) {
  uint8_t maxIndexCol = 0;
  uint8_t maxCountCol = hist.colOffsets[0];
  uint8_t maxIndexRow = 0;
  uint8_t maxCountRow = hist.rowOffsets[0];

  int N = hist.rowOffsets[0];

  for (int histIndex = 1; histIndex < HISTOGRAM_SIZE; histIndex++) {
    if (hist.colOffsets[histIndex] > maxCountCol) {
      maxIndexCol = histIndex;
      maxCountCol = hist.colOffsets[histIndex];
    }
    if (hist.rowOffsets[histIndex] > maxCountRow) {
      maxIndexRow = histIndex;
      maxCountRow = hist.rowOffsets[histIndex];
    }

    N += hist.rowOffsets[histIndex];
  }

  int histMinColIndex = MAX(maxIndexCol - 1, 0);
  int histMaxColIndex = MIN(maxIndexCol + 1, HISTOGRAM_SIZE - 1);

  int histMinRowIndex = MAX(maxIndexRow - 1, 0);
  int histMaxRowIndex = MIN(maxIndexRow + 1, HISTOGRAM_SIZE - 1);

  float sumU = 0.F;
  uint8_t nU = 0;
  float sumV = 0.F;
  uint8_t nV = 0;

  for (int histIndex = histMinColIndex; histIndex <= histMaxColIndex;
       ++histIndex) {
    sumU += static_cast<float>(hist.colOffsets[histIndex]) *
            static_cast<float>(histIndex - SEARCH_SIZE);

    nU += hist.colOffsets[histIndex];
  }

  for (int histIndex = histMinRowIndex; histIndex <= histMaxRowIndex;
       ++histIndex) {
    sumV += static_cast<float>(hist.rowOffsets[histIndex]) *
            static_cast<float>(histIndex - SEARCH_SIZE);

    nV += hist.rowOffsets[histIndex];
  }

  if (nU > 0 && nV > 0) {
    return {sumU / static_cast<float>(nU), sumV / static_cast<float>(nV), N,
            static_cast<float>(nU * nV) / static_cast<float>(N * N)};
  }

  return {0, 0, 0, 0.F};
}

void predict(LinearKalmanFilter &lkf, float dt) {
  // State prediction using modeled acceleration (no IMU input)
  // [vx; ax] transition: F = [1 dt; 0 1]
  lkf.vx += lkf.ax * dt;
  lkf.vy += lkf.ay * dt;
  // ax, ay remain unchanged (constant acceleration model)

  float dt2 = dt * dt;

  // Covariance prediction for x-subsystem: P_new = F*P*F^T + Q
  // P00_new = P00 + 2*dt*P02 + dt²*P22 + Qv
  // P02_new = P02 + dt*P22
  // P22_new = P22 + Qa
  float P00x_new = lkf.P00 + 2.F * dt * lkf.P02 + dt2 * lkf.P22 + lkf.Qv;
  float P02x_new = lkf.P02 + dt * lkf.P22;
  float P22x_new = lkf.P22 + lkf.Qa;
  lkf.P00 = P00x_new;
  lkf.P02 = P02x_new;
  lkf.P22 = P22x_new;

  // Covariance prediction for y-subsystem (same structure)
  float P11y_new = lkf.P11 + 2.F * dt * lkf.P13 + dt2 * lkf.P33 + lkf.Qv;
  float P13y_new = lkf.P13 + dt * lkf.P33;
  float P33y_new = lkf.P33 + lkf.Qa;
  lkf.P11 = P11y_new;
  lkf.P13 = P13y_new;
  lkf.P33 = P33y_new;
}

void update(LinearKalmanFilter &lkf, float vx_meas, float vy_meas,
            float quality) {
  quality = std::clamp(quality, 0.0F, 1.0F);

  // Adaptive measurement noise
  float R = 0.01F + ((1.0F - quality) * 1.0F);

  // Innovation (optical flow measurement only; H = [1 0] for each subsystem)
  float ix = vx_meas - lkf.vx;
  float iy = vy_meas - lkf.vy;

  // Innovation covariance: S = H*P*H^T + R = P00 (or P11) + R
  float Sx = lkf.P00 + R;
  float Sy = lkf.P11 + R;

  // Kalman gains for x-subsystem: K = P*H^T / S = [P00; P02] / Sx
  float Kv_x = lkf.P00 / Sx; // gain for vx
  float Ka_x = lkf.P02 / Sx; // gain for ax

  // Kalman gains for y-subsystem: K = [P11; P13] / Sy
  float Kv_y = lkf.P11 / Sy; // gain for vy
  float Ka_y = lkf.P13 / Sy; // gain for ay

  // State update
  lkf.vx += Kv_x * ix;
  lkf.ax += Ka_x * ix;
  lkf.vy += Kv_y * iy;
  lkf.ay += Ka_y * iy;

  // Covariance update: P_new = (I - K*H)*P
  // x-subsystem: P00_new = P00*(1-Kv_x), P02_new = P02*(1-Kv_x),
  //              P22_new = P22 - Ka_x*P02
  float P02x_old = lkf.P02;
  lkf.P22 -= Ka_x * P02x_old;
  lkf.P00 *= (1.0F - Kv_x);
  lkf.P02 *= (1.0F - Kv_x);

  // y-subsystem: P11_new = P11*(1-Kv_y), P13_new = P13*(1-Kv_y),
  //              P33_new = P33 - Ka_y*P13
  float P13y_old = lkf.P13;
  lkf.P33 -= Ka_y * P13y_old;
  lkf.P11 *= (1.0F - Kv_y);
  lkf.P13 *= (1.0F - Kv_y);
}

} // namespace

extern "C" void estimate_optical_flow(Payload *payload,
                                      WorkPackageType work_package_type,
                                      RecvPacketHeader packetHeader) {
  uint32_t startCycles = DWT_GetCycles();

  payload->header.magic = MAGIC;
  payload->header.length = 0;

  static float vxFilt = 0.F;
  static float vyFilt = 0.F;

  // Get pointer to the current buffer slot
  uint8_t *currbufferView = rxBuffer;
  uint8_t *prevbufferView = rxBuffer + APP_RX_BUFFER_SIZE;
  if (work_package_type == PROCESS_RX_2) {
    currbufferView = rxBuffer + APP_RX_BUFFER_SIZE;
    prevbufferView = rxBuffer;
  }

  Histogram hist{};

  int32_t coordIndex = 0;
  for (int strideIndex = 0; strideIndex < NUMBER_OF_STRIDES; strideIndex++) {
    startOfScope = DWT_GetCycles();
    process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   prevbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   hist, payload, coordIndex);
    durationOfScope = DWT_GetCycles() - startOfScope;
    sumOfScope += durationOfScope;
  }

  HistogramUV histUV = findMax(hist);

  // // correcting for roll and pitch
  // float uRot = -packetHeader.w_y * packetHeader.fx;
  // float vRot = packetHeader.w_x * packetHeader.fy;

  // histUV.u -= uRot;
  // histUV.v -= vRot;

  float vxRaw = std::clamp(-histUV.u * packetHeader.h /
                               (packetHeader.fx * packetHeader.dt),
                           -10.F, 10.F);
  float vyRaw = std::clamp(-histUV.v * packetHeader.h /
                               (packetHeader.fy * packetHeader.dt),
                           -10.F, 10.F);

  // Kalman filter: state = [vx, vy, ax, ay]; acceleration modeled, no IMU input
  // Init: {vx, vy, ax, ay, P00, P02, P22, P11, P13, P33, Qv, Qa}
  static LinearKalmanFilter lkf = {0.F, 0.F, 0.F, 0.F, 1.F,   0.F,
                                   1.F, 1.F, 0.F, 1.F, 0.05F, 0.1F};

  predict(lkf, packetHeader.dt);
  update(lkf, vxRaw, vyRaw, histUV.quality);
  vxFilt = lkf.vx;
  vyFilt = lkf.vy;

  payload->metadata.numPoints = histUV.N;
  payload->metadata.vx = vxFilt;
  payload->metadata.vy = vyFilt;
  payload->metadata.debug = histUV.quality;

  payload->header.length =
      sizeof(Metadata) +
      (NUMBER_OF_STRIDES * NUMBER_OF_BLOCKS_PER_STRIDE * sizeof(Coordinate));

  uint32_t elapsedCycles = DWT_GetCycles() - startCycles;
  payload->metadata.elapsedStrideTimeMs =
      (sumOfScope / NUMBER_OF_STRIDES) / (HAL_RCC_GetHCLKFreq() / 1000U);
  sumOfScope = 0;
  payload->metadata.elapsedTotalTimeMs =
      elapsedCycles / (HAL_RCC_GetHCLKFreq() / 1000U);

  payload->metadata.stackMemUsage = Stack_GetPeakUsage();
  payload->metadata.heapMemUsage = Heap_GetPeakUsage();
}
