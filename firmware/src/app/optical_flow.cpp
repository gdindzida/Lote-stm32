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

void process_stride(const uint8_t *curr_img_stride,
                    const uint8_t *prev_img_stride, Histogram &hist,
                    Payload *payload, int32_t &index) {

  for (int blockIndex = 0; blockIndex < NUMBER_OF_BLOCKS_PER_STRIDE;
       ++blockIndex) {

    int min_sad = SAD_MAX;

    int16_t colOffset = 0;
    int16_t rowOffset = 0;

    Coordinate current_start = {
        static_cast<int16_t>(SEARCH_SIZE),
        static_cast<int16_t>(SEARCH_SIZE + (SAD_BLOCK_SIZE * blockIndex)),
        false};

    int psum = 0;
    int psum2 = 0;
    for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
      for (int col = 0; col < SAD_BLOCK_SIZE; ++col) {
        int current_val = static_cast<int>(prev_img_stride[coord_to_index(
            current_start.row + row, current_start.col + col, IMG_W)]);
        psum += current_val;
        psum2 += current_val * current_val;
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
      int sad = 0;
      Coordinate candidate_start = {
          static_cast<int16_t>(current_start.row + search_index.row),
          static_cast<int16_t>(current_start.col + search_index.col), false};

      for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
        for (int col = 0; col < SAD_BLOCK_SIZE; ++col) {
          int current_val = static_cast<int>(prev_img_stride[coord_to_index(
              current_start.row + row, current_start.col + col, IMG_W)]);
          int candidate_val = static_cast<int>(curr_img_stride[coord_to_index(
              candidate_start.row + row, candidate_start.col + col, IMG_W)]);

          const int diff = candidate_val - current_val;
          sad += diff < 0 ? -diff : diff;
        }
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

void predict(LinearKalmanFilter &lkf, float ax, float ay, float dt) {
  // State prediction
  lkf.vx += ax * dt;
  lkf.vy += ay * dt;

  // Covariance prediction
  lkf.P00 += lkf.Q;
  lkf.P11 += lkf.Q;
}

void update(LinearKalmanFilter &lkf, float vx_meas, float vy_meas,
            float quality) {
  quality = std::clamp(quality, 0.0F, 1.0F);

  // Adaptive measurement noise
  float R = 0.01F + ((1.0F - quality) * 1.0F);

  // Innovation
  float ix = vx_meas - lkf.vx;
  float iy = vy_meas - lkf.vy;

  // Innovation covariance
  float S00 = lkf.P00 + R;
  float S11 = lkf.P11 + R;

  // Kalman gain
  float K00 = lkf.P00 / S00;
  float K11 = lkf.P11 / S11;

  // State update
  lkf.vx += K00 * ix;
  lkf.vy += K11 * iy;

  // Covariance update
  lkf.P00 *= (1.0F - K00);
  lkf.P11 *= (1.0F - K11);
}

} // namespace

extern "C" void process_data(Payload *payload,
                             WorkPackageType work_package_type,
                             RecvPacketHeader packetHeader) {
  uint32_t startCycles = DWT_GetCycles();

  payload->header.magic = MAGIC;
  payload->header.length = 0;

  static float vxFilt = 0.F;
  static float vyFilt = 0.F;
  static float axFilt = 0.F;
  static float ayFilt = 0.F;

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
    process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   prevbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   hist, payload, coordIndex);
  }
  uint32_t elapsedStrideCycles = DWT_GetCycles() - startCycles;

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

  // Low pass filter
  float alpha = packetHeader.dt / (packetHeader.dt + TAU);

  // vxFilt = (alpha * vxRaw) + ((1.F - alpha) * vxFilt);
  // vyFilt = (alpha * vyRaw) + ((1.F - alpha) * vyFilt);
  axFilt = (alpha * packetHeader.ax) + ((1.F - alpha) * axFilt);
  ayFilt = (alpha * packetHeader.ay) + ((1.F - alpha) * ayFilt);

  // Kalman filter
  static LinearKalmanFilter lkf = {0.F, 0.F, 1.F, 0.F, 0.F, 1.F, 0.05F};

  predict(lkf, axFilt, ayFilt, packetHeader.dt);
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
      elapsedStrideCycles / (HAL_RCC_GetHCLKFreq() / 1000U);
  payload->metadata.elapsedTotalTimeMs =
      elapsedCycles / (HAL_RCC_GetHCLKFreq() / 1000U);

  payload->metadata.stackMemUsage = Stack_GetPeakUsage();
  payload->metadata.heapMemUsage = Heap_GetPeakUsage();
}
