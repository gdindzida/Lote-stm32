#include "app/optical_flow.h"
#include "app/app_types.h"
#include "app/image.h"
#include "app/shared_memory.h"
#include "bsp/dwt.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "system/sysmem.h"
#include "usb/usbd_cdc_if.h"
#include "usbd_def.h"
#include <array>
#include <assert.h>
#include <cstdint>
#include <cstdlib>

namespace {

// debug values
int32_t currentOffset;
int32_t maxCol;
int32_t maxRow;
int32_t minCol;
int32_t minRow;

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
  // debug values
  assert((row + currentOffset) < maxRow);
  assert((row + currentOffset) >= minRow);
  assert(col < maxCol);
  assert(col >= minCol);

  return (row * width) + col;
}

void process_stride(volatile const uint8_t *curr_img_stride,
                    volatile const uint8_t *prev_img_stride, Histogram &hist,
                    Payload *payload, int32_t &index) {

  for (int blockIndex = 0; blockIndex < NUMBER_OF_BLOCKS_PER_STRIDE;
       ++blockIndex) {
    // debug values
    minCol = blockIndex * SAD_BLOCK_SIZE;
    maxCol = (blockIndex + 2) * SAD_BLOCK_SIZE;

    volatile int min_sad = SAD_MAX;

    volatile int16_t colOffset = 0;
    volatile int16_t rowOffset = 0;

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
    float mean = static_cast<float>(psum) / 64;
    float variance = (static_cast<float>(psum2) / 64) - (mean * mean);

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

          int diff = candidate_val - current_val;
          if (diff < 0) {
            diff *= -1;
          }

          sad += diff;
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
    return {sumU / static_cast<float>(nU), sumV / static_cast<float>(nV), N};
  }

  return {0, 0, 0};
}

} // namespace

extern "C" void process_data(Payload *payload,
                             WorkPackageType work_package_type) {
  uint32_t start_cycles = DWT_GetCycles();

  payload->header.magic = MAGIC;
  payload->header.length = 0;

  // Get pointer to the current buffer slot
  volatile uint8_t *currbufferView = UserRxBufferFS;
  if (work_package_type == PROCESS_RX_2) {
    currbufferView += APP_RX_BUFFER_SIZE;
  }

  volatile uint8_t *prevbufferView = currbufferView + APP_RX_BUFFER_SIZE;
  if (work_package_type == PROCESS_RX_2) {
    prevbufferView = UserRxBufferFS;
  }

  Histogram hist{};

  int32_t coordIndex = 0;
  for (int strideIndex = 0; strideIndex < NUMBER_OF_STRIDES; strideIndex++) {
    // debug values
    maxRow = SAD_BLOCK_SIZE * (strideIndex + 2);
    minRow = SAD_BLOCK_SIZE * strideIndex;
    currentOffset = SAD_BLOCK_SIZE * strideIndex;

    process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   prevbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   hist, payload, coordIndex);
  }

  HistogramUV histUV = findMax(hist);

  payload->metadata.sum_u = 0;
  payload->metadata.sum_v = 0;

  payload->metadata.num_points = histUV.N;
  payload->metadata.vx =
      MAX(MIN(-histUV.u * current_packet_header.h /
                  (current_packet_header.fx * current_packet_header.dt),
              1),
          -1);
  payload->metadata.vy =
      MAX(MIN(-histUV.v * current_packet_header.h /
                  (current_packet_header.fy * current_packet_header.dt),
              1),
          -1);
  payload->metadata.omega = 0;

  payload->header.length = sizeof(Metadata) + (121 * sizeof(Coordinate));

  uint32_t elapsed_cycles = DWT_GetCycles() - start_cycles;
  payload->metadata.elapsed_time_ms =
      elapsed_cycles / (HAL_RCC_GetHCLKFreq() / 1000U);
  payload->metadata.stack_mem_usage = Stack_GetPeakUsage();
  payload->metadata.heap_mem_usage = Heap_GetPeakUsage();
}
