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
                    volatile const uint8_t *prev_img_stride, LSE_data &lse_data,
                    uint8_t stride_row_offset, Payload *payload,
                    int32_t &index) {

  for (int blockIndex = 0; blockIndex < NUMBER_OF_BLOCKS_PER_STRIDE;
       ++blockIndex) {
    // debug values
    minCol = blockIndex * SAD_BLOCK_SIZE;
    maxCol = (blockIndex + 2) * SAD_BLOCK_SIZE;

    int min_sad = SAD_MAX;
    int16_t colOffset = 0;
    int16_t rowOffset = 0;

    Coordinate current_start = {
        static_cast<int16_t>(SEARCH_SIZE),
        static_cast<int16_t>(SEARCH_SIZE + (SAD_BLOCK_SIZE * blockIndex)),
        false};

    // int psum = 0;
    // int psum2 = 0;
    // for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
    //   for (int col = 0; col < SAD_BLOCK_SIZE; ++col) {
    //     int current_val = static_cast<int>(prev_img_stride[coord_to_index(
    //         current_start.row + row, current_start.col + col, IMG_W)]);
    //     psum += current_val;
    //     psum2 += current_val * current_val;
    //   }
    // }
    // float mean = static_cast<float>(psum) / 64;
    // float variance = (static_cast<float>(psum2) / 64) - (mean * mean);

    payload->coordinates[index] = {rowOffset, colOffset, false};

    // if (variance < VAR_MIN) {
    //   index++;
    //   continue;
    // }

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
      float weight = 1.0f / (1.0f + min_sad);
      payload->coordinates[index - 1].valid = true;
      uint8_t gx = SEARCH_SIZE + (SAD_BLOCK_SIZE / 2 - 1) +
                   (SAD_BLOCK_SIZE * blockIndex);
      uint8_t gy = SEARCH_SIZE + (SAD_BLOCK_SIZE / 2 - 1) + stride_row_offset;

      int32_t rx = static_cast<int32_t>(gx) - CENTER_COL;
      int32_t ry = CENTER_ROW - static_cast<int32_t>(gy);

      lse_data.N++;
      lse_data.u_sum += weight * colOffset;
      lse_data.v_sum += weight * rowOffset;
      lse_data.rx_sum += weight * rx;
      lse_data.ry_sum += weight * ry;
      lse_data.rx2_sum += weight * rx * rx;
      lse_data.ry2_sum += weight * ry * ry;
      lse_data.rxv_sum += weight * rx * rowOffset;
      lse_data.ryu_sum += weight * ry * colOffset;
    }
  }
}

LSE_solution solve_lse(LSE_data &lse_data) {
  LSE_solution sol{};

  if (lse_data.N > 0) {
    sol.theta = ((lse_data.N * (lse_data.ryu_sum - lse_data.rxv_sum)) -
                 (lse_data.ry_sum * lse_data.u_sum) +
                 (lse_data.rx_sum * lse_data.v_sum)) /
                ((lse_data.rx_sum * lse_data.rx_sum) +
                 (lse_data.ry_sum * lse_data.ry_sum) -
                 (lse_data.N * (lse_data.rx2_sum + lse_data.ry2_sum)));

    sol.tx = ((lse_data.u_sum) + ((lse_data.ry_sum) * sol.theta)) /
             static_cast<float>(lse_data.N);

    sol.ty = ((lse_data.v_sum) - ((lse_data.rx_sum) * sol.theta)) /
             static_cast<float>(lse_data.N);

    sol.u = sol.tx -
            ((lse_data.ry_sum) * sol.theta / static_cast<float>(lse_data.N));
    sol.v = sol.ty +
            ((lse_data.rx_sum) * sol.theta / static_cast<float>(lse_data.N));
  }

  return sol;
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

  LSE_data lse_data{};

  int32_t coordIndex = 0;
  for (int strideIndex = 0; strideIndex < NUMBER_OF_STRIDES; strideIndex++) {
    // debug values
    maxRow = SAD_BLOCK_SIZE * (strideIndex + 2);
    minRow = SAD_BLOCK_SIZE * strideIndex;
    currentOffset = SAD_BLOCK_SIZE * strideIndex;

    process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   prevbufferView + (IMG_W * SAD_BLOCK_SIZE * strideIndex),
                   lse_data, SAD_BLOCK_SIZE * strideIndex, payload, coordIndex);
  }

  // process_stride(currbufferView, prevbufferView, lse_data, 0, payload,
  // index); process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 1),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 1), lse_data,
  //                SAD_BLOCK_SIZE, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 2),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 2), lse_data,
  //                SAD_BLOCK_SIZE * 2, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 3),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 3), lse_data,
  //                SAD_BLOCK_SIZE * 3, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 4),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 4), lse_data,
  //                SAD_BLOCK_SIZE * 4, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 5),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 5), lse_data,
  //                SAD_BLOCK_SIZE * 5, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 6),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 6), lse_data,
  //                SAD_BLOCK_SIZE * 6, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 7),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 7), lse_data,
  //                SAD_BLOCK_SIZE * 7, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 8),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 8), lse_data,
  //                SAD_BLOCK_SIZE * 8, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 9),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 9), lse_data,
  //                SAD_BLOCK_SIZE * 9, payload, index);
  // process_stride(currbufferView + (IMG_W * SAD_BLOCK_SIZE * 10),
  //                prevbufferView + (IMG_W * SAD_BLOCK_SIZE * 10), lse_data,
  //                SAD_BLOCK_SIZE * 10, payload, index);

  LSE_solution solution = solve_lse(lse_data);

  payload->metadata.sum_u = lse_data.u_sum;
  payload->metadata.sum_v = lse_data.v_sum;
  payload->metadata.num_points = lse_data.N;
  payload->metadata.vx =
      MAX(MIN(-solution.u * current_packet_header.h /
                  (current_packet_header.fx * current_packet_header.dt),
              1),
          -1);
  payload->metadata.vy =
      MAX(MIN(-solution.v * current_packet_header.h /
                  (current_packet_header.fy * current_packet_header.dt),
              1),
          -1);
  payload->metadata.omega = solution.theta / current_packet_header.dt;

  payload->header.length = sizeof(Metadata) + (121 * sizeof(Coordinate));

  uint32_t elapsed_cycles = DWT_GetCycles() - start_cycles;
  payload->metadata.elapsed_time_ms =
      elapsed_cycles / (HAL_RCC_GetHCLKFreq() / 1000U);
  payload->metadata.stack_mem_usage = Stack_GetPeakUsage();
  payload->metadata.heap_mem_usage = Heap_GetPeakUsage();
}
