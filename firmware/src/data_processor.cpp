#include "data_processor.h"
#include "app_types.h"
#include "dwt.h"
#include "image.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_def.h"
#include "sysmem.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"
#include <array>
#include <cstdlib>

#define FAST_THRESHOLD (3)

namespace {

constexpr std::array<Coordinate, (SEARCH_SIZE * 2 + 1) * (SEARCH_SIZE * 2 + 1)>
make_search_indices() {
  std::array<Coordinate, (SEARCH_SIZE * 2 + 1) * (SEARCH_SIZE * 2 + 1)> ret{};

  int16_t index = 0;
  for (int16_t i = -SEARCH_SIZE; i <= SEARCH_SIZE; i++) {
    for (int16_t j = -SEARCH_SIZE; j <= SEARCH_SIZE; j++) {
      ret[index++] = {i, j};
    }
  }

  return ret;
}

constexpr auto search_indices = make_search_indices();

inline uint32_t coord_to_index(uint8_t row, uint8_t col, uint8_t width) {
  return (row * width) + col;
}

void process_stride(const uint8_t *img_stride, LSE_data &lse_data,
                    uint8_t stride_row_offset) {

  for (int i = 0; i < ((IMG_W / SAD_BLOCK_SIZE) - 1); ++i) {
    int min_sad2 = SAD_MAX;
    min_sad2 *= min_sad2;
    int16_t u = 0;
    int16_t v = 0;
    int sad2 = 0;

    Coordinate current_start = {
        static_cast<int16_t>(SEARCH_SIZE + (SAD_BLOCK_SIZE * i)),
        static_cast<int16_t>(SEARCH_SIZE)};

    for (const Coordinate &search_index : search_indices) {
      Coordinate candidate_start = {
          static_cast<int16_t>(current_start.col + search_index.col),
          static_cast<int16_t>(current_start.row + search_index.row)};

      for (int row = 0; row < SAD_BLOCK_SIZE; ++row) {
        for (int col = 0; col < SAD_BLOCK_SIZE; ++col) {
          int current_val = static_cast<int>(img_stride[coord_to_index(
              current_start.row + row, current_start.col + col, IMG_W)]);
          int candidate_val = static_cast<int>(img_stride[coord_to_index(
              candidate_start.row + row, candidate_start.col + col, IMG_W)]);

          sad2 += (candidate_val - current_val) * (candidate_val - current_val);
        }
      }

      if (sad2 < min_sad2) {
        min_sad2 = sad2;
        u = search_index.col;
        v = search_index.row;
      }
    }

    if (min_sad2 < SAD_CEILING * SAD_CEILING) {
      uint8_t gx =
          SEARCH_SIZE + (SAD_BLOCK_SIZE / 2 - 1) + (SAD_BLOCK_SIZE * i);
      uint8_t gy = SEARCH_SIZE + (SAD_BLOCK_SIZE / 2 - 1) + stride_row_offset;

      uint32_t rx = gx - CENTER_COL;
      uint32_t ry = gy - CENTER_ROW;

      lse_data.N++;
      lse_data.u_sum += u;
      lse_data.v_sum += v;
      lse_data.rx_sum += rx;
      lse_data.ry_sum += ry;
      lse_data.rx2_sum += rx * rx;
      lse_data.ry2_sum += ry * ry;
      lse_data.rxv_sum += rx * v;
      lse_data.ryu_sum += ry * u;
    }
  }
}

LSE_solution solve_lse(LSE_data &lse_data) {
  LSE_solution sol{};
  sol.theta =
      static_cast<float>((lse_data.N * (lse_data.ryu_sum - lse_data.rxv_sum)) -
                         (lse_data.ry_sum * lse_data.u_sum) +
                         (lse_data.rx_sum * lse_data.v_sum)) /
      static_cast<float>(lse_data.rx2_sum + lse_data.ry2_sum -
                         (lse_data.N * (lse_data.rx2_sum + lse_data.ry2_sum)));

  sol.tx = (static_cast<float>(lse_data.u_sum) +
            (static_cast<float>(lse_data.ry_sum) * sol.theta)) /
           static_cast<float>(lse_data.N);

  sol.ty = (static_cast<float>(lse_data.v_sum) -
            (static_cast<float>(lse_data.rx_sum) * sol.theta)) /
           static_cast<float>(lse_data.N);

  return sol;
}

} // namespace

extern "C" void process_data(Payload *payload,
                             WorkPackageType work_package_type) {
  uint32_t start_cycles = DWT_GetCycles();

  payload->header.magic = MAGIC;
  payload->header.length = 0;

  uint8_t *bufferView = UserRxBufferFS;
  if (work_package_type == PROCESS_RX_2) {
    bufferView += APP_RX_BUFFER_SIZE;
  }

  LSE_data lse_data{};

  process_stride(bufferView, lse_data, 0);
  process_stride(bufferView + IMG_W, lse_data, SAD_BLOCK_SIZE);
  process_stride(bufferView + (IMG_W * 2), lse_data, SAD_BLOCK_SIZE * 2);
  process_stride(bufferView + (IMG_W * 3), lse_data, SAD_BLOCK_SIZE * 3);
  process_stride(bufferView + (IMG_W * 4), lse_data, SAD_BLOCK_SIZE * 4);
  process_stride(bufferView + (IMG_W * 5), lse_data, SAD_BLOCK_SIZE * 5);
  process_stride(bufferView + (IMG_W * 6), lse_data, SAD_BLOCK_SIZE * 6);
  process_stride(bufferView + (IMG_W * 7), lse_data, SAD_BLOCK_SIZE * 7);
  process_stride(bufferView + (IMG_W * 8), lse_data, SAD_BLOCK_SIZE * 8);
  process_stride(bufferView + (IMG_W * 9), lse_data, SAD_BLOCK_SIZE * 9);
  process_stride(bufferView + (IMG_W * 10), lse_data, SAD_BLOCK_SIZE * 10);

  LSE_solution solution = solve_lse(lse_data);

  payload->metadata.sum_u = lse_data.u_sum;
  payload->metadata.sum_v = lse_data.v_sum;
  payload->metadata.num_points = lse_data.N;
  payload->metadata.tx = solution.tx;
  payload->metadata.ty = solution.ty;
  payload->metadata.theta = solution.theta;

  payload->header.length = sizeof(Metadata);

  uint32_t elapsed_cycles = DWT_GetCycles() - start_cycles;
  payload->metadata.elapsed_time_ms =
      elapsed_cycles / (HAL_RCC_GetHCLKFreq() / 1000U);
  payload->metadata.stack_mem_usage = Stack_GetPeakUsage();
  payload->metadata.heap_mem_usage = Heap_GetPeakUsage();
}
