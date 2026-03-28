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
#include <cstdint>

#define FAST_THRESHOLD (3)

namespace {

inline uint32_t coord_to_index(uint8_t row, uint8_t col, uint8_t width) {
  return (row * width) + col;
}

using ComparisonFunc = bool (*)(int, int);
bool smaller_then(int value, int threshold) { return value < threshold; }
bool bigger_then(int value, int threshold) { return value > threshold; }
bool null_then(int value, int threshold) {
  UNUSED(value);

  UNUSED(threshold);
  return false;
}

void fast_detector(const uint8_t *img_data, std::array<Corner, 32> &corners,
                   uint32_t &number_of_corners) {
  std::array<uint8_t, 16> circle;
  ComparisonFunc comp_func = null_then;

  for (uint8_t patch_row = 0; patch_row < IMG_H / PATCH_H; patch_row++) {
    for (uint8_t patch_col = 0; patch_col < IMG_W / PATCH_W; patch_col++) {

      uint8_t start_row = MAX(3, patch_row * PATCH_H);
      uint8_t start_col = MAX(3, patch_col * PATCH_W);
      uint8_t end_row = MIN(IMG_H - 3, (patch_row + 1) * PATCH_H);
      uint8_t end_col = MIN(IMG_W - 3, (patch_col + 1) * PATCH_W);

      corners[number_of_corners] = {0, 0, 0};

      bool one_found = false;
      for (uint8_t row = start_row; row < end_row; row++) {
        for (uint8_t col = start_col; col < end_col; col++) {

          // center
          uint8_t center = img_data[coord_to_index(row, col, IMG_W)];
          // top
          circle[1] = img_data[coord_to_index(row - 3, col, IMG_W)];
          // right
          circle[5] = img_data[coord_to_index(row, col + 3, IMG_W)];
          // bottom
          circle[9] = img_data[coord_to_index(row + 3, col, IMG_W)];
          // left
          circle[13] = img_data[coord_to_index(row, col - 3, IMG_W)];

          int top_tresh = center + FAST_THRESHOLD;
          int bottom_tresh = center - FAST_THRESHOLD;

          std::array<bool, 4> bright_conds = {
              circle[1] > top_tresh, circle[5] > top_tresh,
              circle[9] > top_tresh, circle[13] > top_tresh};
          std::array<bool, 4> dark_conds = {
              circle[1] < bottom_tresh, circle[5] < bottom_tresh,
              circle[9] < bottom_tresh, circle[13] < bottom_tresh};

          bool bright = (static_cast<int>(bright_conds[0]) +
                         static_cast<int>(bright_conds[1]) +
                         static_cast<int>(bright_conds[2]) +
                         static_cast<int>(bright_conds[3])) >= 3;
          bool dark = (static_cast<int>(dark_conds[0]) +
                       static_cast<int>(dark_conds[1]) +
                       static_cast<int>(dark_conds[2]) +
                       static_cast<int>(dark_conds[3])) >= 3;

          if (!bright && !dark) {
            continue;
          }

          circle[0] = img_data[coord_to_index(row - 3, col - 1, IMG_W)];
          circle[2] = img_data[coord_to_index(row - 3, col + 1, IMG_W)];
          circle[3] = img_data[coord_to_index(row - 2, col + 2, IMG_W)];
          circle[4] = img_data[coord_to_index(row - 1, col + 3, IMG_W)];
          circle[6] = img_data[coord_to_index(row + 1, col + 3, IMG_W)];
          circle[7] = img_data[coord_to_index(row + 2, col + 2, IMG_W)];
          circle[8] = img_data[coord_to_index(row + 3, col + 1, IMG_W)];
          circle[10] = img_data[coord_to_index(row + 3, col - 1, IMG_W)];
          circle[11] = img_data[coord_to_index(row + 2, col - 2, IMG_W)];
          circle[12] = img_data[coord_to_index(row + 1, col - 3, IMG_W)];
          circle[14] = img_data[coord_to_index(row - 1, col - 3, IMG_W)];
          circle[15] = img_data[coord_to_index(row - 2, col - 2, IMG_W)];

          std::array<bool, 4> &conds_ptr = bright_conds;
          comp_func = bigger_then;
          int threshold = top_tresh;
          if (!bright) {
            comp_func = smaller_then;
            conds_ptr = dark_conds;
            threshold = bottom_tresh;
          }

          std::array<bool, 4> correct_regions = {false};

          if (conds_ptr[0] && conds_ptr[1]) {
            if (comp_func(circle[2], threshold) &&
                comp_func(circle[3], threshold) &&
                comp_func(circle[4], threshold)) {
              correct_regions[0] = true;
            }
          }

          if (conds_ptr[1] && conds_ptr[2]) {
            if (comp_func(circle[6], threshold) &&
                comp_func(circle[7], threshold) &&
                comp_func(circle[8], threshold)) {
              correct_regions[1] = true;
            }
          }

          if (conds_ptr[2] && conds_ptr[3]) {
            if (comp_func(circle[10], threshold) &&
                comp_func(circle[11], threshold) &&
                comp_func(circle[12], threshold)) {
              correct_regions[2] = true;
            }
          }

          if (conds_ptr[3] && conds_ptr[0]) {
            if (comp_func(circle[14], threshold) &&
                comp_func(circle[15], threshold) &&
                comp_func(circle[0], threshold)) {
              correct_regions[3] = true;
            }
          }

          bool is_corner = (static_cast<int>(correct_regions[0]) +
                            static_cast<int>(correct_regions[1]) +
                            static_cast<int>(correct_regions[2]) +
                            static_cast<int>(correct_regions[3])) >= 3;

          if (is_corner) {
            uint32_t score = 0;
            for (int i = 0; i < 16; i++) {
              uint8_t value = circle[i];
              if (value > top_tresh) {
                score += (value - center);
              } else if (value < bottom_tresh) {
                score += (center - value);
              }
            }

            if (score > corners[number_of_corners].score) {
              corners[number_of_corners] = {row, col, score};
              one_found = true;
            }
          }
        }
      }

      if (one_found) {
        number_of_corners++;
      }
    }
  }
}

} // namespace

extern "C" void process_data(Payload *payload,
                             WorkPackageType work_package_type) {
  payload->metadata.elapsed_time_ms = DWT_GetMs();

  payload->header.magic = MAGIC;
  payload->header.length = 0;

  uint8_t *bufferView = UserRxBufferFS;
  if (work_package_type == PROCESS_RX_2) {
    bufferView += APP_RX_BUFFER_SIZE;
  }

  std::array<Corner, 32> corners;
  uint32_t number_of_corners = 0;
  fast_detector(bufferView, corners, number_of_corners);

  payload->metadata.sum = work_package_type;
  payload->metadata.num_points = MIN(32, number_of_corners);
  payload->header.length =
      sizeof(Metadata) + (payload->metadata.num_points * sizeof(Coordinate));

  for (int i = 0; i < payload->metadata.num_points; i++) {
    payload->coordinates[i].row = corners[i].row;
    payload->coordinates[i].col = corners[i].col;
  }

  payload->metadata.elapsed_time_ms =
      DWT_GetMs() - payload->metadata.elapsed_time_ms;
  payload->metadata.stack_mem_usage = Stack_GetPeakUsage();
  payload->metadata.heap_mem_usage = Heap_GetPeakUsage();
}
