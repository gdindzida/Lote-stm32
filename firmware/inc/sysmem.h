/**
 ******************************************************************************
 * @file      sysmem.h
 * @brief     Header for sysmem.c – heap memory tracking utilities
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef SYSMEM_H
#define SYSMEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief Returns the peak heap usage since boot.
 *
 * @return Number of bytes that were allocated at peak heap usage.
 */
uint32_t Heap_GetPeakUsage(void);

/**
 * @brief Returns the current heap usage.
 *
 * @return Number of bytes currently allocated on the heap.
 */
uint32_t Heap_GetCurrentUsage(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSMEM_H */
