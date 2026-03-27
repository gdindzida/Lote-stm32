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
 * @brief Returns the peak heap usage since boot as a fraction of the total
 *        heap size (_Min_Heap_Size from the linker script).
 *
 * @return Value in [0.0, 1.0] representing the peak fraction of heap used.
 */
float Heap_GetPeakUsage(void);

/**
 * @brief Returns the current heap usage.
 *
 * @return Number of bytes currently allocated on the heap.
 */
uint32_t Heap_GetCurrentUsage(void);

/**
 * @brief Fills the stack region (from _sstack up to the current SP) with a
 *        known sentinel value (0xDEADBEEF). Must be called as early as
 *        possible in the startup sequence, before any stack is used.
 */
void Stack_Paint(void);

/**
 * @brief Returns the peak stack usage since boot as a fraction of the total
 *        stack size.
 *
 * @return Value in [0.0, 1.0] representing the peak fraction of stack used.
 */
float Stack_GetPeakUsage(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSMEM_H */
