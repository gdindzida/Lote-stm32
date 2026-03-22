#include "dwt.h"
#include "stm32g4xx_hal.h"

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_GetMs(void) {
  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000U);
}
