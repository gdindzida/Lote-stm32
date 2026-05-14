#include "system/error_handler.h"
#include "stm32g4xx_hal.h"

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* Disable interrupts */
  __disable_irq();

  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    for (volatile int i = 0; i < 500000; i++)
      ;
  }
}
