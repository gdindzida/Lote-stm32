#include "system/assert_handler.h"
#include "system/error_handler.h"
#include "usb/usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line number
   */

  char buffer[256];
  int length = snprintf(buffer, sizeof(buffer),
                        "ASSERT FAILED: file %s on line %lu\r\n", file, line);

  /* Send assert information via USB CDC */
  if (length > 0 && length < (int)sizeof(buffer)) {
    CDC_Transmit_FS((uint8_t *)buffer, (uint16_t)length);
  }

  Error_Handler();
  /* Infinite loop - halt execution on assertion failure */
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    for (volatile int i = 0; i < 500000; i++)
      ;
  }
}
#endif /* USE_FULL_ASSERT */
