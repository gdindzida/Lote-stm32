#ifndef __ASSERT_HANDLER_H__
#define __ASSERT_HANDLER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line);
#endif /* USE_FULL_ASSERT */

#ifdef __cplusplus
}
#endif

#endif /* __ASSERT_HANDLER_H__ */
