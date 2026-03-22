#ifndef __DWT_H__
#define __DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void DWT_Init(void);
uint32_t DWT_GetMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __DWT_H__ */
