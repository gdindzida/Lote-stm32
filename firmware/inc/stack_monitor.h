#ifndef __STACK_MONITOR_H__
#define __STACK_MONITOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void stack_paint(void);
float stack_get_peak_usage(void);

#ifdef __cplusplus
}
#endif

#endif /* __STACK_MONITOR_H__ */
