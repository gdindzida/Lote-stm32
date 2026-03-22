#include "stack_monitor.h"
#include <stdint.h>

#define STACK_PAINT_VALUE 0xDEADBEEFU

// NOLINTNEXTLINE
extern uint32_t _sstack;
// NOLINTNEXTLINE
extern uint32_t _estack;

void stack_paint(void) __attribute__((naked, section(".text.startup")));
void stack_paint(void) {
  __asm volatile("ldr r0, =_sstack          \n"
                 "ldr r1, =0xDEADBEEF       \n"
                 "mov r2, sp                \n"
                 "paint_loop:               \n"
                 "cmp r0, r2                \n"
                 "bge paint_done            \n"
                 "str r1, [r0], #4          \n"
                 "b   paint_loop            \n"
                 "paint_done:               \n"
                 "bx  lr                    \n");
}

float stack_get_peak_usage(void) {
  uint32_t *p = &_sstack;

  while (*p == STACK_PAINT_VALUE && p < &_estack) {
    p++;
  }

  uint32_t total = (uint32_t)(&_estack - &_sstack) * sizeof(uint32_t);
  uint32_t peak = (uint32_t)(&_estack - p) * sizeof(uint32_t);

  return ((float)peak) / ((float)total);
}
