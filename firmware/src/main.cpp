#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define LED_PORT (GPIOC)
#define LED_PIN (GPIO6)

int main() {
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
  gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, GPIO6);

  while (true) {
    gpio_clear(GPIOC, GPIO6); // ON
    for (volatile int i = 0; i < 500000; i++)
      ;
    gpio_set(GPIOC, GPIO6); // OFF
    for (volatile int i = 0; i < 500000; i++)
      ;
  }
}
