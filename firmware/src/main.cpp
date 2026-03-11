#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define LED_PORT (GPIOC)
#define LED_PIN (GPIO6)

// static void clock_setup() {
//   rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_24MHZ]);
// }

// static void gpio_setup() {
//   rcc_periph_clock_enable(RCC_GPIOC);

//   gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
// }

// int main() {
//   rcc_periph_clock_enable(RCC_GPIOA);
//   gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
//   gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, GPIO0);

//   while (true) {
//     gpio_toggle(GPIOA, GPIO0);
//     for (volatile int i = 0; i < 500000; i++)
//       ;
//   }
// }

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

// int main() {
//   // clock_setup();
//   // gpio_setup();

//   // // gpio_set(LED_PORT, LED_PIN);
//   // gpio_clear(LED_PORT, LED_PIN);
//   // while (true) {
//   // }

//   // while (true) {
//   //   gpio_toggle(LED_PORT, LED_PIN);

//   //   for (volatile int i = 0; i < 800000; i++)
//   //     ;
//   // }
//   rcc_periph_clock_enable(RCC_GPIOC);
//   gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO6);
//   gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_LOW, GPIO6);

//   gpio_clear(GPIOC, GPIO6); // LOW = LED ON (active low!)
//   // gpio_set(LED_PORT, LED_PIN);

//   while (true) {
//   }
// }
