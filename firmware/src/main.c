#include "main.h"
#include "dma.h"
#include "gpio.h"
#include "image.h"
#include "stm32g4xx_hal.h"
#include "string.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <stdlib.h>

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

uint32_t stack_get_total_size(void) {
  return (uint32_t)(&_estack - &_sstack) * sizeof(uint32_t);
}

// Other
void SystemClock_Config(void);

volatile McuState currentState = {WAITING_INPUT};

void process_data(PacketHeader *header, Metadata *metadata,
                  Coordinate *coordinates) {
  metadata->elapsed_time_ms = (DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000));

  header->magic = MAGIC;
  header->length = 0;

  uint32_t currentRxBufferOffset = rxBufferOffset + APP_RX_BUFFER_SIZE;
  currentRxBufferOffset %= APP_RX_DATA_SIZE;
  uint8_t *bufferView = UserRxBufferFS + currentRxBufferOffset;

  uint32_t sum = 0;
  for (int i = 0; i < APP_RX_BUFFER_SIZE; i++) {
    sum += bufferView[i];
  }

  metadata->sum = sum;

  if (sum % 2 == 0) {
    header->length = sizeof(Metadata) + sizeof(Coordinate);
    metadata->num_points = 1;
    coordinates->x = 17;
    coordinates->y = 43;
  } else {
    header->length = sizeof(Metadata);
    metadata->num_points = 0;
  }

  metadata->elapsed_time_ms =
      (DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000)) -
      metadata->elapsed_time_ms;
  metadata->stack_mem_usage = stack_get_peak_usage();
}

void DWT_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  stack_paint();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  DWT_Init();

  // const char hello_msg[] = "Hello, World!\r\n";
  // const char receive_msg[] = "Received a message!\r\n";

  // while (1) {
  //   __disable_irq();
  //   McuState snapshot = currentState;
  //   if (snapshot == SENDING) {
  //     currentState = WAITING_OUT_HELLO;
  //   } else if (snapshot == RECEIVED) {
  //     currentState = WAITING_OUT_RECV;
  //   }
  //   __enable_irq();

  //   if (snapshot == SENDING) {
  //     // currentState = WAITING_OUT_HELLO;
  //     CDC_Transmit_FS((uint8_t *)hello_msg, strlen(hello_msg));
  //   } else if (snapshot == RECEIVED) {
  //     // currentState = WAITING_OUT_RECV;
  //     CDC_Transmit_FS((uint8_t *)receive_msg, strlen(receive_msg));
  //   }
  // }

  PacketHeader header = {};
  Payload payload = {};

  while (1) {
    __disable_irq();
    McuState snapshot = currentState;
    if (snapshot == PROCESS_DATA) {
      currentState = SEND_HEADER;
    } else if (snapshot == SEND_HEADER) {
      currentState = WAITING_OUT_HEADER;
    } else if (snapshot == SEND_DATA) {
      currentState = WAITING_OUT_DATA;
    }
    __enable_irq();

    if (snapshot == PROCESS_DATA) {
      process_data(&header, &payload.metadata, payload.coordinates);
    } else if (snapshot == SEND_HEADER) {
      CDC_Transmit_FS((uint8_t *)&header, sizeof(PacketHeader));
    } else if (snapshot == SEND_DATA) {
      uint16_t length =
          sizeof(Metadata) + (sizeof(Coordinate) * payload.metadata.num_points);
      CDC_Transmit_FS((uint8_t *)&payload, length);
    }
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
}
#endif /* USE_FULL_ASSERT */
