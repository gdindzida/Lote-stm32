#include "system/main.h"
#include "app/app_types.h"
#include "app/data_processor.h"
#include "bsp/dma.h"
#include "bsp/dwt.h"
#include "bsp/gpio.h"
#include "stm32g4xx_hal.h"
#include "system/sysmem.h"
#include "usb/usb_device.h"
#include "usb/usbd_cdc_if.h"
#include <stdbool.h>
#include <string.h>

void SystemClock_Config(void);

// Static memory
volatile WorkPackageType currentWorkType = NO_WORK;
volatile uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
volatile uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
volatile uint32_t rxBufferOffset = 0;

volatile RecvPacketHeader current_packet_header = {0};
volatile RecvPacketHeader previous_packet_header = {0};

static Payload payload = {};

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
  Stack_Paint();
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  DWT_Init();

  bool isFirst = true;

  while (1) {
    __disable_irq();
    volatile WorkPackageType work_package_type = currentWorkType;
    __enable_irq();

    if (work_package_type != NO_WORK) {
      int16_t length = 0;
      if (isFirst == false) {
        process_data(&payload, work_package_type);
        length = sizeof(Payload); // sizeof(PacketHeader) + sizeof(Metadata);
        memcpy(UserTxBufferFS, &payload, length);
        CDC_Transmit_FS(UserTxBufferFS, length);
      } else {
        isFirst = false;
      }
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
  __disable_irq();
  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
    HAL_Delay(500);
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
