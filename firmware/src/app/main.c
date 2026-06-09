#include "system/main.h"
#include "app/app_types.h"
#include "app/optical_flow.h"
#include "bsp/dma.h"
#include "bsp/dwt.h"
#include "bsp/gpio.h"
#include "cmsis_gcc.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_flash.h"
#include "system/sysmem.h"
#include "usb/usb_device.h"
#include "usb/usbd_cdc_if.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>

// app memory
volatile WorkPackageType g_currentWorkType = NO_WORK;
uint8_t g_rxBuffer[APP_RX_DATA_SIZE];
uint8_t g_txBuffer[APP_TX_DATA_SIZE];
volatile uint32_t g_rxBufferOffset = 0;
volatile RecvPacketHeader g_currentPacketHeader = {0};

int main(void) {
  Stack_Paint();
  HAL_Init();

  __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
  __HAL_FLASH_DATA_CACHE_ENABLE();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_Device_Init();
  DWT_Init();

  // sanity checks
  static_assert(sizeof(Payload) < APP_TX_DATA_SIZE,
                "SANITY CHECK ERROR: Size of payload should be smaller than tx "
                "data size.");

  bool isFirst = true;
  Payload payload = {0};

  while (1) {
    __disable_irq();
    volatile WorkPackageType workPackageType = g_currentWorkType;
    RecvPacketHeader localPacketHeader = g_currentPacketHeader;
    g_currentWorkType = NO_WORK;
    __enable_irq();

    if (workPackageType != NO_WORK) {
      uint16_t length = 0;
      if (isFirst == false) {
        estimateOpticalFlow(&payload, workPackageType, localPacketHeader);
        length = sizeof(Payload);
        memcpy(g_txBuffer, &payload, length);
        CDC_Transmit_FS(g_txBuffer, length);
      } else {
        isFirst = false;
      }
    }
  }
}
