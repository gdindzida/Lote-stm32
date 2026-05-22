#include "system/main.h"
#include "app/app_types.h"
#include "app/optical_flow.h"
#include "bsp/dma.h"
#include "bsp/dwt.h"
#include "bsp/gpio.h"
#include "cmsis_gcc.h"
#include "stm32g4xx_hal.h"
#include "system/sysmem.h"
#include "usb/usb_device.h"
#include "usb/usbd_cdc_if.h"
#include <assert.h>
#include <stdbool.h>
#include <string.h>

// app memory
volatile WorkPackageType currentWorkType = NO_WORK;
uint8_t rxBuffer[APP_RX_DATA_SIZE];
uint8_t txBuffer[APP_TX_DATA_SIZE];
volatile uint32_t rxBufferOffset = 0;

volatile RecvPacketHeader currentPacketHeader = {0};

static Payload payload = {0};

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

  // sanity checks
  static_assert(sizeof(Payload) < APP_TX_DATA_SIZE,
                "SANITY CHECK ERROR: Size of payload should be smaller than tx "
                "data size.");

  bool isFirst = true;

  while (1) {
    __disable_irq();
    volatile WorkPackageType workPackageType = currentWorkType;
    RecvPacketHeader localPacketHeader = currentPacketHeader;
    currentWorkType = NO_WORK;
    __enable_irq();

    if (workPackageType != NO_WORK) {
      uint16_t length = 0;
      if (isFirst == false) {
        estimate_optical_flow(&payload, workPackageType, localPacketHeader);
        length = sizeof(Payload);
        memcpy(txBuffer, &payload, length);
        CDC_Transmit_FS(txBuffer, length);
      } else {
        isFirst = false;
      }
    }
  }
}
