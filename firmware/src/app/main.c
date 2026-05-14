#include "system/main.h"
#include "app/app_types.h"
#include "app/data_processor.h"
#include "bsp/dma.h"
#include "bsp/dwt.h"
#include "bsp/gpio.h"
#include "cmsis_gcc.h"
#include "stm32g4xx_hal.h"
#include "system/sysmem.h"
#include "usb/usb_device.h"
#include "usb/usbd_cdc_if.h"
#include <stdbool.h>
#include <string.h>

// Static memory
volatile WorkPackageType currentWorkType = NO_WORK;
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
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
