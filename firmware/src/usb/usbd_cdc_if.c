#include "usb/usbd_cdc_if.h"
#include "app/app_types.h"
#include "usbd_cdc.h"
#include "usbd_def.h"
#include <stdint.h>

// extern
extern USBD_HandleTypeDef hUsbDeviceFS;

extern volatile WorkPackageType currentWorkType;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];
extern volatile uint32_t rxBufferOffset;

extern volatile RecvPacketHeader current_packet_header;
extern volatile RecvPacketHeader previous_packet_header;

// local
typedef enum { RX_WAIT_FOR_MAGIC, RX_RECEIVING_DATA } RxStateType;
static volatile RxStateType rxState = RX_WAIT_FOR_MAGIC;
static volatile uint32_t rxBytesReceived = 0;

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {CDC_Init_FS, CDC_DeInit_FS,
                                              CDC_Control_FS, CDC_Receive_FS,
                                              CDC_TransmitCplt_FS};

/**
 * @brief  Initializes the CDC media low layer over the FS USB IP
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Init_FS(void) {
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

/**
 * @brief  DeInitializes the CDC media low layer
 * @retval USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_DeInit_FS(void) { return (USBD_OK); }

/**
 * @brief  Manage the CDC class requests
 * @param  cmd: Command code
 * @param  pbuf: Buffer containing command data (request parameters)
 * @param  length: Number of data to be sent (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else
 * USBD_FAIL
 */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length) {
  switch (cmd) {
  case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

  case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

  case CDC_SET_COMM_FEATURE:

    break;

  case CDC_GET_COMM_FEATURE:

    break;

  case CDC_CLEAR_COMM_FEATURE:

    break;

    /*******************************************************************************/
    /* Line Coding Structure */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per
     * second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits */
    /*                                        0 - 1 Stop bit */
    /*                                        1 - 1.5 Stop bits */
    /*                                        2 - 2 Stop bits */
    /* 5      | bParityType |  1   | Number | Parity */
    /*                                        0 - None */
    /*                                        1 - Odd */
    /*                                        2 - Even */
    /*                                        3 - Mark */
    /*                                        4 - Space */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16). */
    /*******************************************************************************/
  case CDC_SET_LINE_CODING:

    break;

  case CDC_GET_LINE_CODING:

    break;

  case CDC_SET_CONTROL_LINE_STATE:

    break;

  case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
}

/**
 * @brief  Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 *
 *         @note
 *         This function will issue a NAK packet on any OUT packet received on
 *         USB endpoint until exiting this function. If you exit this function
 *         before transfer is complete on CDC interface (ie. using DMA
 * controller) it will result in receiving more data while previous ones are
 * still not sent.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else
 * USBD_FAIL
 */
static int8_t CDC_Receive_FS(uint8_t *Buf, uint32_t *Len) {
  switch (rxState) {

  case RX_WAIT_FOR_MAGIC: {
    RecvPacketHeader *receivedHeader =
        (RecvPacketHeader *)(UserRxBufferFS + rxBufferOffset);
    if (receivedHeader->magic == MAGIC) {
      rxBytesReceived = 0;
      rxState = RX_RECEIVING_DATA;
    }

    previous_packet_header = current_packet_header;
    current_packet_header = *receivedHeader;

    /* Always reset RX pointer to start of slot so magic bytes are
     * overwritten by the first image data packet. */
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS + rxBufferOffset);
    break;
  }

  case RX_RECEIVING_DATA: {
    rxBytesReceived += *Len;

    if (rxBytesReceived >= APP_RX_BUFFER_SIZE) {
      rxBytesReceived = 0;
      rxState = RX_WAIT_FOR_MAGIC;

      if (rxBufferOffset == 0) {
        currentWorkType = PROCESS_RX_1;
      } else {
        currentWorkType = PROCESS_RX_2;
      }

      rxBufferOffset = (rxBufferOffset + APP_RX_BUFFER_SIZE) % APP_RX_DATA_SIZE;
      USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS + rxBufferOffset);
    } else {
      USBD_CDC_SetRxBuffer(&hUsbDeviceFS, Buf + *Len);
    }
    break;
  }
  }

  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return USBD_OK;
}

/**
 * @brief  CDC_Transmit_FS
 *         Data to send over USB IN endpoint are sent over CDC interface
 *         through this function.
 *         @note
 *
 *
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
 */
uint8_t CDC_Transmit_FS(uint8_t *Buf, uint16_t Len) {
  uint8_t result = USBD_OK;
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);

  return result;
}

/**
 * @brief  CDC_TransmitCplt_FS
 *         Data transmitted callback
 *
 *         @note
 *         This function is IN transfer complete callback used to inform user
 * that the submitted Data is successfully sent over USB.
 *
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the operation: USBD_OK if all operations are OK else
 * USBD_FAIL
 */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum) {
  uint8_t result = USBD_OK;
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  return result;
}
