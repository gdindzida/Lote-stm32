import serial.tools.list_ports

STM32_USB_VID = 0x0483


def find_stm32_port() -> str:
    """Auto-detect the STM32 CDC serial port by USB vendor ID (0x0483)."""
    for p in serial.tools.list_ports.comports():
        if p.vid == STM32_USB_VID:
            return p.device
    raise RuntimeError("STM32 CDC port not found")
