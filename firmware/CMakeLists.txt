cmake_minimum_required(VERSION 3.20)
project(Lote-stm32 C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(MCU_FLAGS
    -mcpu=cortex-m4
    -mthumb
    -mfloat-abi=hard
    -mfpu=fpv4-sp-d16
)

add_compile_options(
    ${MCU_FLAGS}
    -ffreestanding
    -Wall
    -Wextra
    -Os
)

add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions>
    $<$<COMPILE_LANGUAGE:CXX>:-fno-rtti>
)

add_definitions(
    -DSTM32G431xx
    -DUSE_HAL_DRIVER
)

add_link_options(
    ${MCU_FLAGS}
    -T${CMAKE_SOURCE_DIR}/ld/linker_stm32g431xx.ld
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -specs=nano.specs
    -specs=nosys.specs
)

include_directories(
    # Application headers
    inc

    # CMSIS core
    drivers/cmsis/include
    drivers/cmsis/device/st/stm32g4xx/include

    # HAL
    drivers/stm32g4xx_hal_driver/inc
    drivers/stm32g4xx_hal_driver/inc/legacy

    # USB Device middleware
    middlewares/st/stm32_usb_device_library/core/inc
    middlewares/st/stm32_usb_device_library/class/cdc/inc
)

# Application sources
set(APP_SOURCES
    src/main.c
    src/data_processor.cpp
    src/stack_monitor.c
    src/dwt.c
    src/dma.c
    src/gpio.c
    src/usb_device.c
    src/usbd_cdc_if.c
    src/usbd_conf.c
    src/usbd_desc.c
    src/stm32g4xx_hal_msp.c
    src/stm32g4xx_it.c
    src/system_stm32g4xx.c
    src/syscall.c
    src/sysmem.c
)

# HAL driver sources
set(HAL_SOURCES
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_rcc.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_rcc_ex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_cortex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_gpio.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_dma.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_dma_ex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_flash.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_flash_ex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_flash_ramfunc.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_pwr.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_pwr_ex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_pcd.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_pcd_ex.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_ll_usb.c
    drivers/stm32g4xx_hal_driver/src/stm32g4xx_hal_exti.c
)

# USB Middleware sources
set(USB_SOURCES
    middlewares/st/stm32_usb_device_library/core/src/usbd_core.c
    middlewares/st/stm32_usb_device_library/core/src/usbd_ctlreq.c
    middlewares/st/stm32_usb_device_library/core/src/usbd_ioreq.c
    middlewares/st/stm32_usb_device_library/class/cdc/src/usbd_cdc.c
)

# Startup
set(STARTUP_SOURCES
    startup/startup_stm32g431xx.s
)

add_executable(firmware.elf
    ${APP_SOURCES}
    ${HAL_SOURCES}
    ${USB_SOURCES}
    ${STARTUP_SOURCES}
)

add_custom_command(TARGET firmware.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
    COMMAND arm-none-eabi-objcopy -O ihex  firmware.elf firmware.hex
    COMMAND arm-none-eabi-size firmware.elf
)

# Suppress warnings in ST-generated and middleware files
set_source_files_properties(
    src/usbd_cdc_if.c
    src/usbd_conf.c
    src/usbd_desc.c
    middlewares/st/stm32_usb_device_library/core/src/usbd_core.c
    middlewares/st/stm32_usb_device_library/core/src/usbd_ctlreq.c
    middlewares/st/stm32_usb_device_library/core/src/usbd_ioreq.c
    middlewares/st/stm32_usb_device_library/class/cdc/src/usbd_cdc.c
    PROPERTIES COMPILE_FLAGS "-Wno-unused-parameter -Wno-missing-field-initializers"
)